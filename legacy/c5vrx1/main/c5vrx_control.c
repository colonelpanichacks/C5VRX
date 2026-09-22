/* SPDX-License-Identifier: GPL-3.0-only */

#include "c5vrx_control.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"

#include "c5vrx_adc_dump.h"
#include "c5vrx_cvbs_out.h"
#include "c5vrx_phy_hacks.h"
#include "c5vrx_wbfm_hw.h"
#include "c5vrx_live_pipeline.h"
#include "c5vrx_cvbs_live_out.h"
#include "sdkconfig.h"
#include "c5vrx_wifi5.h"
#include "c5vrx_rf_dump_producer.h"
#include "c5vrx_rf_probes.h"
#include "c5vrx_usb_preview.h"
#include "c5vrx_bench.h"
#include "c5vrx_usb_transport.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_timer.h"

/* Control replies must never depend on newlib/VFS. The direct USB driver is
 * the known-good transport on the physical XIAO and serializes output with IQ
 * packets through c5vrx_usb_transport. */
#define printf c5vrx_usb_printf

typedef struct {
    c5vrx_band_t band;
    uint8_t channel;
    bool ht40;
    bool direct_tune_enabled;
    bool started;
} c5vrx_control_state_t;

static c5vrx_control_state_t s_state;
static c5vrx_rf_source_t s_finite_source;
static c5vrx_rf_source_t s_ring_source;
static bool s_ring_live;
static bool s_finite_live;
static c5vrx_receiver_capabilities_t s_capabilities;
static bool s_wbfm_self_test_passed;
static bool s_parlio_bench_passed;
static bool s_mode0_soak_passed;
static bool s_synthetic_pipeline_passed;
static bool s_physical_burst_gate_passed;
static uint8_t s_ring_block_bench_seen;
static uint8_t s_ring_block_bench_passed;
static size_t s_selected_ring_block_words;

static esp_err_t live_output_with_preview(const uint8_t *samples,
                                          size_t count,
                                          void *context)
{
    /* AV owns the deadline.  The USB tap is invoked only after PARLIO has
     * accepted this block and only while an actively-draining host holds a
     * short lease.  USB can therefore never delay or backpressure AV. */
    c5vrx_cvbs_sync_tracker_t timing;
    c5vrx_live_pipeline_get_timing(&timing);
    /* Never put unlocked/noise waveform on the connector. The independent AV
     * guardian keeps standards-correct black/sync running while timing
     * reacquires, including after an explicit stream epoch change. */
    const bool legal_live = timing.horizontal_locked && timing.vertical_locked &&
        timing.standard != C5VRX_VIDEO_STANDARD_UNKNOWN;
    esp_err_t av_err = ESP_OK;
    if (legal_live) {
        bool starting = !c5vrx_cvbs_live_out_running();
        if (!starting && c5vrx_cvbs_live_out_take_realign_request()) {
            /* A discontinuity made the live start differ from the actual
             * queued filler tail. Keep the rejected block out of PARLIO,
             * then converge deterministically by rebuilding the two-buffer
             * prefix around this newest canonical block. */
            av_err = c5vrx_cvbs_live_out_stop();
            starting = av_err == ESP_OK;
        }
        if (starting)
            av_err = av_err == ESP_OK ?
                c5vrx_cvbs_live_out_start_aligned(samples, count, &timing) :
                av_err;
        if (av_err == ESP_OK && !starting) {
            c5vrx_cvbs_live_out_update_timing(&timing);
            av_err = c5vrx_cvbs_live_out_write(samples, count, context);
        }
    }
    if (av_err == ESP_OK && c5vrx_usb_preview_consumer_active()) {
        c5vrx_usb_preview_ingest_timed(samples, count, &timing);
    }
    return av_err;
}

static void invalidate_rf_capabilities(void)
{
    memset(&s_capabilities, 0, sizeof(s_capabilities));
    s_mode0_soak_passed = false;
    s_physical_burst_gate_passed = false;
    s_ring_block_bench_seen = 0u;
    s_ring_block_bench_passed = 0u;
    s_selected_ring_block_words = 0u;
}

static int ring_block_index(size_t words)
{
    if (words == 1024u) return 0;
    if (words == 2048u) return 1;
    if (words == 4096u) return 2;
    return -1;
}

static void select_measured_ring_block(void)
{
    s_selected_ring_block_words = 0u;
    s_capabilities.pipeline_service_headroom_valid = false;
    s_capabilities.bitscrambler_path_available = false;
    if (s_ring_block_bench_seen != 0x07u) return;
    static const size_t candidates[] = {1024u, 2048u, 4096u};
    for (unsigned i = 0; i < 3u; ++i) {
        if (s_ring_block_bench_passed & (1u << i)) {
            /* Smallest passing block wins: all candidates passed the same
             * throughput/deadline gates, so this minimizes receiver latency. */
            s_selected_ring_block_words = candidates[i];
            s_capabilities.pipeline_service_headroom_valid = true;
            s_capabilities.bitscrambler_path_available =
                s_wbfm_self_test_passed && s_parlio_bench_passed &&
                s_synthetic_pipeline_passed &&
                s_physical_burst_gate_passed;
            return;
        }
    }
}

#ifdef CONFIG_C5VRX_LIVE_CVBS_INVERT
#define C5VRX_CFG_LIVE_INVERT true
#else
#define C5VRX_CFG_LIVE_INVERT false
#endif

#ifdef CONFIG_C5VRX_WBFM_KERNEL_PHASE8
#define C5VRX_CFG_WBFM_KERNEL C5VRX_WBFM_PHASE8_4TO1
#else
#define C5VRX_CFG_WBFM_KERNEL C5VRX_WBFM_PHASE6_4TO1
#endif

static esp_err_t start_ring_live(c5vrx_rf_dump_mode_t mode,
                                 size_t block_words)
{
    (void)mode;
    (void)block_words;
    /* The MAC dump peripheral temporarily owns the backing HP-SRAM banks.
     * The LP capture kernel is safe because it runs entirely from LP RAM and
     * restores CPU ownership before returning. A FreeRTOS ring worker cannot
     * meet that invariant: real XIAO hardware loses native USB immediately.
     * Keep every caller, including guarded LIVE and BENCH, fail-closed. */
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t stop_live_sources(c5vrx_stream_stats_t *pipeline_stats,
                                   c5vrx_live_ring_stats_t *ring_stats)
{
    const esp_err_t err = c5vrx_live_pipeline_stop();
    /* A worker timeout means ownership is still live and teardown is unsafe.
     * Other errors (notably watchdog restoration) occur after the workers and
     * pipeline buffers are already stopped; preserve that error but still stop
     * PARLIO and the RF source so LIVE STOP remains recoverable. */
    if (err == ESP_ERR_TIMEOUT) return err;
    if (pipeline_stats) c5vrx_live_pipeline_get_stats(pipeline_stats);
    if (ring_stats && s_ring_live)
        c5vrx_live_ring_source_get_stats(&s_ring_source, ring_stats);
    const esp_err_t output_err = c5vrx_cvbs_live_out_stop();
    esp_err_t source_err = ESP_OK;
    if (s_ring_live) {
        source_err = c5vrx_live_ring_source_destroy(&s_ring_source);
        s_ring_live = false;
    } else if (s_finite_live) {
        c5vrx_finite_chain_source_destroy(&s_finite_source);
        s_finite_live = false;
    }
    if (err != ESP_OK) return err;
    if (output_err != ESP_OK) return output_err;
    return source_err;
}

static void print_ring_stats(const c5vrx_live_ring_stats_t *stats)
{
    if (!stats) return;
    printf("C5VRX_LIVE_RING_STATS blocks=%llu words=%llu dropped=%llu missed_words=%llu overruns=%llu discontinuities=%llu stream_epoch=%llu wraps=%llu producer_absolute=%llu consumer_absolute=%llu lag_words=%llu maximum_lag_words=%llu fatal_stops=%llu fatal_reason=%s copy_cycles_total=%llu max_copy_cycles=%u maximum_service_interval_cycles=%u minimum_service_deadline_cycles=%u deadline_headroom_x1000=%u\n",
           (unsigned long long)stats->blocks,
           (unsigned long long)stats->words,
           (unsigned long long)stats->dropped_blocks,
           (unsigned long long)stats->missed_words,
           (unsigned long long)stats->overruns,
           (unsigned long long)stats->discontinuities,
           (unsigned long long)stats->stream_epoch,
           (unsigned long long)stats->wraps_observed,
           (unsigned long long)stats->producer_absolute,
           (unsigned long long)stats->consumer_absolute,
           (unsigned long long)stats->lag_words,
           (unsigned long long)stats->maximum_lag_words,
           (unsigned long long)stats->fatal_stops,
           c5vrx_live_ring_failure_name(stats->fatal_reason),
           (unsigned long long)stats->copy_cycles_total,
           (unsigned)stats->maximum_copy_cycles,
           (unsigned)stats->maximum_service_interval_cycles,
           (unsigned)stats->minimum_service_deadline_cycles,
           (unsigned)stats->deadline_headroom_x1000);
}

#define C5VRX_LIVE_CVBS_SAMPLE_RATE_HZ 20000000u

static const char *cvbs_timing_name(uint32_t line_period_samples)
{
    if (line_period_samples >= 1276u && line_period_samples <= 1284u)
        return "PAL_CANDIDATE";
    if (line_period_samples >= 1267u && line_period_samples <= 1275u)
        return "NTSC_CANDIDATE";
    return "UNKNOWN";
}

static void print_cvbs_lock_status(const c5vrx_usb_preview_stats_t *stats)
{
    const uint32_t line_rate_millihz = stats->line_period_samples ?
        (uint32_t)((uint64_t)C5VRX_LIVE_CVBS_SAMPLE_RATE_HZ * 1000u /
                   stats->line_period_samples) : 0u;
    const bool locked = stats->horizontal_locked && stats->vertical_locked;
    printf("C5VRX_CVBS_LOCK running=%u usb_consumer_active=%u usb_worker_active=%u usb_session_epoch=%u usb_transport_stalls=%u usb_stale_session_drops=%u usb_worker_time_us=%u burst_locked=%u burst_locks=%u h_locked=%u v_locked=%u hsyncs=%u vsyncs=%u frames_completed=%u frames_sent=%u frames_dropped=%u rejected_pulses=%u lock_acquisitions=%u lock_losses=%u line_period_samples=%u line_rate_millihz=%u hsync_width=%u vsync_width=%u threshold=%u timing=%s classification=%s\n",
           c5vrx_usb_preview_running() ? 1u : 0u,
           stats->consumer_active ? 1u : 0u,
           stats->worker_active ? 1u : 0u,
           (unsigned)stats->session_epoch,
           (unsigned)stats->transport_stalls,
           (unsigned)stats->stale_session_drops,
           (unsigned)stats->worker_time_us,
           stats->burst_locked ? 1u : 0u,
           (unsigned)stats->burst_locks,
           stats->horizontal_locked ? 1u : 0u,
           stats->vertical_locked ? 1u : 0u,
           (unsigned)stats->horizontal_syncs, (unsigned)stats->vertical_syncs,
           (unsigned)stats->frames_completed, (unsigned)stats->frames_sent,
           (unsigned)stats->frames_dropped,
           (unsigned)stats->rejected_sync_pulses,
           (unsigned)stats->lock_acquisitions,
           (unsigned)stats->lock_losses, (unsigned)stats->line_period_samples,
           (unsigned)line_rate_millihz, (unsigned)stats->last_hsync_width,
           (unsigned)stats->last_vsync_width, (unsigned)stats->sync_threshold,
           cvbs_timing_name(stats->line_period_samples),
           locked ? "LOCKED" :
               (c5vrx_usb_preview_running() ? "ACQUIRING" : "STOPPED"));
    fflush(stdout);
}

static void run_cvbs_lock_probe(uint32_t duration_ms)
{
    if (!c5vrx_usb_preview_running() || !c5vrx_live_pipeline_running()) {
        printf("C5VRX_CVBS_LOCK_PROBE duration_ms=%u classification=FAILED reason=LIVE_PIPELINE_AND_USB_PREVIEW_REQUIRED code=%d\n",
               (unsigned)duration_ms, (int)ESP_ERR_INVALID_STATE);
        fflush(stdout);
        return;
    }
    c5vrx_usb_preview_stats_t before = {0}, after = {0};
    c5vrx_usb_preview_get_stats(&before);
    const int64_t start_us = esp_timer_get_time();
    uint32_t remaining_ms = duration_ms;
    while (remaining_ms) {
        const uint32_t slice = remaining_ms > 250u ? 250u : remaining_ms;
        c5vrx_usb_preview_keepalive();
        vTaskDelay(pdMS_TO_TICKS(slice));
        remaining_ms -= slice;
    }
    const uint64_t elapsed_us = (uint64_t)(esp_timer_get_time() - start_us);
    c5vrx_usb_preview_get_stats(&after);

    const uint32_t hsyncs = after.horizontal_syncs - before.horizontal_syncs;
    const uint32_t vsyncs = after.vertical_syncs - before.vertical_syncs;
    const uint32_t frames = after.frames_completed - before.frames_completed;
    const uint32_t sent = after.frames_sent - before.frames_sent;
    const uint32_t drops = after.frames_dropped - before.frames_dropped;
    const uint32_t losses = after.lock_losses - before.lock_losses;
    const uint32_t h_rate_millihz = elapsed_us ?
        (uint32_t)((uint64_t)hsyncs * 1000000000ull / elapsed_us) : 0u;
    const uint32_t v_rate_millihz = elapsed_us ?
        (uint32_t)((uint64_t)vsyncs * 1000000000ull / elapsed_us) : 0u;
    const bool timing_plausible = h_rate_millihz >= 15000000u &&
        h_rate_millihz <= 16500000u && v_rate_millihz >= 40000u &&
        v_rate_millihz <= 70000u;
    const bool pass = after.horizontal_locked && after.vertical_locked &&
        frames >= 2u && timing_plausible;
    printf("C5VRX_CVBS_LOCK_PROBE duration_ms=%u elapsed_us=%llu hsyncs=%u vsyncs=%u frames_completed=%u frames_sent=%u frames_dropped=%u lock_losses=%u horizontal_rate_millihz=%u vertical_rate_millihz=%u line_period_samples=%u timing=%s timing_plausible=%u analog_vtx_usable_iq=%u visible_image_machine_proven=0 user_visual_confirmation_required=1 classification=%s code=%d\n",
           (unsigned)duration_ms, (unsigned long long)elapsed_us,
           (unsigned)hsyncs, (unsigned)vsyncs, (unsigned)frames,
           (unsigned)sent, (unsigned)drops, (unsigned)losses,
           (unsigned)h_rate_millihz, (unsigned)v_rate_millihz,
           (unsigned)after.line_period_samples,
           cvbs_timing_name(after.line_period_samples),
           timing_plausible ? 1u : 0u, pass ? 1u : 0u,
           pass ? "MEASURED_CVBS_LOCK" : "NO_STABLE_CVBS_LOCK",
           pass ? (int)ESP_OK : (int)ESP_FAIL);
    fflush(stdout);
}

static bool band_from_char(char c, c5vrx_band_t *out)
{
    if (!out) {
        return false;
    }
    switch ((char)toupper((unsigned char)c)) {
        case 'A': *out = C5VRX_BAND_A; return true;
        case 'B': *out = C5VRX_BAND_B; return true;
        case 'E': *out = C5VRX_BAND_E; return true;
        case 'F': *out = C5VRX_BAND_F; return true;
        case 'R': *out = C5VRX_BAND_R; return true;
        default: return false;
    }
}

static void print_status(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    c5vrx_fpv_channel_t target;
    c5vrx_frequency_plan_t plan;
    c5vrx_wifi5_status_t wifi = {0};

    if (!c5vrx_get_fpv_channel(s_state.band, s_state.channel, &target) ||
        !c5vrx_plan_frequency(target.mhz, &plan)) {
        printf("C5VRX_ERR status-plan\n");
        fflush(stdout);
        return;
    }

    const esp_err_t err = c5vrx_wifi5_get_status(&wifi);
    if (err != ESP_OK) {
        printf("C5VRX_ERR status-wifi code=%d\n", (int)err);
        fflush(stdout);
        return;
    }

    printf("C5VRX_STATUS firmware=%s version=%s idf=%s protocol=8 profile=%s band=%c channel=%u mhz=%u wifi=%u center=%u offset=%d exact=%d inside=%d bw=%u readback=%u direct=%d cvbs=%d av_display=%s phase8_capture=1 heap_internal_free=%u heap_internal_min=%u heap_internal_largest=%u heap_dma_free=%u heap_dma_largest=%u\n",
           app->project_name,
           app->version,
           esp_get_idf_version(),
           CONFIG_C5VRX_BOARD_PROFILE,
           target.letter,
           (unsigned)target.channel,
           (unsigned)target.mhz,
           (unsigned)plan.wifi_channel,
           (unsigned)plan.wifi_center_mhz,
           (int)plan.offset_mhz,
           plan.exact_wifi_center ? 1 : 0,
           plan.inside_c5_rx_window ? 1 : 0,
           s_state.ht40 ? 40u : 20u,
           (unsigned)wifi.active_primary_channel,
           s_state.direct_tune_enabled ? 1 : 0,
           c5vrx_cvbs_test_running() ? 1 : 0,
           c5vrx_cvbs_display_name(c5vrx_cvbs_output_display()),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(
               MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(
               MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    fflush(stdout);
}

static esp_err_t apply_channel(c5vrx_band_t band, uint8_t channel)
{
    c5vrx_fpv_channel_t target;
    c5vrx_frequency_plan_t plan;

    if (!c5vrx_get_fpv_channel(band, channel, &target)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!c5vrx_plan_frequency(target.mhz, &plan)) {
        return ESP_FAIL;
    }
    if (!plan.inside_c5_rx_window) {
        printf("C5VRX_ERR outside-rx-window band=%c channel=%u mhz=%u max=%u\n",
               target.letter, (unsigned)channel, (unsigned)target.mhz,
               (unsigned)C5VRX_C5_RX_MAX_MHZ);
        fflush(stdout);
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = c5vrx_wifi5_start(plan.wifi_channel, s_state.ht40);
    if (err != ESP_OK) {
        printf("C5VRX_ERR tune-wifi code=%d wifi=%u\n", (int)err, (unsigned)plan.wifi_channel);
        fflush(stdout);
        return err;
    }

    if (s_state.direct_tune_enabled && !plan.exact_wifi_center) {
        err = c5vrx_phy_set_frequency_mhz(target.mhz);
        if (err != ESP_OK) {
            printf("C5VRX_WARN direct-tune-failed code=%d requested=%u\n",
                   (int)err, (unsigned)target.mhz);
        }
    }

    s_state.band = band;
    s_state.channel = channel;
    invalidate_rf_capabilities();

    printf("C5VRX_OK set band=%c channel=%u mhz=%u wifi=%u center=%u offset=%d exact=%d bw=%u\n",
           target.letter,
           (unsigned)target.channel,
           (unsigned)target.mhz,
           (unsigned)plan.wifi_channel,
           (unsigned)plan.wifi_center_mhz,
           (int)plan.offset_mhz,
           plan.exact_wifi_center ? 1 : 0,
           s_state.ht40 ? 40u : 20u);
    fflush(stdout);
    return ESP_OK;
}

static void print_help(void)
{
    printf("C5VRX_HELP commands=PING,STATUS,CAPABILITIES,TONE_RESPONSE_PROBE_<0|11>_<signed_offset_hz>_<measured_rate_hz>,APPLY_MEASURED_BANDWIDTH_<occupied_hz>_<factor>_CONFIRMED,APPLY_PHYSICAL_BURST_GATE_CONFIRMED,LIVE_START,LIVE_EXPERIMENTAL_START_<0|11>,LIVE_STOP,SET_<band>_<1-8>,BW_<20|40>,CAPTURE_<256-16384>,CAPTURE_PHASE8_<256-16384>,CHAIN_<2-1024>_<1-16384>,PRODUCER_CADENCE_PROBE_<0|11|ALL>,WRAP_FLAG_PROBE_<0|11>,PHASE_CONTINUITY_PROBE_<0|11>,FINE_TUNE_VERIFY_<center_mhz>_<tone_mhz>_<measured_rate_hz>,PRODUCER_SOAK_<0|11>_<1|10|100|1000|5000|30000_ms>,BENCH_SPARSE_<2|4|8>,BENCH_BITSCRAMBLER,BENCH_PARLIO,BENCH_USB_PREVIEW,BENCH_PIPELINE,BENCH_RING_PIPELINE_<0|11>_<10|100|1000_ms>,USB_PREVIEW_START,USB_PREVIEW_STOP,CVBS_LOCK_STATUS,CVBS_LOCK_PROBE_<1000|5000_ms>,RATE_PROBE_ALL_LEGACY,PHASE_PROBE_<0-7>_LEGACY,DUMP_MODE_PROBE,RF_DEEP_PROBE,RING_PROBE,WBFM_HWTEST,WBFM_CAPTURE_<8-16384_multiple4>,NEARLIVE_START,NEARLIVE_STOP,PIPELINE_STATS,CVBS_TEST,CVBS_STOP\n");
    fflush(stdout);
}

static void print_tuning_snapshot(const char *stage)
{
    c5vrx_fpv_channel_t target = {0};
    c5vrx_wifi5_status_t wifi = {0};
    const bool target_ok = c5vrx_get_fpv_channel(s_state.band, s_state.channel, &target);
    const esp_err_t wifi_err = c5vrx_wifi5_get_status(&wifi);
    printf("C5VRX_TUNING stage=%s requested_mhz=%u public_readback_valid=%u public_channel=%u bandwidth_mhz=%u direct_retune_requested=%u direct_pll_readback_available=0 proxy=PUBLIC_WIFI_CHANNEL\n",
           stage, target_ok ? (unsigned)target.mhz : 0u,
           wifi_err == ESP_OK ? 1u : 0u,
           wifi_err == ESP_OK ? (unsigned)wifi.active_primary_channel : 0u,
           s_state.ht40 ? 40u : 20u,
           s_state.direct_tune_enabled ? 1u : 0u);
    fflush(stdout);
}

static esp_err_t run_deep_probe(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    printf("C5VRX_RF_DEEP_PROBE_BEGIN firmware=%s version=%s idf=%s expected_rftest_sha256=%s producer_enabled=%u warning=NO_INDEFINITE_CAPTURE\n",
           app->project_name, app->version, esp_get_idf_version(),
           C5VRX_RF_DUMP_LIB_SHA256,
           c5vrx_rf_dump_producer_available() ? 1u : 0u);
    print_tuning_snapshot("before");
    esp_err_t result = ESP_OK;
    const c5vrx_rf_dump_mode_t modes[] = {
        C5VRX_RF_DUMP_MODE_ORDINARY_RX,
        C5VRX_RF_DUMP_MODE_11,
    };
    for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
        c5vrx_producer_cadence_t cadence = {0};
        const esp_err_t cadence_err =
            c5vrx_producer_cadence_probe(modes[i], &cadence);
        if (result == ESP_OK && cadence_err != ESP_OK) result = cadence_err;
    }
    print_tuning_snapshot("after_cadence_probe");
    const esp_err_t mode_err = c5vrx_adc_dump_mode_probe();
    printf("C5VRX_RF_DEEP_PROBE_STAGE stage=mode_probe code=%d classification=%s\n",
           (int)mode_err, mode_err == ESP_ERR_NOT_SUPPORTED ?
           "DISABLED_FAIL_CLOSED" : "MEASUREMENT_RECORDED");
    print_tuning_snapshot("after_mode_probe");
    c5vrx_adc_ring_probe_stats_t ring = {0};
    const esp_err_t ring_err = c5vrx_adc_dump_ring_probe(&ring);
    printf("C5VRX_RF_DEEP_PROBE_STAGE stage=ring_probe code=%d pointer_changes=%u content_changes=%u classification=PHYSICAL_INTERPRETATION_REQUIRED\n",
           (int)ring_err, (unsigned)ring.pointer_changes,
           (unsigned)ring.content_changes);
    print_tuning_snapshot("after_ring_probe");
    const esp_err_t finite_err = c5vrx_adc_dump_capture(1024u, false);
    printf("C5VRX_RF_DEEP_PROBE_STAGE stage=finite_iq code=%d classification=IQ10_SANITY_ONLY\n",
           (int)finite_err);
    const esp_err_t wbfm_err = c5vrx_wbfm_hw_probe_dump(1024u);
    printf("C5VRX_RF_DEEP_PROBE_STAGE stage=finite_wbfm code=%d classification=FINITE_CAPTURE_SANITY_ONLY\n",
           (int)wbfm_err);
    print_tuning_snapshot("after_teardown");
    if (result == ESP_OK && ring_err != ESP_OK) result = ring_err;
    if (result == ESP_OK && finite_err != ESP_OK) result = finite_err;
    if (result == ESP_OK && wbfm_err != ESP_OK) result = wbfm_err;
    printf("C5VRX_RF_DEEP_PROBE_DONE code=%d mode_probe_code=%d next=COMPARE_VTX_OFF_AND_A4_5805_LOGS\n",
           (int)result, (int)mode_err);
    fflush(stdout);
    return result;
}

static void handle_line(char *line)
{
    const esp_err_t soak_watchdog_err =
        c5vrx_producer_soak_restore_watchdog();
    if (soak_watchdog_err != ESP_OK) {
        printf("C5VRX_ERR soak-idle-watchdog-restore code=%d\n",
               (int)soak_watchdog_err);
        fflush(stdout);
        return;
    }
    while (*line && isspace((unsigned char)*line)) {
        ++line;
    }
    char *end = line + strlen(line);
    while (end > line && isspace((unsigned char)end[-1])) {
        *--end = '\0';
    }
    if (!*line) {
        return;
    }

    if (strcasecmp(line, "PING") == 0) {
        printf("C5VRX_PONG\n");
        fflush(stdout);
        return;
    }
    if (strcasecmp(line, "HELP") == 0) {
        print_help();
        return;
    }
    if (strcasecmp(line, "STATUS") == 0) {
        print_status();
        return;
    }
    if (c5vrx_live_pipeline_running() &&
        strcasecmp(line, "CAPABILITIES") != 0 &&
        strcasecmp(line, "LIVE STOP") != 0 &&
        strcasecmp(line, "LIVE_STOP") != 0 &&
        strcasecmp(line, "USB PREVIEW START") != 0 &&
        strcasecmp(line, "USB_PREVIEW_START") != 0 &&
        strcasecmp(line, "USB PREVIEW STOP") != 0 &&
        strcasecmp(line, "USB_PREVIEW_STOP") != 0 &&
        strcasecmp(line, "USB PREVIEW KEEPALIVE") != 0 &&
        strcasecmp(line, "USB_PREVIEW_KEEPALIVE") != 0 &&
        strcasecmp(line, "CVBS LOCK STATUS") != 0 &&
        strcasecmp(line, "CVBS_LOCK_STATUS") != 0 &&
        strncasecmp(line, "CVBS LOCK PROBE ", 16) != 0 &&
        strncasecmp(line, "CVBS_LOCK_PROBE_", 16) != 0 &&
        strcasecmp(line, "PIPELINE STATS") != 0 &&
        strcasecmp(line, "PIPELINE_STATS") != 0) {
        printf("C5VRX_ERR receiver-busy owner=LIVE_PIPELINE allowed=STATUS,CAPABILITIES,LIVE_STOP,USB_PREVIEW_START_STOP_KEEPALIVE,PIPELINE_STATS\n");
        fflush(stdout);
        return;
    }
    if (strcasecmp(line, "CAPABILITIES") == 0) {
        const char *reason = NULL;
        const c5vrx_consumer_strategy_t strategy =
            c5vrx_select_consumer(&s_capabilities, &reason);
        printf("C5VRX_CAPABILITIES measured_source_rate=%u source_bandwidth_known=%u phase_continuity_valid=%u continuous_wrap_valid=%u writer_position_valid=%u adjacent_memory_valid=%u pipeline_service_headroom_valid=%u selected_ring_block_words=%u ring_block_bench_seen_mask=0x%02x ring_block_bench_pass_mask=0x%02x hardware_decimation_available=%u sparse_factor_allowed=%u physical_burst_gate_passed=%u bitscrambler_path_available=%u cpu_margin_percent=%d selected=%s fail_reason=%s\n",
               (unsigned)s_capabilities.measured_source_rate,
               s_capabilities.source_bandwidth_known ? 1u : 0u,
               s_capabilities.phase_continuity_valid ? 1u : 0u,
               s_capabilities.continuous_wrap_valid ? 1u : 0u,
               s_capabilities.writer_position_valid ? 1u : 0u,
               s_capabilities.adjacent_memory_valid ? 1u : 0u,
               s_capabilities.pipeline_service_headroom_valid ? 1u : 0u,
               (unsigned)s_selected_ring_block_words,
               s_ring_block_bench_seen, s_ring_block_bench_passed,
               s_capabilities.hardware_decimation_available ? 1u : 0u,
               s_capabilities.sparse_factor_allowed,
               s_physical_burst_gate_passed ? 1u : 0u,
               s_capabilities.bitscrambler_path_available ? 1u : 0u,
               s_capabilities.cpu_margin_percent,
               c5vrx_consumer_strategy_name(strategy), reason ? reason : "NONE");
        fflush(stdout); return;
    }
    if (strcasecmp(line, "LIVE START") == 0 ||
        strcasecmp(line, "LIVE_START") == 0) {
        const char *reason = NULL;
        const c5vrx_consumer_strategy_t strategy =
            c5vrx_select_consumer(&s_capabilities, &reason);
        esp_err_t err = ESP_ERR_NOT_SUPPORTED;
        const char *classification = "FAIL_CLOSED_MEASUREMENT_GATES_REQUIRED";
        if (strategy == C5VRX_CONSUMER_BITSCRAMBLER_RING &&
            s_mode0_soak_passed && s_selected_ring_block_words) {
            c5vrx_wifi5_dedicated_status_t radio = {0};
            err = c5vrx_wifi5_verify_dedicated_receiver(&radio);
            printf("C5VRX_LIVE_RADIO_GUARD band_5g_only=%u power_save_disabled=%u promiscuous_enabled=%u station_disconnected=%u bluetooth_compiled_out=%u ieee802154_compiled_out=%u dedicated=%u code=%d\n",
                   radio.band_5g_only, radio.power_save_disabled,
                   radio.promiscuous_enabled, radio.station_disconnected,
                   radio.bluetooth_compiled_out,
                   radio.ieee802154_compiled_out, radio.dedicated, (int)err);
            if (err == ESP_OK) err = start_ring_live(
                C5VRX_RF_DUMP_MODE_ORDINARY_RX,
                s_selected_ring_block_words);
            classification = err == ESP_OK ? "MEASURED_GATES_PASSED" :
                                              "START_FAILED_FAIL_CLOSED";
            if (!radio.dedicated) reason = "DEDICATED_5G_GUARD_FAILED";
        } else if (strategy != C5VRX_CONSUMER_NONE && !s_mode0_soak_passed) {
            reason = "MODE0_STAGED_SOAK_NOT_PASSED";
        } else if (strategy != C5VRX_CONSUMER_NONE) {
            reason = "SELECTED_STRATEGY_NOT_IMPLEMENTED_FAIL_CLOSED";
        }
        printf("C5VRX_LIVE_START code=%d strategy=%s classification=%s reason=%s\n",
               (int)err,
               c5vrx_consumer_strategy_name(strategy),
               classification, reason ? reason : "NONE");
        fflush(stdout); return;
    }
    if (strcasecmp(line, "APPLY PHYSICAL BURST GATE CONFIRMED") == 0 ||
        strcasecmp(line, "APPLY_PHYSICAL_BURST_GATE_CONFIRMED") == 0) {
        /* This volatile attestation is entered only after the archived scope
         * report passes PAL and NTSC burst amplitude/frequency/phase. A reboot,
         * RF-capability invalidation, or failed kernel self-test revokes it. */
        s_physical_burst_gate_passed = true;
        select_measured_ring_block();
        printf("C5VRX_PHYSICAL_BURST_GATE passed=1 persistence=UNTIL_REBOOT_OR_INVALIDATION classification=OPERATOR_CONFIRMED_ARCHIVED_MEASUREMENT_REQUIRED\n");
        fflush(stdout); return;
    }
    unsigned measured_bandwidth = 0, measured_factor = 0;
    char measured_confirmation[16] = {0};
    if (sscanf(line, "APPLY MEASURED BANDWIDTH %u %u %15s",
               &measured_bandwidth, &measured_factor,
               measured_confirmation) == 3 ||
        sscanf(line, "APPLY_MEASURED_BANDWIDTH_%u_%u_%15s",
               &measured_bandwidth, &measured_factor,
               measured_confirmation) == 3) {
        const bool factor_valid = measured_factor == 1u ||
            measured_factor == 2u || measured_factor == 4u ||
            measured_factor == 8u;
        const bool nyquist_valid = s_capabilities.measured_source_rate &&
            measured_bandwidth > 0u &&
            (uint64_t)measured_bandwidth * measured_factor <=
                s_capabilities.measured_source_rate;
        if (strcasecmp(measured_confirmation, "CONFIRMED") != 0 ||
            !factor_valid || !nyquist_valid) {
            printf("C5VRX_MEASURED_BANDWIDTH_APPLY code=%d occupied_hz=%u factor=%u rate=%u classification=REJECTED reason=CONFIRMATION_FACTOR_OR_NYQUIST_INVALID\n",
                   (int)ESP_ERR_INVALID_ARG, measured_bandwidth,
                   measured_factor,
                   (unsigned)s_capabilities.measured_source_rate);
            fflush(stdout); return;
        }
        s_capabilities.source_bandwidth_known = true;
        s_capabilities.sparse_factor_allowed = (uint8_t)measured_factor;
        printf("C5VRX_MEASURED_BANDWIDTH_APPLY code=0 occupied_hz=%u factor=%u rate=%u classification=MEASURED_ON_HARDWARE evidence_source=HOST_SUPPLIED_RF_SWEEP warning=COMMAND_ASSERTS_RF_SWEEP_CONFIRMED_ANTI_ALIAS_FILTERING\n",
               measured_bandwidth, measured_factor,
               (unsigned)s_capabilities.measured_source_rate);
        fflush(stdout); return;
    }
    unsigned live_mode = 0;
    if (sscanf(line, "LIVE EXPERIMENTAL START %u", &live_mode) == 1 ||
        sscanf(line, "LIVE_EXPERIMENTAL_START_%u", &live_mode) == 1) {
        /* Real ESP32-C5 hardware proved that continuous MAC dump ownership
         * removes normal CPU/FreeRTOS access to the affected HP-SRAM banks.
         * The bounded LP-RAM capture kernel restores ownership before it
         * returns; this ordinary-task ring path cannot do that safely. */
        printf("C5VRX_LIVE_EXPERIMENTAL_START mode=%u block_words=0 code=%d classification=FAIL_CLOSED reason=HP_SRAM_MAC_OWNERSHIP_WEDGES_FREERTOS_USB use=CAPTURE_PHASE8_BOUNDED\n",
               live_mode, (int)ESP_ERR_NOT_SUPPORTED);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "LIVE STOP") == 0 ||
        strcasecmp(line, "LIVE_STOP") == 0) {
        c5vrx_live_ring_stats_t ring = {0};
        const bool was_ring = s_ring_live;
        const esp_err_t err = stop_live_sources(NULL, &ring);
        if (was_ring) print_ring_stats(&ring);
        printf("C5VRX_LIVE_STOP code=%d\n", (int)err);
        fflush(stdout); return;
    }
    int tone_expected_offset = 0;
    unsigned tone_mode = 0, tone_rate = 0;
    if (sscanf(line, "TONE RESPONSE PROBE %u %d %u",
               &tone_mode, &tone_expected_offset, &tone_rate) == 3 ||
        sscanf(line, "TONE_RESPONSE_PROBE_%u_%d_%u",
               &tone_mode, &tone_expected_offset, &tone_rate) == 3) {
        c5vrx_tone_measurement_t measurement = {0};
        const esp_err_t err = c5vrx_producer_tone_response_probe(
            (c5vrx_rf_dump_mode_t)tone_mode, tone_expected_offset,
            tone_rate, &measurement);
        printf("C5VRX_TONE_RESPONSE_DONE mode=%u code=%d\n",
               tone_mode, (int)err);
        fflush(stdout); return;
    }
    unsigned fine_center = 0, fine_tone = 0, fine_rate = 0;
    if (sscanf(line, "FINE TUNE VERIFY %u %u %u",
               &fine_center, &fine_tone, &fine_rate) == 3) {
        if (fine_center < 5000u || fine_center > 6000u ||
            fine_tone < 5000u || fine_tone > 6000u ||
            fine_rate < 1000000u || fine_rate > 320000000u) {
            printf("C5VRX_ERR fine-tune-verify args=center_mhz,tone_mhz,measured_sample_rate_hz\n");
            fflush(stdout); return;
        }
        esp_err_t err = c5vrx_phy_set_frequency_mhz(fine_center);
        c5vrx_tone_measurement_t before = {0}, during = {0}, after = {0};
        if (err == ESP_OK) err = c5vrx_adc_dump_capture(4096u, false);
        if (err == ESP_OK)
            err = c5vrx_iq_tone_measure(4096u, fine_rate, &before);
        if (err == ESP_OK) err = c5vrx_rf_dump_configure(
            C5VRX_ADC_DUMP_MAX_SAMPLES, C5VRX_RF_DUMP_MODE_ORDINARY_RX);
        const bool fine_configured = err == ESP_OK;
        if (fine_configured) err = c5vrx_rf_dump_start();
        const bool fine_started = err == ESP_OK;
        if (fine_started) {
            const int64_t until = esp_timer_get_time() + 500;
            while (esp_timer_get_time() < until) {}
            const esp_err_t stop_err = c5vrx_rf_dump_stop();
            if (stop_err != ESP_OK) err = stop_err;
        } else if (fine_configured) {
            (void)c5vrx_rf_dump_stop();
        }
        if (err == ESP_OK)
            err = c5vrx_iq_tone_measure(4096u, fine_rate, &during);
        if (err == ESP_OK) err = c5vrx_adc_dump_capture(4096u, false);
        if (err == ESP_OK)
            err = c5vrx_iq_tone_measure(4096u, fine_rate, &after);
        printf("C5VRX_FINE_TUNE_VERIFY center_mhz=%u tone_mhz=%u expected_offset_hz=%d measured_sample_rate_hz=%u before_offset_hz=%.3f before_coherence=%.6f during_offset_hz=%.3f during_coherence=%.6f after_offset_hz=%.3f after_coherence=%.6f restore_ok=%u classification=%s code=%d\n",
               fine_center, fine_tone,
               ((int)fine_tone - (int)fine_center) * 1000000, fine_rate,
               before.observed_offset_hz, before.coherence,
               during.observed_offset_hz, during.coherence,
               after.observed_offset_hz, after.coherence,
               c5vrx_rf_dump_last_restore_ok() ? 1u : 0u,
               err == ESP_OK ? "MEASURED_ON_HARDWARE" : "FAILED", (int)err);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "USB PREVIEW START") == 0 ||
        strcasecmp(line, "USB_PREVIEW_START") == 0) {
        const esp_err_t err = c5vrx_usb_preview_start();
        printf("C5VRX_USB_PREVIEW state=START width=%u height=%u encoding=YUV411_PACKED field_native=1 code=%d\n",
               C5VRX_USB_PREVIEW_WIDTH, C5VRX_USB_PREVIEW_HEIGHT, (int)err);
        fflush(stdout);
        return;
    }
    if (strcasecmp(line, "USB PREVIEW STOP") == 0 ||
        strcasecmp(line, "USB_PREVIEW_STOP") == 0) {
        const esp_err_t err = c5vrx_usb_preview_stop();
        printf("C5VRX_USB_PREVIEW state=STOP code=%d\n", (int)err);
        fflush(stdout);
        return;
    }
    if (strcasecmp(line, "USB PREVIEW KEEPALIVE") == 0 ||
        strcasecmp(line, "USB_PREVIEW_KEEPALIVE") == 0) {
        c5vrx_usb_preview_keepalive();
        return;
    }
    if (strcasecmp(line, "CVBS LOCK STATUS") == 0 ||
        strcasecmp(line, "CVBS_LOCK_STATUS") == 0) {
        c5vrx_usb_preview_stats_t stats = {0};
        c5vrx_usb_preview_get_stats(&stats);
        print_cvbs_lock_status(&stats);
        return;
    }
    unsigned cvbs_probe_ms = 0;
    if (sscanf(line, "CVBS LOCK PROBE %u", &cvbs_probe_ms) == 1 ||
        sscanf(line, "CVBS_LOCK_PROBE_%u", &cvbs_probe_ms) == 1) {
        if (cvbs_probe_ms != 1000u && cvbs_probe_ms != 5000u) {
            printf("C5VRX_CVBS_LOCK_PROBE duration_ms=%u classification=REJECTED reason=DURATION_INVALID allowed=1000,5000 code=%d\n",
                   cvbs_probe_ms, (int)ESP_ERR_INVALID_ARG);
            fflush(stdout);
            return;
        }
        run_cvbs_lock_probe(cvbs_probe_ms);
        return;
    }
    unsigned sparse_factor = 0;
    if (sscanf(line, "BENCH SPARSE %u", &sparse_factor) == 1 ||
        sscanf(line, "BENCH_SPARSE_%u", &sparse_factor) == 1) {
        const esp_err_t err = c5vrx_bench_sparse(sparse_factor);
        printf("C5VRX_BENCH_DONE target=SPARSE code=%d\n", (int)err);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "BENCH BITSCRAMBLER") == 0 ||
        strcasecmp(line, "BENCH_BITSCRAMBLER") == 0) {
        uint64_t bytes_per_second = 0;
        const esp_err_t err = c5vrx_bench_bitscrambler(&bytes_per_second);
        printf("C5VRX_BENCH_DONE target=BITSCRAMBLER code=%d\n", (int)err);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "BENCH PARLIO") == 0 ||
        strcasecmp(line, "BENCH_PARLIO") == 0) {
        const esp_err_t err = c5vrx_bench_parlio();
        if (err == ESP_OK) s_parlio_bench_passed = true;
        printf("C5VRX_BENCH_DONE target=PARLIO code=%d\n", (int)err);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "BENCH PARLIO CLOCKS") == 0 ||
        strcasecmp(line, "BENCH_PARLIO_CLOCKS") == 0) {
        static const uint32_t clocks[] = {
            16000000u, 20000000u, 24000000u, 26666667u, 30000000u};
        esp_err_t err = ESP_OK;
        for (unsigned i = 0; i < sizeof(clocks) / sizeof(clocks[0]); ++i) {
            const esp_err_t candidate = c5vrx_bench_parlio_clock(clocks[i]);
            if (err == ESP_OK && candidate != ESP_OK) err = candidate;
        }
        printf("C5VRX_BENCH_DONE target=PARLIO_CLOCK_MATRIX code=%d\n",
               (int)err);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "BENCH USB PREVIEW") == 0 ||
        strcasecmp(line, "BENCH_USB_PREVIEW") == 0) {
        const esp_err_t err = c5vrx_bench_usb_preview();
        printf("C5VRX_BENCH_DONE target=USB_PREVIEW code=%d\n", (int)err);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "BENCH PIPELINE") == 0 ||
        strcasecmp(line, "BENCH_PIPELINE") == 0) {
        uint64_t samples_per_second = 0;
        const esp_err_t err = c5vrx_bench_pipeline(&samples_per_second);
        if (err == ESP_OK && s_capabilities.measured_source_rate) {
            const int64_t margin =
                ((int64_t)samples_per_second * 100 /
                 s_capabilities.measured_source_rate) - 100;
            s_capabilities.cpu_margin_percent =
                margin > 1000 ? 1000 : (margin < -1000 ? -1000 : (int)margin);
            s_synthetic_pipeline_passed = margin >= 20;
            s_capabilities.bitscrambler_path_available = false;
        }
        printf("C5VRX_BENCH_DONE target=PIPELINE code=%d\n", (int)err);
        fflush(stdout); return;
    }
    unsigned ring_bench_mode = 0, ring_bench_ms = 0;
    unsigned ring_bench_words = 4096u;
    int ring_bench_args = sscanf(line, "BENCH RING PIPELINE %u %u %u",
                                 &ring_bench_mode, &ring_bench_ms,
                                 &ring_bench_words);
    if (ring_bench_args < 2)
        ring_bench_args = sscanf(line, "BENCH_RING_PIPELINE_%u_%u_%u",
                                 &ring_bench_mode, &ring_bench_ms,
                                 &ring_bench_words);
    if (ring_bench_args >= 2) {
        const int block_index = ring_block_index(ring_bench_words);
        if ((ring_bench_mode != 0u && ring_bench_mode != 11u) ||
            (ring_bench_ms != 10u && ring_bench_ms != 100u &&
             ring_bench_ms != 1000u) || block_index < 0) {
            printf("C5VRX_BENCH_RING_PIPELINE code=%d classification=REJECTED reason=MODE_DURATION_OR_BLOCK_INVALID allowed_blocks=1024,2048,4096\n",
                   (int)ESP_ERR_INVALID_ARG);
            fflush(stdout); return;
        }
        esp_err_t err = start_ring_live(
            (c5vrx_rf_dump_mode_t)ring_bench_mode, ring_bench_words);
        const bool ring_started = err == ESP_OK;
        c5vrx_cvbs_live_out_stats_t av_before = {0}, av_workload_after = {0},
                                      av_after = {0};
        bool qualification_started = false;
        c5vrx_cvbs_sync_tracker_t acquired_timing = {0};
        const TickType_t acquisition_start = xTaskGetTickCount();
        while (err == ESP_OK) {
            c5vrx_live_pipeline_get_timing(&acquired_timing);
            if (acquired_timing.horizontal_locked &&
                acquired_timing.vertical_locked &&
                acquired_timing.standard != C5VRX_VIDEO_STANDARD_UNKNOWN)
                break;
            if (xTaskGetTickCount() - acquisition_start >=
                pdMS_TO_TICKS(250u)) {
                err = ESP_ERR_TIMEOUT;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1u));
        }
        if (err == ESP_OK) {
            /* Startup continuity filler is legal. Qualification begins only
             * after the canonical authority has acquired H+V+standard. */
            c5vrx_cvbs_live_out_get_stats(&av_before);
            c5vrx_cvbs_live_out_qualification_begin(UINT32_MAX);
            qualification_started = true;
        }
        if (ring_bench_mode == 0u && ring_bench_ms == 1000u)
            s_capabilities.bitscrambler_path_available = false;
        if (err == ESP_OK) vTaskDelay(pdMS_TO_TICKS(ring_bench_ms));
        c5vrx_stream_stats_t pipeline = {0};
        c5vrx_live_ring_stats_t ring = {0};
        if (qualification_started) {
            c5vrx_cvbs_live_out_get_stats(&av_workload_after);
            c5vrx_cvbs_live_out_qualification_end();
        }
        if (ring_started) {
            const esp_err_t stop_err = stop_live_sources(&pipeline, &ring);
            if (err == ESP_OK) err = stop_err;
        }
        c5vrx_cvbs_live_out_get_stats(&av_after);
        const bool av_pass =
            qualification_started &&
            av_before.guardian_running &&
            av_workload_after.mailbox_drops == av_before.mailbox_drops &&
            av_workload_after.qualification_underruns ==
                av_before.qualification_underruns &&
            av_workload_after.phase_mismatch_drops ==
                av_before.phase_mismatch_drops &&
            av_after.guardian_failures == av_before.guardian_failures;
        const bool rate_pass = s_capabilities.measured_source_rate &&
            (uint64_t)pipeline.achieved_input_rate_hz * 100u >=
                (uint64_t)s_capabilities.measured_source_rate * 90u;
        const bool pass = err == ESP_OK && ring.blocks >= 4u &&
            ring.overruns == 0u && ring.fatal_stops == 0u &&
            ring.wraps_observed > 0u &&
            ring.deadline_headroom_x1000 >= 2000u &&
            pipeline.dropped_rf_blocks == 0u &&
            pipeline.output_underruns == 0u && av_pass && rate_pass;
        const uint64_t available_cycles =
            (uint64_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000u * ring_bench_ms;
        const unsigned copy_cpu_percent = available_cycles ?
            (unsigned)(ring.copy_cycles_total * 100u / available_cycles) : 0u;
        const uint64_t copy_bytes_per_second = ring_bench_ms ?
            ring.words * sizeof(uint32_t) * 1000u / ring_bench_ms : 0u;
        const bool copy_shortfall = ring.copy_cycles_total &&
            (copy_cpu_percent >= 25u ||
             ring.fatal_reason == C5VRX_RING_FAILURE_COPY_AMBIGUOUS);
        const char *zero_copy_action = !ring.copy_cycles_total ? "NO_RESULT" :
            (copy_shortfall ? "IMPLEMENT_ZERO_COPY" : "KEEP_IMMUTABLE_COPY");
        if (ring_bench_mode == 0u && ring_bench_ms == 1000u) {
            s_ring_block_bench_seen |= (uint8_t)(1u << block_index);
            if (pass) s_ring_block_bench_passed |=
                (uint8_t)(1u << block_index);
            else s_ring_block_bench_passed &=
                (uint8_t)~(1u << block_index);
            select_measured_ring_block();
        }
        print_ring_stats(&ring);
        printf("C5VRX_BENCH_RING_PIPELINE mode=%u duration_ms=%u block_words=%u blocks=%llu input_rate=%u measured_source_rate=%u rate_pass=%u copy_bytes_per_second=%llu copy_cycles_total=%llu copy_cpu_percent=%u zero_copy_action=%s dropped=%llu output_underruns=%llu av_mailbox_drops=%llu av_qualification_underruns=%llu av_phase_mismatch_drops=%llu av_guardian_failures=%llu av_pass=%u deadline_headroom_x1000=%u two_x_deadline_headroom_pass=%u synthetic_margin_pass=%u matrix_seen_mask=0x%02x matrix_pass_mask=0x%02x selected_block_words=%u classification=%s code=%d\n",
               ring_bench_mode, ring_bench_ms, ring_bench_words,
               (unsigned long long)pipeline.blocks_processed,
               (unsigned)pipeline.achieved_input_rate_hz,
               (unsigned)s_capabilities.measured_source_rate,
               rate_pass ? 1u : 0u,
               (unsigned long long)copy_bytes_per_second,
               (unsigned long long)ring.copy_cycles_total,
               copy_cpu_percent, zero_copy_action,
               (unsigned long long)pipeline.dropped_rf_blocks,
               (unsigned long long)pipeline.output_underruns,
               (unsigned long long)(av_workload_after.mailbox_drops - av_before.mailbox_drops),
               (unsigned long long)(av_workload_after.qualification_underruns - av_before.qualification_underruns),
               (unsigned long long)(av_workload_after.phase_mismatch_drops - av_before.phase_mismatch_drops),
               (unsigned long long)(av_after.guardian_failures - av_before.guardian_failures),
               av_pass ? 1u : 0u,
               (unsigned)ring.deadline_headroom_x1000,
               ring.deadline_headroom_x1000 >= 2000u ? 1u : 0u,
               s_synthetic_pipeline_passed ? 1u : 0u,
               s_ring_block_bench_seen, s_ring_block_bench_passed,
               (unsigned)s_selected_ring_block_words,
               pass ? "MEASURED_ON_HARDWARE" : "FAILED", (int)err);
        fflush(stdout); return;
    }
    if (c5vrx_live_pipeline_running() &&
        (strncasecmp(line, "PRODUCER ", 9) == 0 ||
         strncasecmp(line, "PRODUCER_", 9) == 0 ||
         strncasecmp(line, "WRAP FLAG ", 10) == 0 ||
         strncasecmp(line, "WRAP_FLAG_", 10) == 0 ||
         strncasecmp(line, "PHASE CONTINUITY ", 17) == 0 ||
         strncasecmp(line, "PHASE_CONTINUITY_", 17) == 0)) {
        printf("C5VRX_ERR rf-engine-busy owner=LIVE_PIPELINE\n");
        fflush(stdout);
        return;
    }

    unsigned producer_mode = 0;
    char producer_arg[16] = {0};
    if (sscanf(line, "PRODUCER CADENCE PROBE %15s", producer_arg) == 1 ||
        sscanf(line, "PRODUCER_CADENCE_PROBE_%15s", producer_arg) == 1) {
        const bool all = strcasecmp(producer_arg, "ALL") == 0;
        if (!all && sscanf(producer_arg, "%u", &producer_mode) != 1) {
            printf("C5VRX_ERR invalid-producer-mode allowed=0,11,ALL mode12=VENDOR_BLE_ONLY\n");
            fflush(stdout);
            return;
        }
        const c5vrx_rf_dump_mode_t modes[] = {
            C5VRX_RF_DUMP_MODE_ORDINARY_RX,
            C5VRX_RF_DUMP_MODE_11,
        };
        esp_err_t final = ESP_OK;
        for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
            if (!all && producer_mode != (unsigned)modes[i]) continue;
            c5vrx_producer_cadence_t cadence = {0};
            const esp_err_t err =
                c5vrx_producer_cadence_probe(modes[i], &cadence);
            if (err == ESP_OK && modes[i] == C5VRX_RF_DUMP_MODE_ORDINARY_RX &&
                cadence.ambiguous_intervals == 0u &&
                cadence.complex_samples_per_sec > 0u) {
                if (s_capabilities.measured_source_rate !=
                    (uint32_t)cadence.complex_samples_per_sec) {
                    invalidate_rf_capabilities();
                }
                s_capabilities.measured_source_rate =
                    (uint32_t)cadence.complex_samples_per_sec;
            }
            if (final == ESP_OK && err != ESP_OK) final = err;
        }
        if (!all && producer_mode != 0u && producer_mode != 11u)
            final = ESP_ERR_INVALID_ARG;
        printf("C5VRX_PRODUCER_CADENCE_DONE requested=%s code=%d\n",
               producer_arg, (int)final);
        fflush(stdout);
        return;
    }

    if (sscanf(line, "WRAP FLAG PROBE %u", &producer_mode) == 1 ||
        sscanf(line, "WRAP_FLAG_PROBE_%u", &producer_mode) == 1) {
        const esp_err_t err = c5vrx_producer_wrap_flag_probe(
            (c5vrx_rf_dump_mode_t)producer_mode);
        printf("C5VRX_WRAP_FLAG_PROBE_DONE mode=%u code=%d\n",
               producer_mode, (int)err);
        fflush(stdout);
        return;
    }

    if (sscanf(line, "PHASE CONTINUITY PROBE %u", &producer_mode) == 1 ||
        sscanf(line, "PHASE_CONTINUITY_PROBE_%u", &producer_mode) == 1) {
        c5vrx_phase_continuity_t phase = {0};
        const esp_err_t err = c5vrx_producer_phase_continuity_probe(
            (c5vrx_rf_dump_mode_t)producer_mode, &phase);
        if (err == ESP_OK && producer_mode == 0u && phase.boundary_continuous)
            s_capabilities.phase_continuity_valid = true;
        printf("C5VRX_PHASE_CONTINUITY_DONE mode=%u code=%d\n",
               producer_mode, (int)err);
        fflush(stdout);
        return;
    }

    unsigned soak_ms = 0;
    if (sscanf(line, "PRODUCER SOAK %u %u", &producer_mode, &soak_ms) == 2 ||
        sscanf(line, "PRODUCER_SOAK_%u_%u", &producer_mode, &soak_ms) == 2) {
        if (producer_mode == 0u && soak_ms == 30000u) {
            s_mode0_soak_passed = false;
            s_capabilities.continuous_wrap_valid = false;
            s_capabilities.writer_position_valid = false;
            s_capabilities.adjacent_memory_valid = false;
            s_capabilities.pipeline_service_headroom_valid = false;
            s_capabilities.bitscrambler_path_available = false;
            s_ring_block_bench_seen = 0u;
            s_ring_block_bench_passed = 0u;
            s_selected_ring_block_words = 0u;
        }
        c5vrx_producer_soak_result_t soak = {0};
        const esp_err_t err = c5vrx_producer_soak(
            (c5vrx_rf_dump_mode_t)producer_mode, soak_ms, &soak);
        if (err == ESP_OK && producer_mode == 0u && soak_ms == 30000u) {
            s_mode0_soak_passed = true;
            s_capabilities.continuous_wrap_valid = soak.exact_wraps > 0u &&
                soak.producer_stop_events == 0u;
            s_capabilities.writer_position_valid =
                soak.ambiguous_intervals == 0u &&
                soak.pointer_out_of_range == 0u;
            s_capabilities.adjacent_memory_valid =
                soak.adjacent_canary_failures == 0u;
        }
        printf("C5VRX_PRODUCER_SOAK_DONE mode=%u requested_ms=%u continuous_wrap_valid=%u writer_position_valid=%u adjacent_memory_valid=%u exact_wraps=%llu ambiguous_intervals=%llu adjacent_canary_failures=%u code=%d\n",
               producer_mode, soak_ms,
               s_capabilities.continuous_wrap_valid ? 1u : 0u,
               s_capabilities.writer_position_valid ? 1u : 0u,
               s_capabilities.adjacent_memory_valid ? 1u : 0u,
               (unsigned long long)soak.exact_wraps,
               (unsigned long long)soak.ambiguous_intervals,
               (unsigned)soak.adjacent_canary_failures, (int)err);
        fflush(stdout);
        return;
    }
    if (strcasecmp(line, "RATE PROBE ALL") == 0 ||
        strcasecmp(line, "RATE_PROBE_ALL") == 0) {
        printf("C5VRX_RATE_PROBE_BEGIN fields=8 method=UNMODIFIED_VENDOR_ARMS no_forced_register_values=1\n");
        fflush(stdout);
        const esp_err_t err = c5vrx_adc_rate_probe_all();
        printf("C5VRX_RATE_PROBE_DONE code=%d\n", (int)err);
        fflush(stdout);
        return;
    }
    unsigned phase_field = 0;
    if (sscanf(line, "PHASE PROBE %u", &phase_field) == 1 ||
        sscanf(line, "PHASE_PROBE_%u", &phase_field) == 1) {
        const esp_err_t err = c5vrx_adc_phase_probe(phase_field);
        printf("C5VRX_PHASE_PROBE_DONE field=%u code=%d\n", phase_field, (int)err);
        fflush(stdout);
        return;
    }
    if (strcasecmp(line, "DUMP MODE PROBE") == 0 ||
        strcasecmp(line, "DUMP_MODE_PROBE") == 0) {
        const esp_err_t err = c5vrx_adc_dump_mode_probe();
        printf("C5VRX_DUMP_MODE_PROBE_DONE code=%d classification=%s\n",
               (int)err, err == ESP_ERR_NOT_SUPPORTED ?
               "DISABLED_FAIL_CLOSED" : "PHYSICAL_INTERPRETATION_REQUIRED");
        fflush(stdout);
        return;
    }
    if (strcasecmp(line, "RF DEEP PROBE") == 0 ||
        strcasecmp(line, "RF_DEEP_PROBE") == 0) {
        (void)run_deep_probe();
        return;
    }
    if (strcasecmp(line, "PIPELINE STATS") == 0 ||
        strcasecmp(line, "PIPELINE_STATS") == 0) {
        c5vrx_live_pipeline_log_stats();
        return;
    }
    if (strcasecmp(line, "NEARLIVE START") == 0 ||
        strcasecmp(line, "NEARLIVE_START") == 0) {
        if (c5vrx_live_pipeline_running() || c5vrx_cvbs_test_running()) {
            printf("C5VRX_ERR nearlive-already-running\n"); fflush(stdout); return;
        }
        esp_err_t err = c5vrx_usb_preview_prepare();
        if (err == ESP_OK)
            err = c5vrx_finite_chain_source_create(&s_finite_source, 16384u);
        const c5vrx_live_pipeline_config_t config = {
            .source = &s_finite_source, .sink = live_output_with_preview,
            .maximum_input_words = 16384u,
            .wbfm_kernel = C5VRX_CFG_WBFM_KERNEL,
            .conditioner = {
                .bias_q8 = CONFIG_C5VRX_LIVE_CVBS_BIAS_Q8,
                .gain_q8 = CONFIG_C5VRX_LIVE_CVBS_GAIN_Q8,
                .invert = C5VRX_CFG_LIVE_INVERT,
                .sync_code = 0, .blank_code = 19, .black_code = 20,
                .white_code = 63,
                .clamp_min = CONFIG_C5VRX_LIVE_CVBS_CLAMP_MIN,
                .clamp_max = CONFIG_C5VRX_LIVE_CVBS_CLAMP_MAX,
                .filter_shift = CONFIG_C5VRX_LIVE_CVBS_FILTER_SHIFT,
            },
        };
        if (err == ESP_OK) err = c5vrx_live_pipeline_start(&config);
        s_finite_live = err == ESP_OK;
        if (err != ESP_OK) {
            (void)c5vrx_cvbs_live_out_stop();
            c5vrx_finite_chain_source_destroy(&s_finite_source);
            s_finite_live = false;
        }
        printf("C5VRX_NEARLIVE_START code=%d mode=FINITE_CHAINED_NOT_CONTINUOUS\n", (int)err);
        fflush(stdout); return;
    }
    if (strcasecmp(line, "NEARLIVE STOP") == 0 ||
        strcasecmp(line, "NEARLIVE_STOP") == 0) {
        esp_err_t err = c5vrx_live_pipeline_stop();
        if (err == ESP_OK) {
            (void)c5vrx_cvbs_live_out_stop();
            c5vrx_finite_chain_source_destroy(&s_finite_source);
            s_finite_live = false;
        }
        printf("C5VRX_NEARLIVE_STOP code=%d\n", (int)err); fflush(stdout); return;
    }

    if (strcasecmp(line, "WBFM HWTEST") == 0 ||
        strcasecmp(line, "WBFM_HWTEST") == 0) {
        printf("C5VRX_WBFM_HWTEST_BEGIN\n");
        fflush(stdout);
        const esp_err_t err =
            c5vrx_wbfm_hw_self_test_kernel(C5VRX_CFG_WBFM_KERNEL);
        s_wbfm_self_test_passed = err == ESP_OK;
        if (err != ESP_OK) s_physical_burst_gate_passed = false;
        select_measured_ring_block();
        printf("C5VRX_WBFM_HWTEST_DONE kernel=phase%u code=%d\n",
               C5VRX_CFG_WBFM_KERNEL == C5VRX_WBFM_PHASE8_4TO1 ? 8u : 6u,
               (int)err);
        fflush(stdout);
        return;
    }

    unsigned wbfm_samples = 0;
    if (sscanf(line, "WBFM CAPTURE %u", &wbfm_samples) == 1) {
        if (wbfm_samples < 8 ||
            wbfm_samples > C5VRX_ADC_DUMP_MAX_SAMPLES ||
            (wbfm_samples & 3u) != 0u) {
            printf("C5VRX_ERR invalid-wbfm-capture range=8-%u multiple=4\n",
                   (unsigned)C5VRX_ADC_DUMP_MAX_SAMPLES);
            fflush(stdout);
            return;
        }
        printf("C5VRX_WBFM_CAPTURE_BEGIN iq_samples=%u\n", wbfm_samples);
        fflush(stdout);
        const esp_err_t err = c5vrx_wbfm_hw_probe_dump(wbfm_samples);
        printf("C5VRX_WBFM_CAPTURE_DONE code=%d\n", (int)err);
        fflush(stdout);
        return;
    }

    if (strcasecmp(line, "CVBS TEST") == 0 || strcasecmp(line, "CVBS_TEST") == 0) {
        const esp_err_t err = c5vrx_cvbs_test_start();
        if (err == ESP_OK) {
            printf("C5VRX_OK cvbs-test=1 av_always_on=1 display=TEST standard=PAL625 fields_hz=50 frames_hz=25 sample_rate=20000000\n");
        } else {
            printf("C5VRX_ERR cvbs-test code=%d\n", (int)err);
        }
        fflush(stdout);
        return;
    }

    if (strcasecmp(line, "CVBS STOP") == 0 || strcasecmp(line, "CVBS_STOP") == 0) {
        const esp_err_t err = c5vrx_cvbs_test_stop();
        if (err == ESP_OK) {
            printf("C5VRX_OK cvbs-test=0 av_always_on=1 display=LOGO\n");
        } else {
            printf("C5VRX_ERR cvbs-stop code=%d\n", (int)err);
        }
        fflush(stdout);
        return;
    }

    char band_char = 0;
    unsigned channel = 0;
    if (sscanf(line, "SET %c %u", &band_char, &channel) == 2) {
        c5vrx_band_t band;
        if (!band_from_char(band_char, &band) || channel < 1 || channel > 8) {
            printf("C5VRX_ERR invalid-channel\n");
            fflush(stdout);
            return;
        }
        (void)apply_channel(band, (uint8_t)channel);
        return;
    }

    unsigned bw = 0;
    if (sscanf(line, "BW %u", &bw) == 1) {
        if (bw != 20 && bw != 40) {
            printf("C5VRX_ERR invalid-bw allowed=20,40\n");
            fflush(stdout);
            return;
        }
        s_state.ht40 = (bw == 40);
        const esp_err_t err = apply_channel(s_state.band, s_state.channel);
        if (err == ESP_OK) {
            printf("C5VRX_OK bw=%u\n", bw);
            fflush(stdout);
        }
        return;
    }

    unsigned chain_blocks = 0;
    unsigned chain_samples = 0;
    if (sscanf(line, "CHAIN %u %u", &chain_blocks, &chain_samples) == 2) {
        if (chain_blocks < 2 || chain_blocks > 1024 ||
            chain_samples < 1 || chain_samples > C5VRX_ADC_DUMP_MAX_SAMPLES) {
            printf("C5VRX_ERR invalid-chain blocks=2-1024 samples=1-%u\n",
                   (unsigned)C5VRX_ADC_DUMP_MAX_SAMPLES);
            fflush(stdout);
            return;
        }
        c5vrx_adc_chain_stats_t stats = {0};
        printf("C5VRX_CHAIN_BEGIN blocks=%u samples=%u\n", chain_blocks, chain_samples);
        fflush(stdout);
        const esp_err_t err = c5vrx_adc_dump_capture_chained(
            chain_blocks,
            chain_samples,
            &stats);
        printf("C5VRX_CHAIN_DONE code=%d blocks=%u total=%llu repeated_hashes=%u boundary_jump_power=%llu\n",
               (int)err,
               (unsigned)stats.blocks_completed,
               (unsigned long long)stats.total_samples,
               (unsigned)stats.repeated_block_hashes,
               (unsigned long long)stats.boundary_jump_power_sum);
        fflush(stdout);
        return;
    }

    if (strcasecmp(line, "RING PROBE") == 0 ||
        strcasecmp(line, "RING_PROBE") == 0) {
        c5vrx_adc_ring_probe_stats_t stats = {0};
        printf("C5VRX_RING_PROBE_BEGIN mode=SINGLE_ARM_PRETRIGGER_NOT_CONTINUOUS\n");
        fflush(stdout);
        const esp_err_t err = c5vrx_adc_dump_ring_probe(&stats);
        printf("C5VRX_RING_PROBE_DONE code=%d active_us=%llu observations=%u pointer_changes=%u pointer_min=%u pointer_max=%u content_changes=%u done=%u vendor_timeout=%u status=%08x classification=%s\n",
               (int)err, (unsigned long long)stats.active_time_us,
               (unsigned)stats.observations, (unsigned)stats.pointer_changes,
               (unsigned)stats.minimum_pointer, (unsigned)stats.maximum_pointer,
               (unsigned)stats.content_changes, stats.completion_bit_seen ? 1u : 0u,
               stats.reached_vendor_timeout ? 1u : 0u,
               (unsigned)stats.final_status,
               err == ESP_OK && stats.reached_vendor_timeout &&
                       stats.pointer_changes > 2u
                   ? "PRETRIGGER_RING_CANDIDATE_PHYSICAL_VALIDATION_REQUIRED"
                   : "NO_CONTINUOUS_EVIDENCE");
        fflush(stdout);
        return;
    }

    unsigned samples = 0;
    if (sscanf(line, "CAPTURE PHASE8 %u", &samples) == 1 ||
        sscanf(line, "CAPTURE_PHASE8_%u", &samples) == 1) {
        if (samples < 256 || samples > C5VRX_ADC_DUMP_MAX_SAMPLES) {
            printf("C5VRX_ERR invalid-phase8-capture range=256-%u\n",
                   (unsigned)C5VRX_ADC_DUMP_MAX_SAMPLES);
            fflush(stdout);
            return;
        }
        printf("C5VRX_PHASE8_CAPTURE_BEGIN samples=%u\n", samples);
        fflush(stdout);
        const esp_err_t err = c5vrx_adc_dump_capture_phase8(samples);
        if (err == ESP_OK) {
            /* A varying RF block exists but bounded captures cannot establish
             * gapless H/V lock. Snow is the honest unlocked state and remains
             * selected across later sample gaps instead of flashing LOGO. */
            (void)c5vrx_cvbs_output_set_display(C5VRX_CVBS_DISPLAY_SNOW);
        }
        printf("C5VRX_PHASE8_CAPTURE_DONE code=%d\n", (int)err);
        fflush(stdout);
        return;
    }
    if (sscanf(line, "CAPTURE %u", &samples) == 1) {
        if (samples < 256 || samples > C5VRX_ADC_DUMP_MAX_SAMPLES) {
            printf("C5VRX_ERR invalid-capture range=256-%u\n",
                   (unsigned)C5VRX_ADC_DUMP_MAX_SAMPLES);
            fflush(stdout);
            return;
        }
        printf("C5VRX_CAPTURE_BEGIN samples=%u\n", samples);
        fflush(stdout);
        const esp_err_t err = c5vrx_adc_dump_capture(samples, true);
        if (err == ESP_OK)
            (void)c5vrx_cvbs_output_set_display(C5VRX_CVBS_DISPLAY_SNOW);
        printf("C5VRX_CAPTURE_DONE code=%d\n", (int)err);
        fflush(stdout);
        return;
    }

    printf("C5VRX_ERR unknown-command\n");
    fflush(stdout);
}

static void console_task(void *arg)
{
    (void)arg;
    char line[128] = {0};
    size_t used = 0;
    uint8_t input[64];

    printf("C5VRX_READY protocol=8 usb_preview_binary=1 phase8_capture=1 lab_session_artifacts=1\n");
    print_help();

    for (;;) {
        const int count = usb_serial_jtag_read_bytes(
            input, sizeof(input), pdMS_TO_TICKS(20));
        if (count < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        for (int i = 0; i < count; ++i) {
            const uint8_t byte = input[i];
            if (byte == '\r' || byte == '\n') {
                if (used) {
                    line[used] = '\0';
                    handle_line(line);
                    used = 0;
                }
                continue;
            }
            if (byte < 0x20u || byte > 0x7eu) continue;
            if (used + 1u < sizeof(line)) {
                line[used++] = (char)byte;
            } else {
                used = 0;
                printf("C5VRX_ERR command-too-long max=%u\n",
                       (unsigned)(sizeof(line) - 1u));
            }
        }
    }
}

esp_err_t c5vrx_control_start(c5vrx_band_t band,
                              uint8_t channel,
                              bool ht40,
                              bool direct_tune_enabled)
{
    if (s_state.started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (band >= C5VRX_BAND_COUNT || channel < 1 || channel > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state = (c5vrx_control_state_t) {
        .band = band,
        .channel = channel,
        .ht40 = ht40,
        .direct_tune_enabled = direct_tune_enabled,
        .started = true,
    };

    /* The finite RF dump calls vendor code and logging from this task.  Keep
     * explicit headroom so a diagnostic burst cannot reach the stack guard. */
    /* Normally blocked in the USB driver. When a command or a timed benchmark
     * completes it must preempt ring workers so LIVE remains controllable on
     * the single-core C5 without forcing millisecond sleeps into writer
     * observation. */
    if (xTaskCreate(console_task, "c5vrx_usbctl", 6144, NULL, 20, NULL) != pdPASS) {
        s_state.started = false;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
