/* Bounded, RF-off hardware validation. Never part of live sample pacing. */
#include "sdkconfig.h"
#if CONFIG_C5VRX2_MODE_LINEAR80_ORACLE
#include <stdlib.h>
#include <string.h>
#include "driver/bitscrambler_loopback.h"
#include "driver/parlio_bitscrambler.h"
#include "driver/parlio_tx.h"
#include "driver/parlio_rx.h"
#include "parlio_priv.h" /* IDF 6.0.1: decorator-owned BS teardown */
#include "esp_private/gdma.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "soc/soc_caps.h"
#include "soc/parl_io_struct.h"
#include "hal/parlio_ll.h"
#include "calibration.h"
#include "wbfm_q4.h"
#include "startup_trace.h"

#define INPUT_BYTES 16384u
#define OUTPUT_BYTES (INPUT_BYTES * 2u)
#define OUTPUT_CAPACITY (OUTPUT_BYTES + 64u)

static uint32_t fnv(const void *data, size_t n)
{
    const uint8_t *p=data;
    uint32_t h=2166136261u;
    while (n--) h=(h ^ *p++)*16777619u;
    return h;
}

static esp_err_t timed_tx_mode(uint8_t *raw, uint32_t rate, uint32_t *rows, const void *program, bool counted_eof)
{
    parlio_tx_unit_handle_t tx=NULL;
    bool decorated=false, enabled=false;
    const parlio_tx_unit_config_t cfg={
        .clk_src=PARLIO_CLK_SRC_DEFAULT, .clk_in_gpio_num=-1,
        .output_clk_freq_hz=rate, .data_width=8,
        .data_gpio_nums={23,24,11,12,8,9,-1,-1},
        .clk_out_gpio_num=-1, .valid_gpio_num=-1,
        .trans_queue_depth=1, .max_transfer_size=INPUT_BYTES,
        .dma_burst_size=32, .shift_edge=PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order=PARLIO_BIT_PACK_ORDER_LSB,
    };
    esp_err_t err=parlio_new_tx_unit(&cfg,&tx);
    if (err!=ESP_OK) return err;
    err=parlio_tx_unit_decorate_bitscrambler(tx);
    if (err!=ESP_OK) goto cleanup;
    decorated=true;
    err=parlio_tx_unit_enable(tx);
    if (err!=ESP_OK) goto cleanup;
    enabled=true;
    /* Keep FIFO-empty evidence sticky: stock ISR logs and clears it. */
    parlio_ll_enable_interrupt(&PARL_IO,PARLIO_LL_EVENT_TX_FIFO_EMPTY,false);
    const parlio_transmit_config_t tr={
        .idle_value=20, .bitscrambler_program=program,
    };
    for (unsigned run=0;run<4;++run) {
        uint32_t bytes=(run&1)?INPUT_BYTES:INPUT_BYTES/4;
        uint32_t *r=rows+run*5;
        r[0]=rate; r[1]=bytes;
        PARL_IO.int_clr.tx_fifo_rempty_int_clr=1;
        int64_t start=esp_timer_get_time();
        err=parlio_tx_unit_transmit(tx,raw,bytes*8u,&tr);
        if (err==ESP_OK) {
            /* Diagnostic only: IDF selects DMA EOF when starting TX. Test
             * explicit expanded-output length instead. No DMA input-length
             * inflation, no fabricated completion callback. Set length first.
             * The regular gate above/below retains the stock driver path. */
            if (counted_eof) {
                parlio_ll_tx_set_trans_bit_len(&PARL_IO,bytes*16u);
                parlio_ll_tx_set_eof_condition(&PARL_IO,PARLIO_LL_TX_EOF_COND_DATA_LEN);
            }
            /* Observe the sticky FIFO flag during, not after, the stream.
             * No USB/logging or buffer inspection during transmission. */
            esp_rom_delay_us((uint64_t)bytes*1000000u/rate);
            r[3]=PARL_IO.int_raw.val;
            err=parlio_tx_unit_wait_all_done(tx,1000);
        }
        r[2]=(uint32_t)(esp_timer_get_time()-start); r[4]=err;
        if (err!=ESP_OK) break;
    }
cleanup:
    if (enabled) (void)parlio_tx_unit_disable(tx);
    if (decorated) {
        /* Stock unit disable does not disable BS on interrupted/loop TX;
         * stock EOF ISR does. Freeing a still-enabled BS leaks routing. */
        (void)bitscrambler_disable(tx->bs_handle);
        (void)parlio_tx_unit_undecorate_bitscrambler(tx);
    }
    (void)parlio_del_tx_unit(tx);
    return err;
}

static esp_err_t timed_tx(uint8_t *raw, uint32_t rate, uint32_t *rows, const void *program)
{
    return timed_tx_mode(raw,rate,rows,program,false);
}

/* RF-off repeating deterministic input, persistent BS state across DMA wraps.
 * Read the six existing DAC pads at requested RX40; TX80 is undersampled.
 * No external clock pins, resistor changes, or CPU sample processing. */
static void pad_capture(uint8_t *raw, uint8_t *capture, unsigned trial)
{
    unsigned fast=trial&1u, mode=trial==6?1:trial/2u;
    c5vrx2_trace_stage_detail(0x870+trial,ESP_OK,mode,fast,0);
    parlio_tx_unit_handle_t tx=NULL;
    parlio_rx_unit_handle_t rx=NULL;
    parlio_rx_delimiter_handle_t delimiter=NULL;
    bool decorated=false, te=false, re=false;
    uint32_t h[16]={0x5044384c,1,64,4096,trial==6?48000000:fast?60000000:40000000,40000000};
    h[14]=mode; /* 0 linear80, 1 direct bytes, 2 existing Phase5 */
    h[6]=h[7]=h[8]=UINT32_MAX;
    memset(capture,0xa5,4096);
    const parlio_tx_unit_config_t tc={
        .clk_src=PARLIO_CLK_SRC_DEFAULT,.clk_in_gpio_num=-1,
        .output_clk_freq_hz=h[4],.data_width=8,
        .data_gpio_nums={23,24,11,12,8,9,-1,-1},
        .clk_out_gpio_num=-1,.valid_gpio_num=-1,
        .trans_queue_depth=1,.max_transfer_size=INPUT_BYTES,
        .dma_burst_size=32,.shift_edge=PARLIO_SHIFT_EDGE_NEG,
        .bit_pack_order=PARLIO_BIT_PACK_ORDER_LSB,
    };
    esp_err_t err=parlio_new_tx_unit(&tc,&tx);
    if (err!=ESP_OK) goto cleanup;
    /* Record the driver's selected divider result, not just the request.
     * Restore stock DMA32: tested internal DMA64 did not resolve the fault. */
    h[4]=tx->out_clk_freq_hz;
    if (mode!=1) {
        err=parlio_tx_unit_decorate_bitscrambler(tx);
        if (err!=ESP_OK) goto cleanup;
        decorated=true;
    }
    const parlio_rx_unit_config_t rc={
        .trans_queue_depth=1,.max_recv_size=4096,.dma_burst_size=32,
        .data_width=8,.clk_src=PARLIO_CLK_SRC_DEFAULT,.exp_clk_freq_hz=40000000,
        .clk_in_gpio_num=-1,.clk_out_gpio_num=-1,.valid_gpio_num=-1,
        .data_gpio_nums={23,24,11,12,8,9,-1,-1},.flags.free_clk=true,
    };
    err=parlio_new_rx_unit(&rc,&rx);
    if (err!=ESP_OK) goto cleanup;
    const parlio_rx_soft_delimiter_config_t dc={
        .sample_edge=PARLIO_SAMPLE_EDGE_POS,.bit_pack_order=PARLIO_BIT_PACK_ORDER_LSB,
        .eof_data_len=4096,
    };
    err=parlio_new_rx_soft_delimiter(&dc,&delimiter);
    if (err!=ESP_OK) goto cleanup;
    err=parlio_rx_unit_enable(rx,true);
    if (err!=ESP_OK) goto cleanup;
    re=true;
    err=parlio_tx_unit_enable(tx);
    if (err!=ESP_OK) goto cleanup;
    te=true;
    parlio_ll_enable_interrupt(&PARL_IO,PARLIO_LL_EVENT_TX_FIFO_EMPTY,false);
    const parlio_receive_config_t receive={.delimiter=delimiter};
    err=parlio_rx_unit_receive(rx,capture,4096,&receive);
    if (err!=ESP_OK) goto cleanup;
    const parlio_transmit_config_t transmit={.idle_value=20,
        .bitscrambler_program=mode==1?NULL:mode==2?c5vrx2_wbfm_q4_phase5_program():c5vrx2_wbfm_linear80_program(),
        .flags.loop_transmission=true};
    c5vrx2_trace_stage_detail(0x880+trial,ESP_OK,mode,fast,0);
    err=parlio_tx_unit_transmit(tx,raw,INPUT_BYTES*8,&transmit);
    h[6]=err;
    if (err!=ESP_OK) goto cleanup;
    esp_rom_delay_us(3000);
    h[9]=PARL_IO.int_raw.val;
    int64_t start=esp_timer_get_time();
    err=parlio_rx_soft_delimiter_start_stop(rx,delimiter,true);
    if (err==ESP_OK) err=parlio_rx_unit_wait_all_done(rx,1000);
    h[7]=err;h[10]=esp_timer_get_time()-start;h[11]=PARL_IO.int_raw.val;
    h[15]=PARL_IO.tx_st0.val;
cleanup:
    if (re) {
        (void)parlio_rx_soft_delimiter_start_stop(rx,delimiter,false);
        (void)parlio_rx_unit_disable(rx);
    }
    if (te) (void)parlio_tx_unit_disable(tx);
    if (delimiter) (void)parlio_del_rx_delimiter(delimiter);
    if (rx) (void)parlio_del_rx_unit(rx);
    if (decorated) {
        (void)bitscrambler_disable(tx->bs_handle);
        (void)parlio_tx_unit_undecorate_bitscrambler(tx);
    }
    if (tx) (void)parlio_del_tx_unit(tx);
    h[8]=err;h[12]=fnv(capture,4096);h[13]=fnv(raw,INPUT_BYTES);
    const esp_partition_t *p=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,0x42,"diagcap");
    size_t offset=0x12000+trial*0x2000;
    esp_err_t saved=p?ESP_OK:ESP_ERR_NOT_FOUND;
    if (p && offset+8192>p->size) saved=ESP_ERR_INVALID_SIZE;
    if (saved==ESP_OK) saved=esp_partition_erase_range(p,offset,8192);
    if (saved==ESP_OK) saved=esp_partition_write(p,offset+sizeof(h),capture,4096);
    if (saved==ESP_OK) saved=esp_partition_write(p,offset,h,sizeof(h));
    c5vrx2_trace_stage_detail(0x860+trial,saved,h[8],h[10],h[12]);
}

/* Diagnostic-only image copy. IDF 6.0.1 bitscrambler.c defines the v1
 * 12-byte header: trailing_bits at 8, eof_on at 10, nine words/instruction.
 * Never patch the embedded live program, its instructions, or its LUT.
 * Each completed trial is durable even if a later trial cannot finish. */
static void eof_sweep(uint8_t *raw, uint8_t *actual, const uint8_t *expected)
{
    const uint8_t *source=c5vrx2_wbfm_linear80_program();
    if (source[0]!=1 || source[2]!=3 || source[3]!=7) {
        c5vrx2_trace_stage(0x800,ESP_ERR_INVALID_VERSION);
        return;
    }
    size_t size=source[2]*4u+source[3]*36u+
                ((unsigned)source[4]+((unsigned)source[5]<<8))*4u;
    if (size>8192) return;
    uint8_t *program=malloc(size);
    if (!program) return;
    memcpy(program,source,size);
    const unsigned tails[]={8,9,10,12,16,24,32};
    for (unsigned trial=0;trial<sizeof(tails)/sizeof(tails[0]);++trial) {
        unsigned tail=tails[trial];
        program[8]=(tail*8)&255; program[9]=(tail*8)>>8;
        bitscrambler_handle_t bs=NULL;
        size_t written=0;
        memset(actual,0xa5,OUTPUT_CAPACITY);
        esp_err_t err=bitscrambler_loopback_create(&bs,SOC_BITSCRAMBLER_ATTACH_I2S0,OUTPUT_CAPACITY);
        if (err==ESP_OK) err=bitscrambler_load_program(bs,program);
        if (err==ESP_OK) err=bitscrambler_loopback_run(bs,raw,INPUT_BYTES,actual,OUTPUT_CAPACITY,&written);
        if (bs) bitscrambler_free(bs);
        unsigned different=0;
        for (size_t i=0;i<written && i<OUTPUT_BYTES;++i)
            different+=actual[i]!=expected[i];
        c5vrx2_trace_stage_detail(0x810+trial,err,tail,written,different);
        for (unsigned fast=0;fast<2;++fast) {
            uint32_t rows[20]; memset(rows,0xff,sizeof(rows));
            err=timed_tx(raw,fast?80000000:40000000,rows,program);
            c5vrx2_trace_stage_detail((fast?0x830:0x820)+trial,err,
                                      tail,rows[2],rows[3]);
        }
    }
    free(program);
    /* Control uses the existing Phase5 program, not linear80. No rate
     * inference from its different expansion ratio; test completion only. */
    uint32_t rows[20]; memset(rows,0xff,sizeof(rows));
    esp_err_t err=timed_tx(raw,40000000,rows,c5vrx2_wbfm_q4_phase5_program());
    c5vrx2_trace_stage_detail(0x840,err,40,rows[2],rows[3]);
    for (unsigned fast=0;fast<2;++fast) {
        memset(rows,0xff,sizeof(rows));
        err=timed_tx_mode(raw,fast?80000000:40000000,rows,source,true);
        for (unsigned run=0;run<4;++run) {
            uint32_t *r=rows+run*5;
            c5vrx2_trace_stage_detail(0x850+fast*4+run,(esp_err_t)r[4],r[1],r[2],r[3]);
        }
    }
    c5vrx2_trace_stage(0x84f,ESP_OK);
}

esp_err_t c5vrx2_linear80_oracle_run(void)
{
    uint32_t h[64]={0x4f30384cu,1,sizeof(h),INPUT_BYTES,OUTPUT_BYTES};
    h[6]=h[7]=h[8]=UINT32_MAX;
    h[12]=h[13]=UINT32_MAX;
    h[14]=CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ*1000000u;
    for (unsigned i=16;i<56;++i) h[i]=UINT32_MAX;
    const c5vrx2_calibration_t *cal=c5vrx2_calibration_get();
    if (cal->pedestal_code!=20 || cal->discriminator_gain!=2 || cal->polarity!=0)
        return ESP_ERR_INVALID_STATE;
    uint8_t *raw=heap_caps_aligned_alloc(64,INPUT_BYTES,MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
    uint8_t *actual=heap_caps_malloc(OUTPUT_CAPACITY,MALLOC_CAP_DMA|MALLOC_CAP_INTERNAL);
    uint8_t *expected=malloc(OUTPUT_BYTES);
    uint8_t *base=malloc(INPUT_BYTES/2);
    if (!raw || !actual || !expected || !base) {
        free(raw); free(actual); free(expected); free(base); return ESP_ERR_NO_MEM;
    }
    for (size_t i=0;i<INPUT_BYTES;++i) raw[i]=(i*73u+(i>>3)*29u+(i>>7)*11u+17u)&255;
    /* Direct controls must precede any BitScrambler allocation this boot.
     * Previous sequential test stopped before direct completion after BS
     * teardown. Order is deliberate to test that state-leak hypothesis. */
    const unsigned pad_order[]={2,6,3,4,5,0,1};
    for (unsigned i=0;i<7;++i) pad_capture(raw,actual,pad_order[i]);
    c5vrx2_wbfm_q4_phase5_reference(raw,INPUT_BYTES,base,INPUT_BYTES/2);
    unsigned a=0;
    for (size_t i=0;i<INPUT_BYTES/2;++i) {
        unsigned b=base[i];
        expected[i*4]=a; expected[i*4+1]=(3*a+b)/4;
        expected[i*4+2]=(a+b)/2; expected[i*4+3]=(a+3*b)/4;
        a=b;
    }
    memset(actual,0xa5,OUTPUT_CAPACITY);
    bitscrambler_handle_t bs=NULL;
    esp_err_t err=bitscrambler_loopback_create(&bs,SOC_BITSCRAMBLER_ATTACH_I2S0,OUTPUT_CAPACITY);
    size_t written=0;
    if (err==ESP_OK) err=bitscrambler_load_program(bs,c5vrx2_wbfm_linear80_program());
    if (err==ESP_OK) err=bitscrambler_loopback_run(bs,raw,INPUT_BYTES,actual,OUTPUT_CAPACITY,&written);
    h[5]=written; h[6]=err;
    if (bs) (void)bitscrambler_free(bs);
    if (err==ESP_OK) {
        h[7]=0;
        size_t count=written<OUTPUT_BYTES?written:OUTPUT_BYTES;
        for (size_t i=0;i<count;++i) if (actual[i]!=expected[i]) {
            if (h[8]==UINT32_MAX) h[8]=i;
            h[7]++;
        }
        h[7]+=OUTPUT_BYTES-count;
        /* Timing runs also execute on a mismatch, to preserve diagnostic
         * evidence; a mismatch is never promoted to a passing live gate. */
        h[12]=timed_tx(raw,40000000,h+16,c5vrx2_wbfm_linear80_program());
        h[13]=timed_tx(raw,80000000,h+36,c5vrx2_wbfm_linear80_program());
    }
    h[9]=fnv(raw,INPUT_BYTES); h[10]=fnv(actual,OUTPUT_CAPACITY);
    h[11]=fnv(expected,OUTPUT_BYTES); h[15]=OUTPUT_CAPACITY;
    const esp_partition_t *p=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,0x42,"diagcap");
    size_t total=sizeof(h)+OUTPUT_CAPACITY+OUTPUT_BYTES;
    esp_err_t saved=p?ESP_OK:ESP_ERR_NOT_FOUND;
    size_t erase=p?((total+p->erase_size-1)/p->erase_size)*p->erase_size:0;
    if (p && erase>p->size) saved=ESP_ERR_INVALID_SIZE;
    if (saved==ESP_OK) saved=esp_partition_erase_range(p,0,erase);
    if (saved==ESP_OK) saved=esp_partition_write(p,sizeof(h),actual,OUTPUT_CAPACITY);
    if (saved==ESP_OK) saved=esp_partition_write(p,sizeof(h)+OUTPUT_CAPACITY,expected,OUTPUT_BYTES);
    if (saved==ESP_OK) saved=esp_partition_write(p,0,h,sizeof(h));
    uint8_t check[256];
    for (size_t pos=0;saved==ESP_OK && pos<total;pos+=sizeof(check)) {
        size_t n=total-pos<sizeof(check)?total-pos:sizeof(check);
        saved=esp_partition_read(p,pos,check,n);
        for (size_t j=0;saved==ESP_OK && j<n;++j) {
            size_t k=pos+j;
            uint8_t v=k<sizeof(h)?((uint8_t*)h)[k]:
                k<sizeof(h)+OUTPUT_CAPACITY?actual[k-sizeof(h)]:expected[k-sizeof(h)-OUTPUT_CAPACITY];
            if (check[j]!=v) saved=ESP_FAIL;
        }
    }
    ESP_LOGW("linear80", "loop_err=%lu written=%lu mismatches=%lu first=%lu flash=%s",
             h[6],h[5],h[7],h[8],esp_err_to_name(saved));
    for (unsigned i=0;i<8;++i) {
        const uint32_t *r=h+16+i*5;
        ESP_LOGW("linear80","rate=%lu input=%lu elapsed_us=%lu mid_irq=0x%lx error=%lu",r[0],r[1],r[2],r[3],r[4]);
    }
    if (saved==ESP_OK) {
        eof_sweep(raw,actual,expected);
    }
    free(raw);free(actual);free(expected);free(base);
    bool failed=h[6]!=0 || h[7]!=0 || h[12]!=0 || h[13]!=0;
    for (unsigned i=0;i<8;++i) {
        const uint32_t *r=h+16+i*5;
        if (r[4]!=0 || (r[3]&1u)) failed=true;
    }
    return saved!=ESP_OK?saved:failed?ESP_FAIL:ESP_OK;
}
#endif
