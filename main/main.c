/**
 * main.c - C5VRX-3 application entry point.
 *
 * Ultra-minimal single-purpose FPV receiver.
 * Starts RF frontend, starts video pipeline, then exits.
 * The hardware runs forever; the application is done.
 */

#include "rf.h"
#include "video.h"
#include "esp_log.h"

void app_main(void)
{
    ESP_ERROR_CHECK(rf_start());
    ESP_ERROR_CHECK(video_start());
    /* Hardware pipeline is running. Application has nothing more to do. */
}
