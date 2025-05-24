#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_camera.h"

/*
 * The ESP32-P4 camera interface uses MIPI CSI. The standard esp_camera
 * driver currently targets DVP sensors, so for the P4 we expect a new
 * HAL driver. The configuration below is a placeholder and will need
 * to be updated with the actual MIPI CSI pins provided by the board.
 */
// Placeholder for P4 H264 encoder API
//#include "p4_h264.h"

static const char *TAG = "p4_cam";

static void camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn  = -1,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sccb_sda = 0,
        .pin_sccb_scl = 0,
        .pin_d7 = 0,
        .pin_d6 = 0,
        .pin_d5 = 0,
        .pin_d4 = 0,
        .pin_d3 = 0,
        .pin_d2 = 0,
        .pin_d1 = 0,
        .pin_d0 = 0,
        .pin_vsync = 0,
        .pin_href = 0,
        .pin_pclk = 0,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_YUV422,
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 12,
        .fb_count = 1,
    };

    ESP_ERROR_CHECK(esp_camera_init(&config));
}

static void h264_init(void)
{
    /*
     * The ESP32-P4 provides a built-in H264 encoder. The API for this
     * is not yet public, so this function is left as a placeholder for
     * the actual initialization code.
     */
    // TODO: Initialize P4 hardware H264 encoder
}

/*
 * The streaming task repeatedly captures a frame from the camera,
 * encodes it with the hardware H264 encoder and then would typically
 * send the encoded bitstream over Wi-Fi (for example via RTP/RTSP).
 */
static void stream_task(void *arg)
{
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }

        // TODO: Encode fb->buf with hardware H264 encoder
        // p4_h264_encode(fb->buf, fb->len);

        esp_camera_fb_return(fb);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    // TODO: initialise Wi-Fi connection so the encoded video can be streamed
    ESP_LOGI(TAG, "Initializing camera");
    camera_init();
    ESP_LOGI(TAG, "Initializing H264 encoder");
    h264_init();

    xTaskCreate(stream_task, "stream_task", 4096, NULL, 5, NULL);
}

