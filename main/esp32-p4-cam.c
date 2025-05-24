#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_types.h"
#include "esp_h264_enc_single.h"
#include "esp_h264_enc_param_hw.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_heap_caps.h"
#include "sc2336.h"
#include "esp_wifi_remote.h"
#include "esp_sccb_types.h"
#include "esp_sccb_i2c.h"
#include "driver/i2c_master.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include <freertos/ringbuf.h>

#ifndef MAP_FAILED
#define MAP_FAILED ((void *)-1)
#endif

#define BUFFER_COUNT 2

static const char *TAG = "ESP32-P4-CAM";

// Camera and encoder handles
static esp_cam_sensor_device_t *camera = NULL;
static esp_h264_enc_handle_t h264_encoder = NULL;
static httpd_handle_t server = NULL;  // Add server handle declaration
#define CAMERA_I2C_PORT 0

// Camera configuration for ESP32-P4 dev board with SC2336
#define CAMERA_SCCB_SCL_PIN    8     // I2C SCL pin
#define CAMERA_SCCB_SDA_PIN    7     // I2C SDA pin
#define CAMERA_RESET_PIN      -1     // Reset pin for SC2336
#define CAMERA_PWDN_PIN       -1     // Power down pin for SC2336
#define CAMERA_XCLK_PIN       -1     // XCLK pin for SC2336
#define CAMERA_XCLK_FREQ      24000000 // 24MHz typical for MIPI
#define CAMERA_PORT           ESP_CAM_SENSOR_MIPI_CSI

// Camera format (update as needed for your camera)
#define CAMERA_WIDTH 1280
#define CAMERA_HEIGHT 720
#define CAMERA_FPS 30
#define CAMERA_FORMAT ESP_CAM_SENSOR_PIXFORMAT_RGB565

// H264 encoder configuration
#define H264_BITRATE 2000000  // 2Mbps
#define H264_GOP 30
#define H264_QP_MIN 20
#define H264_QP_MAX 40

// WiFi configuration
#define WIFI_SSID "Boiles"
#define WIFI_PASS "stinaissohot"

// Camera frame buffer (allocate as needed)
static uint8_t *camera_frame_buf = NULL; // For RGB565
static uint8_t *h264_out_buf = NULL; // Output buffer (adjust size as needed)

// Ring buffer configuration
#define RINGBUF_SIZE (CAMERA_WIDTH * CAMERA_HEIGHT * 10)  // Size for 2 frames
#define MAX_FRAME_SIZE (CAMERA_WIDTH * CAMERA_HEIGHT)    // Maximum encoded frame size

// Ring buffer handle
static RingbufHandle_t frame_ringbuf = NULL;

// Frame structure to store in ring buffer
typedef struct {
    size_t size;
    uint8_t data[MAX_FRAME_SIZE];
} encoded_frame_t;

// Initialize ring buffer
static void init_frame_buffer(void)
{
    frame_ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (frame_ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create ring buffer");
    }
}

// HTTP server handlers
static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "video/H264");
    while (1) {
        // Get frame from ring buffer
        size_t size;
        encoded_frame_t *frame = xRingbufferReceive(frame_ringbuf, &size, pdMS_TO_TICKS(100));
        if (frame != NULL) {
            // Send H264 data
            esp_err_t res = httpd_resp_send_chunk(req, (const char *)frame->data, frame->size);
            if (res != ESP_OK) {
                // Connection closed or error
                vRingbufferReturnItem(frame_ringbuf, frame);
                break;
            }
            vRingbufferReturnItem(frame_ringbuf, frame);
        }
    }
    return ESP_OK;
}

static const httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
};

// WiFi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
}

// Initialize WiFi using ESP-Hosted
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize ESP-Hosted with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_remote_init(&cfg));

    // Connect to WiFi
    wifi_config_t sta_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_remote_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_remote_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_remote_start());
    ESP_ERROR_CHECK(esp_wifi_remote_connect());
}

// Initialize camera
static void camera_video_init(esp_sccb_io_handle_t sccb_handle)
{
    esp_cam_sensor_config_t cam_config = {
        .sccb_handle = sccb_handle,  // Use the SCCB handle from esp_video
        .reset_pin = CAMERA_RESET_PIN,
        .pwdn_pin = CAMERA_PWDN_PIN,
        .xclk_pin = CAMERA_XCLK_PIN,
        .xclk_freq_hz = CAMERA_XCLK_FREQ,
        .sensor_port = CAMERA_PORT,
    };

    // Use SC2336 camera detection function
    camera = sc2336_detect(&cam_config);
    if (!camera) {
        ESP_LOGE(TAG, "Failed to detect SC2336 camera");
        return;
    }

    // Query supported formats
    esp_cam_sensor_format_array_t fmt_array = {0};
    esp_cam_sensor_query_format(camera, &fmt_array);
    
    // Log all available formats
    ESP_LOGI(TAG, "Available camera formats:");
    for (uint32_t i = 0; i < fmt_array.count; ++i) {
        ESP_LOGI(TAG, "Format %d: %dx%d, format=%d", 
                 i,
                 fmt_array.format_array[i].width,
                 fmt_array.format_array[i].height,
                 fmt_array.format_array[i].format);
    }

    // Pick a format (try to match our config, but fall back to first available if needed)
    const esp_cam_sensor_format_t *chosen_fmt = NULL;
    for (uint32_t i = 0; i < fmt_array.count; ++i) {
        if (fmt_array.format_array[i].width == CAMERA_WIDTH &&
            fmt_array.format_array[i].height == CAMERA_HEIGHT &&
            fmt_array.format_array[i].format == CAMERA_FORMAT) {
            chosen_fmt = &fmt_array.format_array[i];
            break;
        }
    }

    // If no exact match, try to find a format with matching resolution
    if (!chosen_fmt) {
        for (uint32_t i = 0; i < fmt_array.count; ++i) {
            if (fmt_array.format_array[i].width == CAMERA_WIDTH &&
                fmt_array.format_array[i].height == CAMERA_HEIGHT) {
                chosen_fmt = &fmt_array.format_array[i];
                ESP_LOGW(TAG, "Using format with matching resolution but different pixel format");
                break;
            }
        }
    }

    // If still no match, use the first available format
    if (!chosen_fmt && fmt_array.count > 0) {
        chosen_fmt = &fmt_array.format_array[0];
        ESP_LOGW(TAG, "No matching format found, using first available format");
    }

    if (!chosen_fmt) {
        ESP_LOGE(TAG, "No camera format found");
        return;
    }

    ESP_LOGI(TAG, "Selected format: %dx%d, format=%d", 
             chosen_fmt->width, chosen_fmt->height, chosen_fmt->format);
    esp_cam_sensor_set_format(camera, chosen_fmt);

    // Start streaming
    int enable = 1;
    esp_cam_sensor_ioctl(camera, ESP_CAM_SENSOR_IOC_S_STREAM, &enable);
}

// H264 encoder initialization
static void h264_encoder_init(void)
{
    esp_h264_enc_cfg_hw_t enc_cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY, // For hardware encoder, use this format
        .gop = H264_GOP,
        .fps = CAMERA_FPS,
        .res = { .width = CAMERA_WIDTH, .height = CAMERA_HEIGHT },
        .rc = { 
            .bitrate = H264_BITRATE,
            .qp_min = H264_QP_MIN,
            .qp_max = H264_QP_MAX,
        },
    };
    esp_h264_err_t err = esp_h264_enc_hw_new(&enc_cfg, &h264_encoder);
    if (err != ESP_H264_ERR_OK || !h264_encoder) {
        ESP_LOGE(TAG, "Failed to initialize H264 encoder: %d", err);
        h264_encoder = NULL;
        return;
    }
    err = esp_h264_enc_open(h264_encoder);
    if (err != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "Failed to open H264 encoder: %d", err);
        esp_h264_enc_del(h264_encoder);
        h264_encoder = NULL;
        return;
    }
    ESP_LOGI(TAG, "H264 encoder initialized and opened");
}

// Frame capture and encoding (polling example)
static void capture_and_encode_task(void *arg)
{
    int fd;
    struct v4l2_capability capability;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    uint8_t *buffer[BUFFER_COUNT];
    size_t buffer_size[BUFFER_COUNT];

    // Open video device
    fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open video device");
        return;
    }

    // Query device capabilities
    if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0) {
        ESP_LOGE(TAG, "Failed to query device capabilities");
        close(fd);
        return;
    }

    // Check if device supports video capture
    if (!(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        ESP_LOGE(TAG, "Device does not support video capture");
        close(fd);
        return;
    }

    // Set format
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = CAMERA_WIDTH;
    format.fmt.pix.height = CAMERA_HEIGHT;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420; // Use YUV420 as it's commonly supported
    format.fmt.pix.field = V4L2_FIELD_ANY;

    if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
        ESP_LOGE(TAG, "Failed to set format");
        close(fd);
        return;
    }

    // Verify the format was set correctly
    if (format.fmt.pix.width != CAMERA_WIDTH || 
        format.fmt.pix.height != CAMERA_HEIGHT) {
        ESP_LOGW(TAG, "Format not set as requested. Got %dx%d instead of %dx%d",
                 format.fmt.pix.width, format.fmt.pix.height,
                 CAMERA_WIDTH, CAMERA_HEIGHT);
    }

    // Request buffers
    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;  // Request BUFFER_COUNT buffers
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        ESP_LOGE(TAG, "Failed to request buffers");
        close(fd);
        return;
    }

    // Map the buffers
    for (int i = 0; i < BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            ESP_LOGE(TAG, "Failed to query buffer");
            close(fd);
            return;
        }

        buffer_size[i] = buf.length;
        buffer[i] = mmap(NULL, buffer_size[i], PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffer[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "Failed to map buffer");
            close(fd);
            return;
        }

        // Queue the buffer
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "Failed to queue buffer");
            munmap(buffer[i], buffer_size[i]);
            close(fd);
            return;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE(TAG, "Failed to start streaming");
        for (int i = 0; i < BUFFER_COUNT; i++) {
            munmap(buffer[i], buffer_size[i]);
        }
        close(fd);
        return;
    }

    // Main capture loop
    while (1) {
        // Dequeue buffer
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            ESP_LOGE(TAG, "Failed to dequeue buffer");
            break;
        }

        // Check encoder handle before encoding
        if (!h264_encoder) {
            ESP_LOGE(TAG, "H264 encoder not initialized or opened");
            break;
        }

        // Encode the frame using H264 encoder
        esp_h264_enc_in_frame_t enc_in = {0};
        enc_in.raw_data.buffer = buffer[buf.index];
        enc_in.raw_data.len = buf.bytesused;

        esp_h264_enc_out_frame_t enc_out = {0};
        enc_out.raw_data.buffer = h264_out_buf;
        enc_out.raw_data.len = CAMERA_WIDTH * CAMERA_HEIGHT; // or a safe max size

        esp_h264_err_t err = esp_h264_enc_process(h264_encoder, &enc_in, &enc_out);
        if (err != ESP_H264_ERR_OK) {
            ESP_LOGE(TAG, "Failed to encode frame: %d", err);
        } else {
            // Allocate space in ring buffer
            void *item;
            if (xRingbufferSendAcquire(frame_ringbuf, &item, sizeof(encoded_frame_t), pdMS_TO_TICKS(100)) == pdTRUE) {
                encoded_frame_t *frame = (encoded_frame_t *)item;
                // Copy encoded data to ring buffer
                frame->size = enc_out.length;
                // Maybe could figure out how to put this directly into the ring buffer
                memcpy(frame->data, enc_out.raw_data.buffer, enc_out.length);
                
                // Send the frame to ring buffer
                xRingbufferSendComplete(frame_ringbuf, item);
                
            } else {
                // ESP_LOGW(TAG, "Ring buffer full, dropping frame");
            }
        }

        // Queue buffer back
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "Failed to queue buffer");
            break;
        }
    }

    // Stop streaming
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        ESP_LOGE(TAG, "Failed to stop streaming");
    }

    // Cleanup
    for (int i = 0; i < BUFFER_COUNT; i++) {
        munmap(buffer[i], buffer_size[i]);
    }
    close(fd);
}

// Initialize HTTP server
static void http_server_init(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &stream_uri);
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Prepare SCCB/I2C config for CSI
    static esp_video_init_csi_config_t csi_config = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = CAMERA_I2C_PORT,
                .scl_pin = CAMERA_SCCB_SCL_PIN,
                .sda_pin = CAMERA_SCCB_SDA_PIN,
            },
            .freq = 100000,
        },
        .reset_pin = CAMERA_RESET_PIN,
        .pwdn_pin = CAMERA_PWDN_PIN,
    };
    esp_video_init_config_t video_config = {
        .csi = &csi_config,
        .dvp = NULL,
        .jpeg = NULL,
    };
    ESP_ERROR_CHECK(esp_video_init(&video_config));

    // Allocate large buffers in SPIRAM with proper cache alignment
    const size_t cache_line_size = 64;  // ESP32 cache line size
    const size_t frame_size = CAMERA_WIDTH * CAMERA_HEIGHT * 2;
    const size_t h264_size = CAMERA_WIDTH * CAMERA_HEIGHT;
    
    // Align sizes up to cache line size
    const size_t aligned_frame_size = (frame_size + cache_line_size - 1) & ~(cache_line_size - 1);
    const size_t aligned_h264_size = (h264_size + cache_line_size - 1) & ~(cache_line_size - 1);
    
    camera_frame_buf = (uint8_t *)heap_caps_aligned_alloc(cache_line_size, aligned_frame_size, MALLOC_CAP_SPIRAM);
    h264_out_buf = (uint8_t *)heap_caps_aligned_alloc(cache_line_size, aligned_h264_size, MALLOC_CAP_SPIRAM);
    
    if (!camera_frame_buf || !h264_out_buf) {
        ESP_LOGE(TAG, "Failed to allocate frame buffers in SPIRAM");
        return;
    }

    // Initialize components
    wifi_init();
    // camera_video_init();
    
    // Only initialize H264 encoder if camera_video_init succeeded
    h264_encoder_init();
    http_server_init();
    // Start capture and encode task
    xTaskCreate(capture_and_encode_task, "capture_and_encode", 8192, NULL, 5, NULL);

    // Initialize ring buffer
    init_frame_buffer();

    ESP_LOGI(TAG, "ESP32-P4 Camera initialized");
}
