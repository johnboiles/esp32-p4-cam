idf_component_register(
    SRCS "esp32-p4-cam.c"
    INCLUDE_DIRS "."
    REQUIRES 
        espressif__esp_cam_sensor
        espressif__esp_h264
        esp_http_server
        esp_wifi
        nvs_flash
        espressif__esp_video
        esp_ringbuf
)

# Add SC2336 sensor include directory
target_include_directories(${COMPONENT_LIB} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../managed_components/espressif__esp_cam_sensor/sensors/sc2336/include"
)
