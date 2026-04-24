idf_component_register(SRCS "main.c" "app_runtime.c" "ssd1306_util.c" "font8x8_basic.c" "ina219.c"
                           "ext_flash.c" "web_server.c" "web_server_api.c" "web_server_static.c" "webrtc.c"
                     INCLUDE_DIRS "."
                     REQUIRES driver spi_flash spiffs esp_http_server esp_wifi nvs_flash esp_timer
                              json esp_netif sepfy__libpeer)
