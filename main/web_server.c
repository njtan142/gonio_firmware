#include "web_server.h"
#include <string.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

static const char *TAG = "web_server";

#define WIFI_SSID  "ESP32-Monitor"
#define WIFI_PASS  "12345678"

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = event_data;
        ESP_LOGI(TAG, "Station joined AID=%d", e->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = event_data;
        ESP_LOGI(TAG, "Station left AID=%d", e->aid);
    }
}

void wifi_init_softap(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid           = WIFI_SSID,
            .ssid_len       = sizeof(WIFI_SSID) - 1,
            .password       = WIFI_PASS,
            .max_connection = 4,
            .authmode       = WIFI_AUTH_WPA_WPA2_PSK,
            .channel        = 1,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started — SSID: %s  http://192.168.4.1", WIFI_SSID);
}

static esp_err_t http_get_handler(httpd_req_t *req) {
    const char *uri = req->uri;

    char filepath[600];
    if (strcmp(uri, "/") == 0) {
        snprintf(filepath, sizeof(filepath), "/spiffs/index.html");
    } else {
        snprintf(filepath, sizeof(filepath), "/spiffs%s", uri);
    }

    char filepath_gz[604];
    snprintf(filepath_gz, sizeof(filepath_gz), "%s.gz", filepath);

    bool is_gzip = false;
    FILE *f = fopen(filepath_gz, "rb");
    if (f) {
        is_gzip = true;
    } else {
        f = fopen(filepath, "rb");
    }

    if (!f) {
        ESP_LOGW(TAG, "Not found: %s", filepath);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    const char *content_type = "application/octet-stream";
    if      (strstr(filepath, ".html")) content_type = "text/html; charset=utf-8";
    else if (strstr(filepath, ".js"))   content_type = "text/javascript; charset=utf-8";
    else if (strstr(filepath, ".css"))  content_type = "text/css; charset=utf-8";
    else if (strstr(filepath, ".svg"))  content_type = "image/svg+xml";
    else if (strstr(filepath, ".json")) content_type = "application/json";
    else if (strstr(filepath, ".png"))  content_type = "image/png";
    else if (strstr(filepath, ".jpg") || strstr(filepath, ".jpeg")) content_type = "image/jpeg";
    else if (strstr(filepath, ".ico"))  content_type = "image/x-icon";

    httpd_resp_set_type(req, content_type);
    if (is_gzip) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        httpd_resp_send_chunk(req, buf, n);
    }
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(f);

    ESP_LOGI(TAG, "Served: %s", filepath);
    return ESP_OK;
}

void web_server_start(void) {
    httpd_config_t config  = HTTPD_DEFAULT_CONFIG();
    config.server_port     = 80;
    config.max_open_sockets = 7;
    config.stack_size      = 8192;
    config.uri_match_fn    = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t root = { .uri = "/",  .method = HTTP_GET, .handler = http_get_handler };
    httpd_uri_t wild = { .uri = "/*", .method = HTTP_GET, .handler = http_get_handler };
    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &wild);
    ESP_LOGI(TAG, "HTTP server started on port 80");
}
