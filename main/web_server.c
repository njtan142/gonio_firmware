#include "web_server.h"
#include "webrtc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "cJSON.h"

static const char *TAG = "web_server";

#define WIFI_SSID  "ESP32-Monitor"
#define WIFI_PASS  "12345678"
#define MAX_BODY   8192

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

/* ── Helpers ────────────────────────────────────────────────────────────── */

static void set_cors_headers(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t read_body(httpd_req_t *req, char **out) {
    int len = req->content_len;
    if (len <= 0 || len >= MAX_BODY) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Content-Length");
        return ESP_FAIL;
    }
    char *buf = malloc(len + 1);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }
    int received = 0;
    while (received < len) {
        int r = httpd_req_recv(req, buf + received, len - received);
        if (r <= 0) {
            free(buf);
            if (r == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
            return ESP_FAIL;
        }
        received += r;
    }
    buf[received] = '\0';
    *out = buf;
    return ESP_OK;
}

/* ── Static file handler / 404 catch-all ──────────────────────────────── */

static esp_err_t http_get_handler(httpd_req_t *req);

static esp_err_t http_404_handler(httpd_req_t *req, httpd_err_code_t err) {
    return http_get_handler(req);
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

/* ── API: GET /api/status ───────────────────────────────────────────────── */

static esp_err_t api_status_handler(httpd_req_t *req) {
    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json");

    char json[128];
    snprintf(json, sizeof(json),
             "{\"soc\":%.1f,\"mode\":\"webrtc\",\"rate\":%d,\"live\":%s}",
             (float)50.0f,        /* placeholder — real SoC passed via webrtc_set_soc */
             50,
             webrtc_is_connected() ? "true" : "false");

    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

/* ── API: POST /api/offer ───────────────────────────────────────────────── */

static esp_err_t api_offer_handler(httpd_req_t *req) {
    set_cors_headers(req);

    char *body = NULL;
    if (read_body(req, &body) != ESP_OK) return ESP_FAIL;

    /* Extract "sdp" field from JSON body */
    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *sdp_item = cJSON_GetObjectItemCaseSensitive(root, "sdp");
    if (!cJSON_IsString(sdp_item) || !sdp_item->valuestring) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'sdp' field");
        return ESP_FAIL;
    }

    char *answer_sdp = NULL;
    esp_err_t err = webrtc_handle_offer(sdp_item->valuestring, &answer_sdp);
    cJSON_Delete(root);

    if (err != ESP_OK || !answer_sdp) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Signaling failed");
        return ESP_FAIL;
    }

    /* Build JSON answer: {"type":"answer","sdp":"..."} */
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "type", "answer");
    cJSON_AddStringToObject(resp, "sdp",  answer_sdp);
    webrtc_free_answer(answer_sdp);

    char *resp_str = cJSON_PrintUnformatted(resp);
    cJSON_Delete(resp);
    if (!resp_str) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp_str);
    free(resp_str);
    return ESP_OK;
}

/* ── API: POST /api/ice ─────────────────────────────────────────────────── */

static esp_err_t api_ice_handler(httpd_req_t *req) {
    set_cors_headers(req);

    char *body = NULL;
    if (read_body(req, &body) != ESP_OK) return ESP_FAIL;

    cJSON *root = cJSON_Parse(body);
    free(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *cand = cJSON_GetObjectItemCaseSensitive(root, "candidate");
    if (cJSON_IsString(cand) && cand->valuestring) {
        webrtc_add_ice_candidate(cand->valuestring);
    }
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── API: POST /api/start ───────────────────────────────────────────────── */

static esp_err_t api_start_handler(httpd_req_t *req) {
    set_cors_headers(req);

    char *body = NULL;
    if (read_body(req, &body) != ESP_OK) return ESP_FAIL;

    cJSON *root = cJSON_Parse(body);
    free(body);

    if (root) {
        cJSON *rate_item = cJSON_GetObjectItemCaseSensitive(root, "rate");
        if (cJSON_IsNumber(rate_item)) {
            webrtc_set_stream_rate((int)rate_item->valuedouble);
        }
        cJSON_Delete(root);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── API: POST /api/stop ────────────────────────────────────────────────── */

static esp_err_t api_stop_handler(httpd_req_t *req) {
    set_cors_headers(req);
    webrtc_stop_stream();
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── API: POST /api/zero ────────────────────────────────────────────────── */

static esp_err_t api_zero_handler(httpd_req_t *req) {
    set_cors_headers(req);
    /* Zero-offset calibration is implemented on the browser side for now.
     * This endpoint exists for future firmware-side offset storage. */
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

/* ── Server start ───────────────────────────────────────────────────────── */

void web_server_start(void) {
    httpd_config_t config   = HTTPD_DEFAULT_CONFIG();
    config.server_port      = 80;
    config.max_open_sockets = 7;
    config.stack_size       = 8192;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    /* API endpoints (registered before wildcard so they take priority) */
    httpd_uri_t routes[] = {
        { .uri = "/api/status", .method = HTTP_GET,  .handler = api_status_handler },
        { .uri = "/api/offer",  .method = HTTP_POST, .handler = api_offer_handler  },
        { .uri = "/api/ice",    .method = HTTP_POST, .handler = api_ice_handler    },
        { .uri = "/api/start",  .method = HTTP_POST, .handler = api_start_handler  },
        { .uri = "/api/stop",   .method = HTTP_POST, .handler = api_stop_handler   },
        { .uri = "/api/zero",   .method = HTTP_POST, .handler = api_zero_handler   },
    };
    for (int i = 0; i < 6; i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    /* All unmatched requests (including GET /) are caught by the 404 handler
     * which tries to serve the file from SPIFFS. */
    httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_handler);

    ESP_LOGI(TAG, "HTTP server started on port 80");
}
