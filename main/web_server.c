#include "web_server.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "webrtc.h"
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>


static const char *TAG = "web_server";
#define ENABLE_SIGNALING_TRACE 1
static char g_last_sta_ip[INET_ADDRSTRLEN] = "";

#define WIFI_SSID "ESP32-Monitor"
#define WIFI_PASS "12345678"
#define MAX_BODY 8192

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *e = event_data;
    ESP_LOGI(TAG, "Station joined AID=%d", e->aid);
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *e = event_data;
    ESP_LOGI(TAG, "Station left AID=%d", e->aid);
  } else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_AP_STAIPASSIGNED) {
    ip_event_ap_staipassigned_t *e = event_data;
    snprintf(g_last_sta_ip, sizeof(g_last_sta_ip), IPSTR, IP2STR(&e->ip));
    ESP_LOGI(TAG, "Station got IP: %s", g_last_sta_ip);
  }
}

void wifi_init_softap(void) {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = WIFI_SSID,
              .ssid_len = sizeof(WIFI_SSID) - 1,
              .password = WIFI_PASS,
              .max_connection = 4,
              .authmode = WIFI_AUTH_WPA_WPA2_PSK,
              .channel = 1,
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

static bool get_client_ip(httpd_req_t *req, char *out, size_t out_len) {
  struct sockaddr_storage addr;
  socklen_t addr_len = sizeof(addr);
  int fd = httpd_req_to_sockfd(req);

  if (getpeername(fd, (struct sockaddr *)&addr, &addr_len) != 0) {
    return false;
  }

  if (addr.ss_family == AF_INET) {
    struct sockaddr_in *in = (struct sockaddr_in *)&addr;
    if (!inet_ntop(AF_INET, &in->sin_addr, out, out_len))
      return false;
    return true;
  }

  if (addr.ss_family == AF_INET6) {
    struct sockaddr_in6 *in6 = (struct sockaddr_in6 *)&addr;
    const uint8_t *a = in6->sin6_addr.s6_addr;
    bool mapped_v4 = true;
    for (int i = 0; i < 10; i++) {
      if (a[i] != 0x00) {
        mapped_v4 = false;
        break;
      }
    }
    if (mapped_v4 && a[10] == 0xff && a[11] == 0xff) {
      struct in_addr v4;
      memcpy(&v4, &a[12], sizeof(v4));
      if (inet_ntop(AF_INET, &v4, out, out_len))
        return true;
    }
  }

  return false;
}

static bool rewrite_candidate_address(const char *candidate, const char *ip,
                                      char *out, size_t out_len) {
  const char *p = candidate;
  int field = 0;
  size_t used = 0;
  bool replaced = false;

  if (!candidate || !ip || !out || out_len == 0)
    return false;

  out[0] = '\0';

  while (*p) {
    while (*p == ' ')
      p++;
    if (!*p)
      break;

    const char *tok = p;
    size_t tok_len = 0;
    while (*p && *p != ' ') {
      p++;
      tok_len++;
    }

    if (field == 4 && tok_len >= 6) {
      for (size_t i = 0; i + 6 <= tok_len; i++) {
        if (memcmp(tok + i, ".local", 6) == 0) {
          tok = ip;
          tok_len = strlen(ip);
          replaced = true;
          break;
        }
      }
    }

    if (used && used + 1 >= out_len)
      return false;
    if (used) {
      out[used++] = ' ';
    }
    if (used + tok_len >= out_len)
      return false;

    memcpy(out + used, tok, tok_len);
    used += tok_len;
    out[used] = '\0';

    if (field == 4 && tok == ip) {
      replaced = true;
    }

    field++;
  }

  return replaced;
}

static bool candidate_is_udp(const char *candidate) {
  const char *p = candidate;
  int field = 0;

  if (!candidate)
    return true;

  while (*p) {
    while (*p == ' ')
      p++;
    if (!*p)
      break;

    const char *tok = p;
    size_t tok_len = 0;
    while (*p && *p != ' ') {
      p++;
      tok_len++;
    }

    if (field == 2) {
      return (tok_len == 3 && (tok[0] == 'U' || tok[0] == 'u') &&
              (tok[1] == 'D' || tok[1] == 'd') &&
              (tok[2] == 'P' || tok[2] == 'p'));
    }

    field++;
  }

  return true;
}

static esp_err_t sanitize_offer_sdp(const char *offer_sdp, const char *client_ip,
                                    char **sanitized_out) {
  char *work = strdup(offer_sdp);
  size_t cap = strlen(offer_sdp) + 64;
  char *sanitized = malloc(cap);
  char *rewritten_line = malloc(cap);
  char *cursor;
  char *line;
  int dropped = 0;
  int rewritten = 0;

  if (!work || !sanitized || !rewritten_line) {
    free(work);
    free(sanitized);
    free(rewritten_line);
    return ESP_ERR_NO_MEM;
  }

  sanitized[0] = '\0';
  cursor = work;
  while (cursor && *cursor) {
    char *next = strchr(cursor, '\n');
    line = cursor;
    if (next) {
      *next = '\0';
      cursor = next + 1;
    } else {
      cursor = NULL;
    }

    size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\r') {
      line[len - 1] = '\0';
    }

    if (strncmp(line, "a=candidate:", 12) == 0) {
      const char *candidate_to_append = line;

      if (!candidate_is_udp(line)) {
        dropped++;
        continue;
      }

      if (client_ip && client_ip[0] != '\0' &&
          rewrite_candidate_address(line, client_ip, rewritten_line,
                                    cap)) {
        candidate_to_append = rewritten_line;
        rewritten++;
      }

      size_t used = strlen(sanitized);
      int wrote = snprintf(sanitized + used, cap - used, "%s\r\n",
                           candidate_to_append);
      if (wrote < 0 || (size_t)wrote >= cap - used) {
        free(work);
        free(sanitized);
        free(rewritten_line);
        return ESP_ERR_NO_MEM;
      }
      continue;
    }

    size_t used = strlen(sanitized);
    int wrote = snprintf(sanitized + used, cap - used, "%s\r\n", line);
    if (wrote < 0 || (size_t)wrote >= cap - used) {
      free(work);
      free(sanitized);
      free(rewritten_line);
      return ESP_ERR_NO_MEM;
    }
  }

  if (rewritten > 0 || dropped > 0) {
    ESP_LOGI(TAG, "Sanitized offer SDP (rewritten mDNS=%d, dropped non-UDP=%d)",
             rewritten, dropped);
  }

  free(work);
  free(rewritten_line);
  *sanitized_out = sanitized;
  return ESP_OK;
}

static bool extract_first_mid(const char *sdp, char *out, size_t out_len) {
  const char *mid = strstr(sdp, "a=mid:");
  if (!mid || out_len < 2)
    return false;

  mid += 6;
  size_t len = 0;
  while (mid[len] && mid[len] != '\r' && mid[len] != '\n' &&
         mid[len] != ' ' && len + 1 < out_len) {
    len++;
  }
  if (len == 0)
    return false;

  memcpy(out, mid, len);
  out[len] = '\0';
  return true;
}

static esp_err_t normalize_answer_sdp_mid(const char *answer_sdp,
                                          const char *offer_sdp,
                                          char **normalized_out) {
  char offer_mid[32];
  char *work = NULL;
  char *normalized = NULL;
  size_t cap;
  char *cursor;
  int rewrites = 0;

  if (!extract_first_mid(offer_sdp, offer_mid, sizeof(offer_mid))) {
    *normalized_out = strdup(answer_sdp);
    return *normalized_out ? ESP_OK : ESP_ERR_NO_MEM;
  }

  cap = strlen(answer_sdp) + 64;
  work = strdup(answer_sdp);
  normalized = malloc(cap);
  if (!work || !normalized) {
    free(work);
    free(normalized);
    return ESP_ERR_NO_MEM;
  }

  normalized[0] = '\0';
  cursor = work;
  while (cursor && *cursor) {
    char *next = strchr(cursor, '\n');
    char *line = cursor;
    if (next) {
      *next = '\0';
      cursor = next + 1;
    } else {
      cursor = NULL;
    }

    size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\r') {
      line[len - 1] = '\0';
    }

    const char *line_out = line;
    char rewritten_line[96];
    if (strncmp(line, "a=mid:", 6) == 0) {
      snprintf(rewritten_line, sizeof(rewritten_line), "a=mid:%s", offer_mid);
      line_out = rewritten_line;
      rewrites++;
    } else if (strncmp(line, "a=group:BUNDLE ", 15) == 0) {
      snprintf(rewritten_line, sizeof(rewritten_line), "a=group:BUNDLE %s",
               offer_mid);
      line_out = rewritten_line;
      rewrites++;
    }

    size_t used = strlen(normalized);
    int wrote = snprintf(normalized + used, cap - used, "%s\r\n", line_out);
    if (wrote < 0 || (size_t)wrote >= cap - used) {
      free(work);
      free(normalized);
      return ESP_ERR_NO_MEM;
    }
  }

  if (rewrites > 0) {
    ESP_LOGI(TAG, "Normalized answer SDP mid to '%s' (%d rewrite%s)", offer_mid,
             rewrites, rewrites == 1 ? "" : "s");
  }

  free(work);
  *normalized_out = normalized;
  return ESP_OK;
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
      if (r == HTTPD_SOCK_ERR_TIMEOUT)
        httpd_resp_send_408(req);
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
  if (strstr(filepath, ".html"))
    content_type = "text/html; charset=utf-8";
  else if (strstr(filepath, ".js"))
    content_type = "text/javascript; charset=utf-8";
  else if (strstr(filepath, ".css"))
    content_type = "text/css; charset=utf-8";
  else if (strstr(filepath, ".svg"))
    content_type = "image/svg+xml";
  else if (strstr(filepath, ".json"))
    content_type = "application/json";
  else if (strstr(filepath, ".png"))
    content_type = "image/png";
  else if (strstr(filepath, ".jpg") || strstr(filepath, ".jpeg"))
    content_type = "image/jpeg";
  else if (strstr(filepath, ".ico"))
    content_type = "image/x-icon";

  httpd_resp_set_type(req, content_type);
  if (is_gzip)
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

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
           (float)50.0f, /* placeholder — real SoC passed via webrtc_set_soc */
           50, webrtc_is_connected() ? "true" : "false");

  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

/* ── API: POST /api/offer ───────────────────────────────────────────────── */

static esp_err_t api_offer_handler(httpd_req_t *req) {
  set_cors_headers(req);

  char *body = NULL;
  if (read_body(req, &body) != ESP_OK)
    return ESP_FAIL;

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

  char client_ip[INET_ADDRSTRLEN] = {0};
  char *sanitized_offer_sdp = NULL;
  char *answer_sdp = NULL;
  char *normalized_answer_sdp = NULL;
  esp_err_t err;

  bool got_ip = get_client_ip(req, client_ip, sizeof(client_ip));
  if (!got_ip && g_last_sta_ip[0] != '\0') {
    snprintf(client_ip, sizeof(client_ip), "%s", g_last_sta_ip);
  }
#if ENABLE_SIGNALING_TRACE
  ESP_LOGI(TAG, "POST /api/offer from %s (sdp len=%d)",
           client_ip[0] ? client_ip : "unknown", (int)strlen(sdp_item->valuestring));
#endif
  err = sanitize_offer_sdp(sdp_item->valuestring, client_ip, &sanitized_offer_sdp);
  if (err != ESP_OK || !sanitized_offer_sdp) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }

  err = webrtc_handle_offer(sanitized_offer_sdp, &answer_sdp);

  if (err != ESP_OK || !answer_sdp) {
    free(sanitized_offer_sdp);
    cJSON_Delete(root);
    ESP_LOGW(TAG, "WebRTC offer handling failed err=0x%x", (unsigned int)err);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Signaling failed");
    return ESP_FAIL;
  }

  err = normalize_answer_sdp_mid(answer_sdp, sanitized_offer_sdp,
                                 &normalized_answer_sdp);
  free(sanitized_offer_sdp);
  cJSON_Delete(root);
  if (err != ESP_OK || !normalized_answer_sdp) {
    webrtc_free_answer(answer_sdp);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }

#if ENABLE_SIGNALING_TRACE
  ESP_LOGI(TAG, "POST /api/offer success (answer len=%d)", (int)strlen(answer_sdp));
#endif

  /* Build JSON answer: {"type":"answer","sdp":"..."} */
  cJSON *resp = cJSON_CreateObject();
  cJSON_AddStringToObject(resp, "type", "answer");
  cJSON_AddStringToObject(resp, "sdp", normalized_answer_sdp);
  webrtc_free_answer(answer_sdp);
  free(normalized_answer_sdp);

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
  if (read_body(req, &body) != ESP_OK)
    return ESP_FAIL;

  cJSON *root = cJSON_Parse(body);
  free(body);
  if (!root) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }

  cJSON *cand = cJSON_GetObjectItemCaseSensitive(root, "candidate");
  if (cJSON_IsString(cand) && cand->valuestring) {
    char client_ip[INET_ADDRSTRLEN];
    const char *candidate = cand->valuestring;
    size_t candidate_len = strlen(candidate);
    char *rewritten = malloc(candidate_len + INET_ADDRSTRLEN + 8);

    if (!rewritten) {
      cJSON_Delete(root);
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
      return ESP_FAIL;
    }

    if (!candidate_is_udp(candidate)) {
      ESP_LOGI(TAG, "Dropped non-UDP trickle ICE candidate");
      free(rewritten);
      cJSON_Delete(root);
      httpd_resp_set_type(req, "application/json");
      httpd_resp_sendstr(req, "{\"ok\":true,\"dropped\":true}");
      return ESP_OK;
    }

    bool got_ip = get_client_ip(req, client_ip, sizeof(client_ip));
    if (!got_ip && g_last_sta_ip[0] != '\0') {
      snprintf(client_ip, sizeof(client_ip), "%s", g_last_sta_ip);
      got_ip = true;
    }

    if (got_ip &&
        rewrite_candidate_address(candidate, client_ip, rewritten,
                                  candidate_len + INET_ADDRSTRLEN + 8)) {
      ESP_LOGI(TAG, "Rewriting mDNS ICE candidate to client IP %s", client_ip);
      candidate = rewritten;
    }

    ESP_LOGI(TAG, "ICE candidate: %s", candidate);
    esp_err_t add_err = webrtc_add_ice_candidate(candidate);
    if (add_err != ESP_OK) {
      ESP_LOGW(TAG, "Failed to add ICE candidate err=0x%x", (unsigned int)add_err);
    }
    free(rewritten);
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
  if (read_body(req, &body) != ESP_OK)
    return ESP_FAIL;

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
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 13;
  config.lru_purge_enable = true;
  config.stack_size = 8192;

  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return;
  }

  /* API endpoints (registered before wildcard so they take priority) */
  httpd_uri_t routes[] = {
      {.uri = "/api/status", .method = HTTP_GET, .handler = api_status_handler},
      {.uri = "/api/offer", .method = HTTP_POST, .handler = api_offer_handler},
      {.uri = "/api/ice", .method = HTTP_POST, .handler = api_ice_handler},
      {.uri = "/api/start", .method = HTTP_POST, .handler = api_start_handler},
      {.uri = "/api/stop", .method = HTTP_POST, .handler = api_stop_handler},
      {.uri = "/api/zero", .method = HTTP_POST, .handler = api_zero_handler},
  };
  for (int i = 0; i < 6; i++) {
    httpd_register_uri_handler(server, &routes[i]);
  }

  /* All unmatched requests (including GET /) are caught by the 404 handler
   * which tries to serve the file from SPIFFS. */
  httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_handler);

  ESP_LOGI(TAG, "HTTP server started on port 80");
}
