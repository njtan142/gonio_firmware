#include "web_server.h"
#include "web_server_api.h"
#include "web_server_state.h"
#include "web_server_static.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "app_config.h"
#if !ENABLE_WEBSERVER_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <arpa/inet.h>
#include <stdio.h>

static const char *TAG = "web_server";

// Fallback client address captured from AP events when socket peer lookup fails.
static char g_last_sta_ip[INET_ADDRSTRLEN] = "";

#define WIFI_SSID "ESP32-Monitor"
#define WIFI_PASS "12345678"

/**
 * @brief Gets the last station IP assigned by the AP.
 *
 * Shared read-only accessor used by the API module as a fallback.
 *
 * @return const char* String representing the IP address.
 */
const char *web_server_get_last_sta_ip(void) {
  // Shared read-only accessor used by API module.
  return g_last_sta_ip;
}

/**
 * @brief Tracks AP client join/leave events and IP assignments.
 *
 * Remembers the latest assigned station IP. The API layer uses this as a fallback
 * when per-request peer IP lookup fails.
 *
 * @param arg User data.
 * @param event_base Event base.
 * @param event_id Event ID.
 * @param event_data Event data payload.
 */
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
    // Cache most recent station IP for SDP/ICE mDNS rewrites.
    ip_event_ap_staipassigned_t *e = event_data;
    snprintf(g_last_sta_ip, sizeof(g_last_sta_ip), IPSTR, IP2STR(&e->ip));
    ESP_LOGI(TAG, "Station got IP: %s", g_last_sta_ip);
  }
}

/**
 * @brief Initializes the ESP32-C3 in standalone Access Point (SoftAP) mode.
 *
 * This function bootstraps the entire networking stack required for the local, 
 * offline WebRTC signaling architecture:
 * 1. It initializes Non-Volatile Storage (NVS). The Wi-Fi driver requires NVS to store 
 *    calibration data and PHY configurations. If the NVS partition is corrupted or version-mismatched, 
 *    it safely erases and reformats it.
 * 2. It brings up the LwIP networking core and the default system event loop.
 * 3. It configures the Wi-Fi interface to broadcast the "ESP32-Monitor" SSID.
 * 4. It wires up system event callbacks so we can passively monitor DHCP lease assignments, 
 *    which is critical for the SDP IP-rewriting logic.
 */
void wifi_init_softap(void) {
  // NVS must be ready before Wi-Fi init; erase/reinit if partition state changed.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // AP mode networking stack + default event loop.
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  // Creates default AP netif object used by esp_wifi in AP mode.
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // Register both generic AP station events and IP-assigned events.
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .ap =
          {
              // Public SSID visible to phone/browser client.
              .ssid = WIFI_SSID,
              .ssid_len = sizeof(WIFI_SSID) - 1,
              .password = WIFI_PASS,
              // Allow a few concurrent clients for debugging.
              .max_connection = 4,
              .authmode = WIFI_AUTH_WPA_WPA2_PSK,
              .channel = 1,
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  // Apply AP config to Wi-Fi interface.
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "WiFi AP started — SSID: %s  http://192.168.4.1", WIFI_SSID);
}

/**
 * @brief Bootstraps the ESP-IDF HTTP server for WebRTC signaling and static asset delivery.
 *
 * The HTTP server acts as the primary signaling channel. Browser clients connect here to 
 * download the web app (HTML/JS/CSS) and to exchange SDP offers/answers over REST APIs.
 *
 * Critical Tuning Parameters:
 * - `max_open_sockets = 13`: Browsers often open 6+ concurrent sockets to fetch static assets 
 *   (images, fonts) simultaneously. The default of 7 is too low and leads to stalled page loads.
 * - `lru_purge_enable = true`: If all 13 sockets are busy and a new request arrives, 
 *   the server aggressively closes the oldest idle keep-alive socket rather than dropping the new request.
 * - `stack_size = 8192`: Signaling involves heavy cJSON parsing and string manipulation 
 *   which easily overflows the default 4KB task stack.
 */
void web_server_start(void) {
  // Tune defaults for mixed API + static-content workload.
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  // Port 80 keeps browser URL simple.
  config.server_port = 80;
  // Extra sockets help when browser opens parallel asset/API requests.
  config.max_open_sockets = 13;
  // Let server reclaim least-recently-used sockets under pressure.
  config.lru_purge_enable = true;
  // Larger stack for JSON parsing + signaling transforms.
  config.stack_size = 8192;

  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return;
  }

  // API endpoints are registered first so they are not shadowed by 404 fallback.
  web_server_register_api_routes(server);
  web_server_register_static_handlers(server);

  ESP_LOGI(TAG, "HTTP server started on port 80");
}
