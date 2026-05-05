#include "web_server.h"
#include "web_server_api.h"
#include "web_server_state.h"
#include "web_server_static.h"
#include "hfhl_ws.h"
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

// Wi-Fi Access Point Credentials
#define WIFI_SSID "ESP32-Monitor" // The public SSID broadcasted by the ESP32 in Access Point mode.
#define WIFI_PASS "12345678"      // The WPA2-PSK password required for clients (phones/laptops) to connect to the AP.

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
  // Handle the event when a new client (station) successfully authenticates and connects to our AP.
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *e = event_data;
    ESP_LOGI(TAG, "Station joined AID=%d", e->aid);
    
  // Handle the event when a client disconnects from our AP.
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *e = event_data;
    ESP_LOGI(TAG, "Station left AID=%d", e->aid);
    
  // Handle the event when the internal DHCP server assigns an IP address to the connected client.
  } else if (event_base == IP_EVENT &&
             event_id == IP_EVENT_AP_STAIPASSIGNED) {
    // Cast the generic event payload to the IP assignment structure
    ip_event_ap_staipassigned_t *e = event_data;
    
    // Convert the raw IP address into a human-readable string and cache it in a global variable.
    // We need this cached IP later when generating WebRTC SDP offers and ICE candidates,
    // to dynamically rewrite mDNS addresses or 0.0.0.0 into the client's actual routable IP.
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
  // 1. Non-Volatile Storage (NVS) Initialization
  // The ESP32 Wi-Fi driver requires NVS to store calibration data and PHY configurations.
  esp_err_t ret = nvs_flash_init();
  
  // If the NVS partition is corrupted (no free pages) or contains a layout from a different 
  // ESP-IDF version, we must safely erase and reformat it before proceeding.
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // 2. Network Stack and Event Loop Initialization
  // Initialize the underlying LwIP TCP/IP stack.
  ESP_ERROR_CHECK(esp_netif_init());
  
  // Create the default system event loop which handles Wi-Fi and IP events asynchronously.
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  
  // Create the default network interface instance specifically configured for Access Point (AP) mode.
  esp_netif_create_default_wifi_ap();

  // Initialize the Wi-Fi driver with the default configuration parameters.
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // 3. Event Handler Registration
  // We need to passively monitor when clients connect to our AP and when the DHCP server 
  // assigns them an IP address. This is critical for our WebRTC SDP IP-rewriting logic.
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, wifi_event_handler, NULL, NULL));

  // 4. Access Point Configuration
  wifi_config_t wifi_config = {
      .ap =
          {
              // The public SSID that phones and browsers will see when scanning for Wi-Fi.
              .ssid = WIFI_SSID,
              .ssid_len = sizeof(WIFI_SSID) - 1,
              // WPA2 Password for the network.
              .password = WIFI_PASS,
              // Restrict to 4 concurrent clients to conserve memory and bandwidth.
              .max_connection = 4,
              // Use standard WPA2-PSK security.
              .authmode = WIFI_AUTH_WPA_WPA2_PSK,
              // Broadcast on channel 1.
              .channel = 1,
          },
  };
  
  // Set the hardware strictly to Access Point mode (disable Station mode).
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  
  // Commit the configuration to the Wi-Fi driver.
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  
  // Bring up the radio and start broadcasting the SSID.
  ESP_ERROR_CHECK(esp_wifi_start());

  // Reduce TX power from the default 20 dBm (80) to ~8.5 dBm (34).
  // The unit is 0.25 dBm per step. 20 dBm is the factory default and the RF PA
  // at that level is the primary heat source on the ESP32-C3 SuperMini's tiny LDO.
  // 8.5 dBm is still adequate for a room-scale AP (~10m radius) and cuts RF heat significantly.
  // Increase this value (max 80) if range becomes an issue.
  esp_wifi_set_max_tx_power(34);

  // Log successful startup. In AP mode, the ESP32's default gateway IP is always 192.168.4.1.
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

  // Declare the server handle, initially NULL.
  httpd_handle_t server = NULL;
  
  // Start the HTTP server. 
  // We pass the address of our handle (&server) as a double pointer. 
  // The esp_http_server library internally allocates the memory for the new server 
  // instance and writes the memory address back into our 'server' variable.
  // If the function returns ESP_OK, 'server' will hold the valid, instantiated handle.
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return;
  }

  // API endpoints are registered first so they are not shadowed by 404 fallback.
  web_server_register_api_routes(server);
  // HFHL WebSocket endpoint must be registered before the static wildcard handler.
  hfhl_ws_init(server);
  web_server_register_static_handlers(server);

  ESP_LOGI(TAG, "HTTP server started on port 80");
}
