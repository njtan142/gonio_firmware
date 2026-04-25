#include "web_server_static.h"
#include "esp_http_server.h"
#include "app_config.h"
#if !ENABLE_WEBSERVER_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_log.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "web_server_static";

// Static-serving strategy:
// 1) map URI to /spiffs path
// 2) prefer .gz asset when available
// 3) infer content-type from extension
// 4) stream as HTTP chunks

/**
 * @brief Maps incoming HTTP URIs to the physical SPIFFS file paths.
 *
 * This performs the core routing translation. For example, a request for `/app.js` 
 * is mapped to `/spiffs/app.js`. The root path `/` is explicitly mapped to the 
 * single-page application shell at `/spiffs/index.html`.
 *
 * Crucially, this function also precomputes the `.gz` (gzip) variant of the path 
 * (e.g. `/spiffs/app.js.gz`) so the asset loader can quickly check for pre-compressed 
 * files without doing string manipulation later.
 *
 * @param uri The raw incoming HTTP request URI (e.g. "/", "/style.css").
 * @param[out] filepath Buffer to write the absolute uncompressed SPIFFS path into.
 * @param filepath_len Maximum size of the filepath buffer to prevent overflows.
 * @param[out] filepath_gz Buffer to write the absolute compressed SPIFFS path into.
 * @param filepath_gz_len Maximum size of the filepath_gz buffer.
 */
static void build_spiffs_paths(const char *uri, char *filepath,
                               size_t filepath_len, char *filepath_gz,
                               size_t filepath_gz_len) {
  if (strcmp(uri, "/") == 0) {
    // Root route serves app shell.
    snprintf(filepath, filepath_len, "/spiffs/index.html");
  } else {
    // Keep URI path as-is under /spiffs.
    snprintf(filepath, filepath_len, "/spiffs%s", uri);
  }
  // Precompute gzip variant for quick existence check.
  snprintf(filepath_gz, filepath_gz_len, "%s.gz", filepath);
}

/**
 * @brief Attempts to open a static asset, strongly preferring pre-compressed variants.
 *
 * Serving uncompressed HTML/JS over the ESP32's Wi-Fi is slow. During the build process, 
 * we gzip all frontend assets. This function first attempts to open the `.gz` file. 
 * If it exists, it sets the `is_gzip` flag so the caller can inject the `Content-Encoding: gzip` 
 * HTTP header, allowing the browser to decompress it on the fly.
 * 
 * If the `.gz` version is missing, it falls back to the raw, uncompressed file.
 *
 * @param filepath The fallback absolute path for the uncompressed file.
 * @param filepath_gz The preferred absolute path for the gzipped file.
 * @param[out] is_gzip Set to true if the returned FILE handle points to the gzipped variant.
 * @return FILE* An open file handle, or NULL if the file does not exist on the partition.
 */
static FILE *open_static_asset(const char *filepath, const char *filepath_gz,
                               bool *is_gzip) {
  // Prefer precompressed assets when available to reduce transfer time.
  FILE *f = fopen(filepath_gz, "rb");
  if (f) {
    // Caller will set Content-Encoding: gzip.
    *is_gzip = true;
    return f;
  }
  *is_gzip = false;
  return fopen(filepath, "rb");
}

/**
 * @brief Infers the HTTP content type based on the file extension.
 *
 * @param filepath Path of the file being served.
 * @return const char* MIME type string.
 */
static const char *guess_content_type(const char *filepath) {
  // Keep this in sync with frontend asset types in spiffs_image.
  if (strstr(filepath, ".html"))
    return "text/html; charset=utf-8";
  if (strstr(filepath, ".js"))
    return "text/javascript; charset=utf-8";
  if (strstr(filepath, ".css"))
    return "text/css; charset=utf-8";
  if (strstr(filepath, ".svg"))
    return "image/svg+xml";
  if (strstr(filepath, ".json"))
    return "application/json";
  if (strstr(filepath, ".png"))
    return "image/png";
  if (strstr(filepath, ".jpg") || strstr(filepath, ".jpeg"))
    return "image/jpeg";
  if (strstr(filepath, ".ico"))
    return "image/x-icon";
  // Safe fallback for unknown extensions.
  return "application/octet-stream";
}

/**
 * @brief Streams file contents to the HTTP client using chunked transfer encoding.
 *
 * The ESP32 cannot afford to `malloc()` a 200KB JS bundle into heap memory before sending it.
 * Instead, this function uses a small 1KB stack buffer to read the file in chunks. 
 * The ESP-IDF `httpd_resp_send_chunk` API automatically handles formatting the HTTP chunked 
 * framing (e.g. sending the hex length followed by the payload).
 *
 * @param req The active HTTP request object.
 * @param f The open FILE handle pointing to the asset to stream.
 * @return ESP_OK once the entire file has been sent and the zero-length terminator is written.
 */
static esp_err_t stream_file_chunks(httpd_req_t *req, FILE *f) {
  char buf[1024];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
    // The ESP-IDF HTTP server copies each chunk before returning.
    httpd_resp_send_chunk(req, buf, n);
  }
  // Final zero-length chunk marks end of response body.
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

/**
 * @brief Single entry-point used by the 404 fallback to serve any frontend asset.
 *
 * @param req The HTTP request handle.
 * @return ESP_OK on success, ESP_FAIL on error.
 */
static esp_err_t http_get_handler(httpd_req_t *req) {
  // We rely on req->uri exactly as received by HTTP server.
  const char *uri = req->uri;

  // 1. Map the request URI to physical SPIFFS file paths.
  // We generate both the standard path and a .gz variant to check for compressed assets.
  char filepath[600];
  char filepath_gz[604];
  build_spiffs_paths(uri, filepath, sizeof(filepath), filepath_gz,
                     sizeof(filepath_gz));

  // 2. Attempt to open the asset. The loader prioritizes .gz files to save
  // bandwidth and flash read time on the ESP32.
  bool is_gzip = false;
  FILE *f = open_static_asset(filepath, filepath_gz, &is_gzip);
  if (!f) {
    // If the file is missing from the partition, return a standard 404.
    ESP_LOGW(TAG, "Not found: %s", filepath);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  // 3. Set the appropriate Content-Type and add the GZIP encoding header if serving a compressed file.
  httpd_resp_set_type(req, guess_content_type(filepath));
  if (is_gzip)
    // Tells the browser to transparently decompress the incoming stream.
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

  // 4. Stream the file content using chunked transfer encoding to keep memory usage low and constant.
  stream_file_chunks(req, f);
  fclose(f);

  ESP_LOGI(TAG, "Served: %s", filepath);
  return ESP_OK;
}

/**
 * @brief Fallback HTTP 404 handler used to serve static files.
 *
 * Treats unresolved routes as static-file lookups.
 *
 * @param req The HTTP request handle.
 * @param err The error code that triggered the handler.
 * @return ESP_OK on success, ESP_FAIL on error.
 */
static esp_err_t http_404_handler(httpd_req_t *req, httpd_err_code_t err) {
  // Treat unresolved routes as static-file lookups.
  return http_get_handler(req);
}

/**
 * @brief Registers static file serving handlers with the HTTP server.
 *
 * All unmatched requests (including GET /) are caught by the 404 handler
 * which tries to serve the file from SPIFFS.
 * This effectively gives us a wildcard GET handler without registering one
 * that could conflict with explicit /api/ endpoints.
 *
 * @param server The HTTP server handle.
 */
void web_server_register_static_handlers(httpd_handle_t server) {
  /* All unmatched requests (including GET /) are caught by the 404 handler
   * which tries to serve the file from SPIFFS.
   * This effectively gives us a wildcard GET handler without registering one
   * that could conflict with explicit /api/ endpoints. */
  httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_handler);
}
