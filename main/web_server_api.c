#include "web_server.h"
#include "web_server_api.h"
#include "web_server_state.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "app_config.h"
#if !ENABLE_WEBSERVER_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_log.h"
#include "webrtc.h"
#include "hfhl_ws.h"
#include <arpa/inet.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

static const char *TAG = "web_server_api";
#define ENABLE_SIGNALING_TRACE 1
#define MAX_BODY 8192
// MAX_BODY limits JSON request payload size for all POST endpoints.

// API module responsibilities:
// - Handle browser <-> firmware signaling endpoints (/api/offer, /api/ice)
// - Normalize SDP/candidate data so libpeer on ESP32 accepts it reliably
// - Expose lightweight control/status endpoints used by the UI

/**
 * @brief Sets permissive CORS headers for the HTTP request.
 *
 * CORS is open because the hosted web app can be served from different origins
 * during development (e.g., local file/dev server vs device-hosted page).
 * 
 * @param req The HTTP request handle.
 */
static void set_cors_headers(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

/**
 * @brief Sends a generic JSON success response.
 *
 * Payload sent: `{"ok":true}`
 *
 * @param req The HTTP request handle.
 */
static void send_json_ok(httpd_req_t *req) {
  // Tiny common response payload used by control endpoints.
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, "{\"ok\":true}");
}

/**
 * @brief Attempts to resolve the remote socket peer address for this HTTP request.
 *
 * Extracts the client IP address from the underlying TCP socket associated with the HTTP request.
 * Handles both IPv4 and IPv4-mapped IPv6 addresses.
 *
 * @param req The HTTP request handle.
 * @param[out] out Buffer to store the resulting IP string.
 * @param out_len Length of the output buffer.
 * @return true if the IP address was successfully resolved, false otherwise.
 */
static bool get_client_ip(httpd_req_t *req, char *out, size_t out_len) {
  struct sockaddr_storage addr;
  socklen_t addr_len = sizeof(addr);
  // Convert HTTP request handle -> underlying TCP socket fd.
  int fd = httpd_req_to_sockfd(req);

  if (getpeername(fd, (struct sockaddr *)&addr, &addr_len) != 0) {
    // Can fail for some proxy/socket edge-cases.
    return false;
  }

  if (addr.ss_family == AF_INET) {
    // Case 1: Standard IPv4 connection. Extract the 32-bit address.
    struct sockaddr_in *in = (struct sockaddr_in *)&addr;
    // Convert the binary network-byte-order address into a dotted-decimal string (e.g. "192.168.1.5").
    if (!inet_ntop(AF_INET, &in->sin_addr, out, out_len))
      return false;
    return true;
  }

  if (addr.ss_family == AF_INET6) {
    // Case 2: IPv6 connection. We check for "IPv4-mapped IPv6" addresses (::ffff:a.b.c.d).
    // This happens when an IPv4 client connects to a dual-stack listening socket.
    struct sockaddr_in6 *in6 = (struct sockaddr_in6 *)&addr;
    const uint8_t *a = in6->sin6_addr.s6_addr;
    bool mapped_v4 = true;
    
    // Check if the first 80 bits (10 bytes) are all zero.
    for (int i = 0; i < 10; i++) {
      if (a[i] != 0x00) {
        mapped_v4 = false;
        break;
      }
    }
    
    // Check if the next 16 bits are 0xffff, which confirms the mapping prefix.
    if (mapped_v4 && a[10] == 0xff && a[11] == 0xff) {
      struct in_addr v4;
      // The actual IPv4 address is encapsulated in the final 4 bytes of the IPv6 address.
      memcpy(&v4, &a[12], sizeof(v4));
      // Convert the encapsulated IPv4 address into its standard string representation.
      if (inet_ntop(AF_INET, &v4, out, out_len))
        return true;
    }
  }

  return false;
}

/**
 * @brief Resolves the client IP address with a fallback mechanism.
 *
 * Prefers the actual socket peer IP address. If that fails, it falls back to
 * the latest IP assigned by the soft-AP DHCP server (useful for AP mode limitations).
 *
 * @param req The HTTP request handle.
 * @param[out] out Buffer to store the resulting IP string.
 * @param out_len Length of the output buffer.
 * @return true if an IP address was resolved, false otherwise.
 */
static bool resolve_client_ip(httpd_req_t *req, char *out, size_t out_len) {
  // 1. Primary Strategy: Extract the client IP directly from the active socket metadata.
  if (get_client_ip(req, out, out_len)) {
    // This provides the most accurate "ground truth" for the current request.
    return true;
  }

  // 2. Fallback Strategy: Use the latest IP address recorded by the Soft-AP DHCP handler.
  // This is essential for AP mode where socket-level peer resolution can occasionally fail 
  // during the rapid teardown/re-establishment of signaling connections.
  const char *last_sta_ip = web_server_get_last_sta_ip();
  if (last_sta_ip[0] != '\0') {
    snprintf(out, out_len, "%s", last_sta_ip);
    return true;
  }

  // 3. Final Safety: If all resolution attempts fail, ensure the output buffer is a valid empty string.
  if (out_len > 0) {
    out[0] = '\0';
  }
  return false;
}

/**
 * @brief Appends an SDP line with canonical CRLF endings to a buffer.
 *
 * @param[in,out] dst The destination string buffer.
 * @param cap The total capacity of the destination buffer.
 * @param line The null-terminated line string to append.
 * @return ESP_OK on success, ESP_ERR_NO_MEM if the buffer lacks capacity.
 */
static esp_err_t append_crlf_line(char *dst, size_t cap, const char *line) {
  // Append into already-built SDP text buffer.
  size_t used = strlen(dst);
  int wrote = snprintf(dst + used, cap - used, "%s\r\n", line);
  if (wrote < 0 || (size_t)wrote >= cap - used) {
    // Caller treats this as OOM-style capacity failure.
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

/**
 * @brief Rewrites an ICE candidate string to replace mDNS hostnames with a concrete IP.
 *
 * WebRTC browsers sometimes obscure local IPs with mDNS addresses (e.g., `*.local`).
 * The ESP32 WebRTC stack requires a concrete client IP to respond to. This function
 * finds the hostname token in the candidate string and replaces it with the resolved IP.
 *
 * @param candidate The original ICE candidate string.
 * @param ip The concrete IP string to substitute.
 * @param[out] out Buffer to store the rewritten candidate.
 * @param out_len Capacity of the output buffer.
 * @return true if the address was successfully replaced, false otherwise.
 */
static bool rewrite_candidate_address(const char *candidate, const char *ip,
                                      char *out, size_t out_len) {
  // p walks the source candidate text.
  const char *p = candidate;
  // field counts space-separated tokens in the candidate line.
  // Example:
  //   a=candidate:foundation 1 UDP 2122260223 host.local 54321 typ host
  //                   ^0      ^1 ^2  ^3         ^4
  int field = 0;
  // used is current write offset into output buffer.
  size_t used = 0;
  // replaced indicates whether we actually swapped any hostname with IP.
  bool replaced = false;

  if (!candidate || !ip || !out || out_len == 0)
    return false;

  out[0] = '\0';

  // Iterate through the candidate string, tokenizing by space
  while (*p) {
    // Skip any repeated whitespace between candidate tokens.
    while (*p == ' ')
      p++;
    if (!*p)
      break; // End of string reached

    // Record the start of the current token and measure its length
    const char *tok = p;
    size_t tok_len = 0;
    while (*p && *p != ' ') {
      p++;
      tok_len++;
    }

    // ICE candidate token #4 is the connection-address (RFC 5245 grammar).
    // E.g., in "a=candidate:1 1 UDP 2122260223 host.local 54321 typ host", "host.local" is field 4.
    if (field == 4 && tok_len >= 6) {
      // Search token text for ".local" suffix rather than strict equality
      // because some browsers can include generated host labels (e.g., uuid.local).
      for (size_t i = 0; i + 6 <= tok_len; i++) {
        if (memcmp(tok + i, ".local", 6) == 0) {
          // If a .local mDNS hostname is found, override this token pointer
          // to point to the concrete station IP string provided by the caller.
          tok = ip;
          tok_len = strlen(ip);
          replaced = true;
          break;
        }
      }
    }

    // Bounds checking: Ensure we have space for the token and optional space separator.
    if (used && used + 1 >= out_len)
      return false; // Output buffer overflow

    if (used) {
      // Re-insert single-space token separator into rebuilt candidate line.
      out[used++] = ' ';
    }
    
    // Check if we have room for the token bytes itself + final NUL terminator.
    if (used + tok_len >= out_len)
      return false; // Output buffer overflow

    // Append the token (either the original one or the substituted IP) to the output string.
    memcpy(out + used, tok, tok_len);
    used += tok_len;
    out[used] = '\0';

    // Redundant but explicit "did rewrite happen?" tracking.
    if (field == 4 && tok == ip) {
      replaced = true;
    }

    // Advance to next candidate token position.
    field++;
  }

  return replaced;
}

/**
 * @brief Checks if an ICE candidate is using the UDP transport protocol.
 *
 * Current data path expects UDP-only ICE transport. TCP candidates are generally dropped.
 *
 * @param candidate The ICE candidate string to inspect.
 * @return true if the candidate uses UDP or is null, false otherwise.
 */
static bool candidate_is_udp(const char *candidate) {
  const char *p = candidate;
  int field = 0;

  // Treat null candidate as "no-op/acceptable".
  if (!candidate)
    return true;

  while (*p) {
    // Same token parsing style as rewrite_candidate_address().
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

    // ICE candidate token #2 is the transport protocol (UDP/TCP).
    if (field == 2) {
      // Token #2 is transport token in ICE candidate grammar.
      // Accept any case variant of "UDP".
      return (tok_len == 3 && (tok[0] == 'U' || tok[0] == 'u') &&
              (tok[1] == 'D' || tok[1] == 'd') &&
              (tok[2] == 'P' || tok[2] == 'p'));
    }

    field++;
  }

  return true;
}

/**
 * @brief Cleans offer SDP before passing it to the WebRTC engine.
 *
 * 1) Drops non-UDP ICE candidates.
 * 2) Replaces mDNS candidate hostnames with the concrete client IP when possible.
 *
 * @param offer_sdp The original offer SDP string from the browser.
 * @param client_ip The resolved IP address of the client.
 * @param[out] sanitized_out Pointer to store the newly allocated, sanitized SDP string.
 * @return ESP_OK on success, ESP_ERR_NO_MEM on allocation failure.
 */
static esp_err_t sanitize_offer_sdp(const char *offer_sdp, const char *client_ip,
                                    char **sanitized_out) {
  // work: mutable copy for in-place line splitting
  // sanitized: final SDP text we forward to webrtc layer
  // rewritten_line: scratch space when we replace mDNS candidate host token
  char *work = strdup(offer_sdp);
  // Output is usually same size as input; keep modest headroom for rewrites.
  size_t cap = strlen(offer_sdp) + 64;
  char *sanitized = malloc(cap);
  char *rewritten_line = malloc(cap);
  char *cursor;
  char *line;
  int dropped = 0;
  int rewritten = 0;

  if (!work || !sanitized || !rewritten_line) {
    // Free any partial allocations before returning.
    free(work);
    free(sanitized);
    free(rewritten_line);
    return ESP_ERR_NO_MEM;
  }

  sanitized[0] = '\0';
  cursor = work;
  // Process line-by-line so we can selectively rewrite/drop candidate lines.
  while (cursor && *cursor) {
    // SDP is line-oriented text; split by newline.
    char *next = strchr(cursor, '\n');
    line = cursor;
    if (next) {
      // Terminate current line and move to next one.
      *next = '\0';
      cursor = next + 1;
    } else {
      cursor = NULL;
    }

    size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\r') {
      // Normalize input to LF-only so we control final CRLF output.
      line[len - 1] = '\0';
    }

    // Keep signaling robust on ESP32 by accepting only UDP candidates and
    // rewriting mDNS hostnames to the caller IP when possible.
    if (strncmp(line, "a=candidate:", 12) == 0) {
      // Candidate lines need transport/address policy filtering.
      const char *candidate_to_append = line;

      if (!candidate_is_udp(line)) {
        // Drop TCP candidates to keep transport assumptions simple/consistent.
        dropped++;
        continue;
      }

      if (client_ip && client_ip[0] != '\0' &&
          rewrite_candidate_address(line, client_ip, rewritten_line, cap)) {
        // Keep original line structure, only swap host token.
        candidate_to_append = rewritten_line;
        rewritten++;
      }

      // Re-emit candidate line (possibly rewritten) with canonical CRLF ending.
      if (append_crlf_line(sanitized, cap, candidate_to_append) != ESP_OK) {
        free(work);
        free(sanitized);
        free(rewritten_line);
        return ESP_ERR_NO_MEM;
      }
      continue;
    }

    // For all other SDP attributes/media/session lines, copy verbatim.
    if (append_crlf_line(sanitized, cap, line) != ESP_OK) {
      // Preserve all non-candidate lines unchanged.
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

/**
 * @brief Extracts the first media ID (a=mid:<id>) from an SDP offer.
 *
 * Used to normalize the generated answer SDP to appease stricter browser WebRTC implementations.
 *
 * @param sdp The SDP string to parse.
 * @param[out] out Buffer to store the extracted mid token.
 * @param out_len Length of the output buffer.
 * @return true if a mid was successfully extracted, false otherwise.
 */
static bool extract_first_mid(const char *sdp, char *out, size_t out_len) {
  // Use the first media-id as canonical across rewritten answer lines.
  // Typical line format: "a=mid:0"
  const char *mid = strstr(sdp, "a=mid:");
  if (!mid || out_len < 2)
    return false;

  mid += 6;
  size_t len = 0;
  // Scan forward character by character to find the end of the ID string.
  // We stop scanning if we hit:
  // 1. A null terminator (end of string)
  // 2. A carriage return or newline (end of SDP line)
  // 3. A space (just in case the mid token is followed by trailing properties)
  // We also ensure we don't exceed the capacity of 'out_len' (leaving 1 byte for null-terminator).
  while (mid[len] && mid[len] != '\r' && mid[len] != '\n' && mid[len] != ' ' &&
         len + 1 < out_len) {
    len++;
  }

  // If the length is 0, the "a=mid:" attribute was empty, so return failure.
  if (len == 0)
    return false;

  // Copy the identified 'len' bytes into the output buffer and null-terminate it.
  memcpy(out, mid, len);
  out[len] = '\0';
  return true;
}

/**
 * @brief Aligns answer mid/BUNDLE values to the offer's mid.
 *
 * Some strict browsers reject WebRTC answers if the BUNDLE/mid values do not exactly
 * mirror the offer. This function forces the answer to use the offer's first mid token.
 *
 * @param answer_sdp The raw answer SDP generated by the WebRTC engine.
 * @param offer_sdp The sanitized offer SDP received from the browser.
 * @param[out] normalized_out Pointer to store the newly allocated, normalized SDP string.
 * @return ESP_OK on success, ESP_ERR_NO_MEM on allocation failure.
 */
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
    // If offer has no explicit mid, keep answer untouched.
    *normalized_out = strdup(answer_sdp);
    return *normalized_out ? ESP_OK : ESP_ERR_NO_MEM;
  }

  // Similar sizing strategy as sanitize_offer_sdp().
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
      // Split answer SDP into mutable line fragments by null-terminating at the newline.
      *next = '\0';
      cursor = next + 1;
    } else {
      cursor = NULL;
    }

    // Strip trailing carriage return to normalize line endings.
    size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\r') {
      line[len - 1] = '\0';
    }

    const char *line_out = line;
    char rewritten_line[96];
    // Some browsers reject the answer if BUNDLE/mid values do not mirror the
    // offer exactly; force the answer to use the offer's first mid token.
    if (strncmp(line, "a=mid:", 6) == 0) {
      // Force media section mid to match browser offer.
      snprintf(rewritten_line, sizeof(rewritten_line), "a=mid:%s", offer_mid);
      line_out = rewritten_line;
      rewrites++;
    } else if (strncmp(line, "a=group:BUNDLE ", 15) == 0) {
      // Force BUNDLE group token to the same mid for strict peers.
      snprintf(rewritten_line, sizeof(rewritten_line), "a=group:BUNDLE %s",
               offer_mid);
      line_out = rewritten_line;
      rewrites++;
    }

    // Always append either original line or rewritten line.
    if (append_crlf_line(normalized, cap, line_out) != ESP_OK) {
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

/**
 * @brief Reads the full HTTP request body and NUL-terminates it.
 *
 * Allocates memory for the body based on the `Content-Length` header.
 *
 * @param req The HTTP request handle.
 * @param[out] out Pointer to store the allocated body string.
 * @return ESP_OK on success, ESP_FAIL on reading or allocation errors.
 */
static esp_err_t read_body(httpd_req_t *req, char **out) {
  int len = req->content_len;
  if (len <= 0 || len >= MAX_BODY) {
    // Bound payload size to avoid unexpected memory pressure.
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Content-Length");
    return ESP_FAIL;
  }
  // +1 for explicit NUL terminator so JSON parser can treat as C-string.
  char *buf = malloc(len + 1);
  if (!buf) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }
  int received = 0;
  // Read until full Content-Length to avoid parsing partial JSON payloads.
  while (received < len) {
    // Keep reading in case TCP packetization splits payload.
    int r = httpd_req_recv(req, buf + received, len - received);
    if (r <= 0) {
      free(buf);
      if (r == HTTPD_SOCK_ERR_TIMEOUT)
        // Distinguish socket timeout from generic parse errors.
        httpd_resp_send_408(req);
      return ESP_FAIL;
    }
    received += r;
  }
  buf[received] = '\0';
  *out = buf;
  return ESP_OK;
}

/**
 * @brief Parses the HTTP request body as a JSON object.
 *
 * Uses `read_body` to obtain the payload. If parsing fails, automatically sends
 * an HTTP 400 Bad Request response.
 *
 * @param req The HTTP request handle.
 * @param[out] root_out Pointer to store the parsed cJSON root object.
 * @return ESP_OK on success, ESP_FAIL on errors.
 */
static esp_err_t parse_json_body(httpd_req_t *req, cJSON **root_out) {
  char *body = NULL;
  if (read_body(req, &body) != ESP_OK) {
    // read_body already sent appropriate HTTP error.
    return ESP_FAIL;
  }

  // cJSON expects a NUL-terminated input buffer.
  cJSON *root = cJSON_Parse(body);
  free(body);
  if (!root) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
    return ESP_FAIL;
  }

  *root_out = root;
  return ESP_OK;
}

/**
 * @brief Complete offer-to-answer pipeline for `/api/offer`.
 *
 * 1. Sanitizes the incoming offer SDP.
 * 2. Invokes the WebRTC engine to generate an answer.
 * 3. Normalizes the answer SDP fields to match browser expectations.
 *
 * @param offer_sdp The raw offer SDP string.
 * @param client_ip The resolved client IP address.
 * @param[out] normalized_answer_out Pointer to store the final, normalized answer SDP.
 * @return ESP_OK on success, or an error code upon failure.
 */
static esp_err_t build_offer_answer(const char *offer_sdp, const char *client_ip,
                                    char **normalized_answer_out) {
  char *sanitized_offer_sdp = NULL;
  char *answer_sdp = NULL;
  char *normalized_answer_sdp = NULL;

  esp_err_t err = sanitize_offer_sdp(offer_sdp, client_ip, &sanitized_offer_sdp);
  if (err != ESP_OK || !sanitized_offer_sdp) {
    // Sanitizer allocates output buffer; propagate as memory error.
    return ESP_ERR_NO_MEM;
  }

  // This call blocks until webrtc.c posts the local answer SDP.
  err = webrtc_handle_offer(sanitized_offer_sdp, &answer_sdp);
  if (err != ESP_OK || !answer_sdp) {
    free(sanitized_offer_sdp);
    ESP_LOGW(TAG, "WebRTC offer handling failed err=0x%x", (unsigned int)err);
    return err;
  }

  // Normalize answer with context from sanitized offer.
  err = normalize_answer_sdp_mid(answer_sdp, sanitized_offer_sdp,
                                 &normalized_answer_sdp);
  // From here forward, answer_sdp is no longer needed.
  free(sanitized_offer_sdp);
  webrtc_free_answer(answer_sdp);
  if (err != ESP_OK || !normalized_answer_sdp) {
    free(normalized_answer_sdp);
    return ESP_ERR_NO_MEM;
  }

  // Ownership of normalized_answer_sdp transfers to caller.
  *normalized_answer_out = normalized_answer_sdp;
  return ESP_OK;
}

/**
 * @brief Serializes the answer SDP into a JSON payload and sends it.
 *
 * Output format: `{"type":"answer","sdp":"..."}`
 *
 * @param req The HTTP request handle.
 * @param normalized_answer_sdp The normalized answer SDP string.
 * @return ESP_OK on success, ESP_FAIL on errors (e.g., OOM).
 */
static esp_err_t send_offer_answer_json(httpd_req_t *req,
                                        const char *normalized_answer_sdp) {
  cJSON *resp = cJSON_CreateObject();
  if (!resp) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }

  cJSON_AddStringToObject(resp, "type", "answer");
  cJSON_AddStringToObject(resp, "sdp", normalized_answer_sdp);
  // Unformatted JSON keeps payload small over constrained AP link.
  char *resp_str = cJSON_PrintUnformatted(resp);
  cJSON_Delete(resp);

  if (!resp_str) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  // Send as one compact JSON string to reduce transfer overhead.
  httpd_resp_sendstr(req, resp_str);
  free(resp_str);
  return ESP_OK;
}

/**
 * @brief Lightweight status endpoint used by frontend polling.
 *
 * Endpoint: `GET /api/status`
 * Provides current connection status, stream rate, and device state.
 *
 * @param req The HTTP request handle.
 * @return ESP_OK
 */
static esp_err_t api_status_handler(httpd_req_t *req) {
  set_cors_headers(req);
  httpd_resp_set_type(req, "application/json");

  char json[128];
  // Keep status endpoint fast and allocation-free.
  snprintf(json, sizeof(json),
           "{\"soc\":%.1f,\"mode\":\"webrtc\",\"rate\":%d,\"live\":%s}",
           (float)50.0f, /* placeholder — real SoC passed via webrtc_set_soc */
           50, webrtc_is_connected() ? "true" : "false");

  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

/**
 * @brief Handles the initial WebRTC offer from the browser and returns the SDP answer.
 *
 * Endpoint: `POST /api/offer`
 * Expects a JSON payload with an "sdp" field containing the offer.
 *
 * @param req The HTTP request handle.
 * @return ESP_OK on successful request handling, ESP_FAIL otherwise.
 */
static esp_err_t api_offer_handler(httpd_req_t *req) {
  set_cors_headers(req);

  // 1) Parse JSON body and validate "sdp".
  cJSON *root = NULL;
  if (parse_json_body(req, &root) != ESP_OK) {
    return ESP_FAIL;
  }

  /* Extract "sdp" field from JSON body */
  cJSON *sdp_item = cJSON_GetObjectItemCaseSensitive(root, "sdp");
  if (!cJSON_IsString(sdp_item) || !sdp_item->valuestring) {
    // Reject malformed payloads early with clear 400.
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'sdp' field");
    return ESP_FAIL;
  }

  char client_ip[INET_ADDRSTRLEN] = {0};
  char *normalized_answer_sdp = NULL;
  // Useful for mDNS candidate rewrite path.
  resolve_client_ip(req, client_ip, sizeof(client_ip));
#if ENABLE_SIGNALING_TRACE
  ESP_LOGI(TAG, "POST /api/offer from %s (sdp len=%d)",
           client_ip[0] ? client_ip : "unknown", (int)strlen(sdp_item->valuestring));
#endif
  esp_err_t err =
      build_offer_answer(sdp_item->valuestring, client_ip, &normalized_answer_sdp);
  if (err != ESP_OK || !normalized_answer_sdp) {
    // Differentiate OOM from signaling/runtime errors for easier debugging.
    cJSON_Delete(root);
    if (err == ESP_ERR_NO_MEM) {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    } else {
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Signaling failed");
    }
    return ESP_FAIL;
  }
  cJSON_Delete(root);

  // 2) Return browser-consumable answer payload.
#if ENABLE_SIGNALING_TRACE
  ESP_LOGI(TAG, "POST /api/offer success (answer len=%d)",
           (int)strlen(normalized_answer_sdp));
#endif

  /* Build JSON answer: {"type":"answer","sdp":"..."} */
  err = send_offer_answer_json(req, normalized_answer_sdp);
  free(normalized_answer_sdp);
  return err;
}

/**
 * @brief Processes a single trickled ICE candidate from the browser.
 *
 * Filters non-UDP candidates, rewrites mDNS addresses if needed, and forwards
 * valid candidates to the WebRTC engine.
 *
 * @param req The HTTP request handle.
 * @param candidate The ICE candidate string.
 * @return ESP_OK (failures to add the candidate are logged but do not fail the request).
 */
static esp_err_t handle_ice_candidate(httpd_req_t *req, const char *candidate) {
  // candidate points to cJSON-owned memory; do not modify directly.
  char client_ip[INET_ADDRSTRLEN];
  // Candidate can be long; use exact length for safe scratch-buffer sizing.
  size_t candidate_len = strlen(candidate);
  // Allocate once so candidate may be rewritten in-place-like fashion.
  char *rewritten = malloc(candidate_len + INET_ADDRSTRLEN + 8);
  if (!rewritten) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    return ESP_FAIL;
  }

  // Ignore non-UDP trickle candidates: current transport path is UDP only.
  if (!candidate_is_udp(candidate)) {
    ESP_LOGI(TAG, "Dropped non-UDP trickle ICE candidate");
    free(rewritten);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"dropped\":true}");
    return ESP_OK;
  }

  bool got_ip = resolve_client_ip(req, client_ip, sizeof(client_ip));
  if (got_ip &&
      rewrite_candidate_address(candidate, client_ip, rewritten,
                                candidate_len + INET_ADDRSTRLEN + 8)) {
    // Switch pointer to rewritten buffer only when rewrite succeeded.
    ESP_LOGI(TAG, "Rewriting mDNS ICE candidate to client IP %s", client_ip);
    candidate = rewritten;
  }

  ESP_LOGI(TAG, "ICE candidate: %s", candidate);
  // Forward candidate into WebRTC engine.
  esp_err_t add_err = webrtc_add_ice_candidate(candidate);
  if (add_err != ESP_OK) {
    // Candidate add failures are logged but do not fail whole HTTP request.
    ESP_LOGW(TAG, "Failed to add ICE candidate err=0x%x", (unsigned int)add_err);
  }
  free(rewritten);
  send_json_ok(req);
  return ESP_OK;
}

/**
 * @brief Endpoint for accepting trickle-ICE payloads from the browser.
 *
 * Endpoint: `POST /api/ice`
 * Expects a JSON payload with a "candidate" field. Empty/partial payloads
 * are tolerated for browser compatibility.
 *
 * @param req The HTTP request handle.
 * @return ESP_OK
 */
static esp_err_t api_ice_handler(httpd_req_t *req) {
  set_cors_headers(req);

  // Payload usually looks like: {"candidate":"a=candidate:..."}.
  cJSON *root = NULL;
  if (parse_json_body(req, &root) != ESP_OK) {
    return ESP_FAIL;
  }

  cJSON *cand = cJSON_GetObjectItemCaseSensitive(root, "candidate");
  esp_err_t ret = ESP_OK;
  if (cJSON_IsString(cand) && cand->valuestring) {
    ret = handle_ice_candidate(req, cand->valuestring);
  } else {
    // Empty/partial ICE payloads are tolerated for browser compatibility.
    send_json_ok(req);
  }
  cJSON_Delete(root);
  return ret;
}

/**
 * @brief Optional stream configuration endpoint.
 *
 * Endpoint: `POST /api/start`
 * Accepts optional parameters (e.g., sample rate) to configure the stream.
 *
 * @param req The HTTP request handle.
 * @return ESP_OK
 */
static esp_err_t api_start_handler(httpd_req_t *req) {
  set_cors_headers(req);

  // Optional payload: {"rate": <number>}
  char *body = NULL;
  if (read_body(req, &body) != ESP_OK)
    return ESP_FAIL;

  // start endpoint remains permissive by design (invalid JSON -> ignore config).
  cJSON *root = cJSON_Parse(body);
  free(body);

  if (root) {
    cJSON *fpp_item  = cJSON_GetObjectItemCaseSensitive(root, "frames_per_packet");
    cJSON *freq_item = cJSON_GetObjectItemCaseSensitive(root, "packet_freq_hz");
    if (cJSON_IsNumber(fpp_item) && cJSON_IsNumber(freq_item)) {
      int fpp  = (int)fpp_item->valuedouble;
      int freq = (int)freq_item->valuedouble;
      webrtc_set_batch_params(fpp, freq);
      hfhl_ws_set_rate(fpp * freq);
    }
    cJSON_Delete(root);
  }

  send_json_ok(req);
  return ESP_OK;
}

/**
 * @brief Stops the active stream pipeline on the WebRTC side.
 *
 * Endpoint: `POST /api/stop`
 * Idempotent operation; safe to call even if the stream is inactive.
 *
 * @param req The HTTP request handle.
 * @return ESP_OK
 */
static esp_err_t api_stop_handler(httpd_req_t *req) {
  set_cors_headers(req);
  webrtc_stop_stream();
  hfhl_ws_stop();
  send_json_ok(req);
  return ESP_OK;
}

/**
 * @brief Reserved endpoint stub — zero-offset calibration is handled client-side.
 *
 * Endpoint: `POST /api/zero`
 * Zero offsets are pure display math applied in the browser; the firmware
 * streams raw absolute angles and does not participate in calibration.
 *
 * @param req The HTTP request handle.
 * @return ESP_OK
 */
static esp_err_t api_zero_handler(httpd_req_t *req) {
  set_cors_headers(req);
  send_json_ok(req);
  return ESP_OK;
}

/**
 * @brief Registers all API endpoints with the HTTP server.
 *
 * This function mounts handlers for `/api/status`, `/api/offer`, `/api/ice`,
 * `/api/start`, `/api/stop`, and `/api/zero`. It must be called before setting up
 * the static file fallback handler to ensure API routes take precedence.
 *
 * @param server The HTTP server handle to register routes on.
 */
void web_server_register_api_routes(httpd_handle_t server) {
  /* API endpoints are explicit and should win over static file fallback. */
  httpd_uri_t routes[] = {
      // Status polled by UI for connection/stream readiness hints.
      {.uri = "/api/status", .method = HTTP_GET, .handler = api_status_handler},
      // SDP offer/answer exchange.
      {.uri = "/api/offer", .method = HTTP_POST, .handler = api_offer_handler},
      // Trickle ICE candidate ingestion.
      {.uri = "/api/ice", .method = HTTP_POST, .handler = api_ice_handler},
      // Runtime control endpoints.
      {.uri = "/api/start", .method = HTTP_POST, .handler = api_start_handler},
      {.uri = "/api/stop", .method = HTTP_POST, .handler = api_stop_handler},
      {.uri = "/api/zero", .method = HTTP_POST, .handler = api_zero_handler},
  };
  // Register each route with ESP-IDF HTTP server.
  for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
    httpd_register_uri_handler(server, &routes[i]);
  }
}
