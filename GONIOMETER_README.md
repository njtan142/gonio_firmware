# Electrogoniometer — Implementation Overview

This document describes what the system needs to implement based on the thesis
(Chapter 3: Methodology). Use this as the source of truth before writing any code.

---

## 0. Repository Layout

| Repo | URL | Contents |
|---|---|---|
| Firmware | *(local, no remote)* | ESP32-C3 firmware — this repo (`ssd1306_test`) |
| Web App | https://github.com/njtan142/goniometer-web-app — local: `C:\Users\njtan\Documents\GitHub\goniometer\web` | Single codebase built in two modes via ENV vars: 1) Local SPIFFS (ESP32 HTTP), 2) Vercel PWA (offline JSON playback) |

---

## 1. Hardware Stack

| Component | Role |
|---|---|
| ESP32-C3 SUPER MINI | Central compute + WiFi + SPI/I2C bus master |
| MT6701 (×4) | Magnetic absolute encoders via SSI (14-bit, 0–360°) |
| INA219 | High-side current sensor for coulomb counting (SoC) |
| SSD1306 OLED | Display: battery %, IP address |
| W25Q64 SPI flash (8MB) | Stores static web app files (HTML/JS/CSS) via SPIFFS |
| LiPo 400mAh | Power source |
| XL63070 buck-boost | 3.3V regulation |
| TP4057 | LiPo charger |

---

## 2. Firmware Architecture

### 2.1 Sensor Reading (SSI Protocol)
- Each MT6701 is read via **Synchronous Serial Interface (SSI)** — not I2C, not PWM
- Each SSI frame = **24 bits**: `[14-bit angle | 4-bit magnetic status | 6-bit CRC]`
- CRC is validated on every frame — invalid frames are **discarded**
- If the magnetic status field indicates weak/missing field → flag a hardware fault
- 4 sensors, each on its own CS pin via direct Chip Select topology on SPI bus

### 2.2 Ring Buffer
- A **FIFO ring buffer** in DRAM (~80 KB, ~2500 frames) decouples sensor acquisition
  from the slower Wi-Fi transmission
- Producer: high-priority ISR pushes validated 24-bit frames to head
- Consumer: network task polls and flushes chunks to the socket
- On buffer saturation: **drop-oldest** policy (overwrite tail), always keep most current data

### 2.3 State Machine (FSM)
- `IDLE` (Main Menu) → low power, sensors off, display on
- `ACTIVE_READINGS` → sensors on, ring buffer filling, network streaming
- Returns to IDLE only on explicit Stop or Back command

### 2.4 Power / SoC
- Coulomb counting via INA219: integrate current over time
- Boot-time estimate from OCV (open-circuit voltage) LiPo lookup table
- SoC encoded as **8-bit value** (0–255 = 0–100%, resolution ≈ 0.4%)

### 2.5 Zero-Position Calibration
- Handled entirely client-side — firmware streams raw absolute angles (0–360°) unchanged
- User clicks Set Zero in the UI while the limb is in the neutral anatomical pose
- Browser accumulates a `zeroOffsets` map (joint → degrees) and subtracts it from every incoming reading before display
- Offsets live in React state only; they reset on page reload (by design — no NVS or firmware involvement)

---

## 3. Wireless Communication

### 3.1 Binary Packet Format (8 bytes = 64 bits per sensor per timestep)

```
Bits 63–50  [14 bits]  Raw angle from MT6701 (0–16383)
Bits 49–18  [32 bits]  Timestamp (µs from esp_timer_get_time())
Bits 17–10  [ 8 bits]  State-of-Charge (0–255)
Bits  9– 0  [10 bits]  Status flags
                         bit 9: sensor 0 magnetic error
                         bit 8: sensor 1 magnetic error
                         bit 7: sensor 2 magnetic error
                         bit 6: sensor 3 magnetic error
                         bits 5–0: reserved
```

For a 4-sensor timestep, the ESP32 sends **4 × 8 = 32 bytes** sequentially.

### 3.2 UDP Mode — Low Fidelity, Low Latency (LFLL)
- **Purpose**: real-time live visualization
- Connectionless UDP, no ACK, packets may be lost
- Max safe payload: ~1472 bytes → fits ~46 timesteps for 4 sensors
- Web app should tolerate gaps; do NOT use for recording

### 3.3 TCP/WebSocket Mode — High Fidelity, High Latency (HFHL)
- **Purpose**: reliable data capture for clinical records
- WebSocket over TCP at `ws://192.168.4.1/ws`
- Firmware buffers 1–3 seconds of readings in DRAM, then flushes bulk
- TCP guarantees in-order, error-free delivery
- Adds noticeable latency — not suitable for live biofeedback, essential for recording

### 3.4 HTTP API (command & control only — no sensor data over HTTP)

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Serve `index.html` from SPIFFS |
| `/*` | GET | Serve any static asset from SPIFFS (.js, .css, .gz) |
| `/api/status` | GET | JSON: `{ soc, mode, rate, live }` |
| `/api/zero` | POST | No-op stub (calibration is client-side) |
| `/api/start` | POST | JSON body: `{ frames_per_packet: 1–46, packet_freq_hz: 10–400 }` — begin streaming |
| `/api/stop` | POST | Stop streaming, return to IDLE |

---

## 4. Web Application

### 4.1 Connection Layer (not yet implemented)
- On load: fetch `/api/status` to get current state and battery
- User selects mode (UDP/TCP) and rate before starting
- **TCP mode**: open `ws://192.168.4.1/ws`, receive binary ArrayBuffers
- **UDP mode**: browser cannot use raw UDP → implement via WebRTC data channel
  OR treat UDP-only as a native-client feature and use WebSocket for the web UI
- **Offline PWA Mode**: Build configured via environment variables, hosted externally (e.g., Vercel) to allow importing `.json` recordings without ESP32 connection
- Fall back to JS mock data when not connected to ESP32 (for dev/demo)

### 4.2 Binary Parser (web side)
```ts
// Each 8-byte frame per sensor
function parseFrame(view: DataView, offset: number) {
  const hi32 = view.getUint32(offset, false);     // big-endian
  const lo32 = view.getUint32(offset + 4, false);
  const angle_raw  = (hi32 >>> 18) & 0x3FFF;     // bits 63–50
  const timestamp  = ((hi32 & 0x3FFFF) << 14) | (lo32 >>> 18); // bits 49–18
  const soc        = (lo32 >>> 10) & 0xFF;        // bits 17–10
  const flags      = lo32 & 0x3FF;               // bits 9–0
  const degrees    = angle_raw * (360.0 / 16384.0);
  const soc_pct    = soc * (100.0 / 255.0);
  return { degrees, timestamp, soc_pct, flags };
}
```

### 4.3 Features to Implement

| Feature | Status |
|---|---|
| Live gauge (current joint angle) | Done — fed by WebRTC real data |
| Live chart (rolling 29-point buffer) | Done — fed by WebRTC real data |
| Normative range overlay + warnings | Done |
| Hold / freeze stream | Done |
| Set Zero (calibrate) | Done — client-side only; offset accumulated in `zeroOffsets` state and subtracted from incoming degrees |
| Record / Stop | Done in JS; needs `POST /api/start` / `POST /api/stop` |
| Export CSV | Done |
| Battery % in sidebar | Done — real SoC decoded from WebRTC packet |
| Live indicator in sidebar | Done — reflects actual WebRTC data channel state |
| Sampling rate control | Done — two sliders: Frames/pkt (1–46) and Pkt freq (10–400 Hz); displays Total readings/s |
| 3D avatar animation | Three.js model loaded; Play button is a no-op — needs bone rotation from live data |
| History persistence | In-memory only; thesis strategy updated to JSON export (local app) → JSON import (PWA mirror) |
| Mode selector (UDP/TCP) | Implicit — Record switches to WebSocket (HFHL), Stop returns to WebRTC (LFLL); no explicit toggle in UI |
| PDF report generation | Thesis specifies jsPDF — not implemented |

---

## 5. Sensor-to-Joint Mapping

The 4 SSI sensor slots map to joints based on device placement. The web app
should allow the clinician to configure which sensor corresponds to which joint.
Default mapping for gait analysis:

| Sensor index | Joint |
|---|---|
| 0 | Knee (Central Node — always present) |
| 1 | Hip (yaw-type external node) |
| 2 | Hip (pitch-type external node) |
| 3 | Ankle (yaw-type external node) |

For upper-extremity: sensor 0 = Elbow, others configured per session.

---

## 6. Project Status

### ✅ Done

**Firmware**
| Item | Notes |
|---|---|
| WiFi Soft-AP | SSID `ESP32-Monitor`, password `12345678`, IP `192.168.4.1` |
| HTTP server | Serves SPIFFS static assets on port 80 |
| External SPI flash | W25Q64 8 MB on SPI2, SPIFFS mounted at `/spiffs` |
| INA219 SoC | Coulomb counting at 100 Hz + OCV boot estimate |
| SSD1306 display | Voltage, SoC%, mAh/s, SSID, IP — updates at 1 Hz |
| MT6701 SSI driver | 4 sensors on SPI2, Mode 3, CRC-6 (x⁶+x+1), boot presence probe, magnetic error flags in telemetry |
| WebRTC streaming | LFLL live data channel; batched packets (1–46 timesteps × 32 bytes), 10–400 Hz send rate; verified responsive in browser |
| Dynamic payload batching | `streaming_task` batches `frames_per_packet` timesteps per DataChannel send; `/api/start` accepts `{ frames_per_packet, packet_freq_hz }` |
| TCP/WebSocket HFHL endpoint | `/ws` on port 80; `sensor_acq_task` (pri 6) + `ws_flush_task` (pri 4); auto-start on connect, auto-stop on disconnect; `/api/stop` tears down both paths |
| Ring buffer | Heap-allocated on WS connect, freed on disconnect — 500 samples × 32 B = 16 KB; drop-oldest policy; mutex-protected push/pop; freed between sessions so WebRTC DTLS heap is available in live mode |

**Web App**
| Item | Notes |
|---|---|
| Live gauge | Fed by WebRTC real data |
| Live chart | Rolling 29-point buffer, fed by WebRTC real data |
| Normative range overlay + warnings | Done |
| Hold / freeze stream | Done |
| Set Zero (calibrate) | Done — client-side; `zeroOffsets` state subtracted from incoming degrees |
| Export CSV | Done |
| Battery % in sidebar | Real SoC decoded from WebRTC packet |
| Live indicator | Reflects actual WebRTC data channel state |
| Sampling rate UI | Two sliders: Frames/pkt (1–46) and Pkt freq (10–400 Hz); displays Total readings/s |
| WebSocket client + binary parser | `WSClient` in `src/lib/ws.ts`; shares `parsePacket` with WebRTC path; dispatches per-timestep `onPacket` callbacks |
| Record / Stop | Record switches to WebSocket mode (calls `/api/stop` first, 600 ms delay, then connects `ws://192.168.4.1/ws`); Stop disconnects WebSocket and returns to WebRTC live mode |
| Mode switching | Automatic — driven by Record/Stop; WebRTC and WebSocket are mutually exclusive via mode-gated `useEffect` hooks |
| Chart X-axis: elapsed time | `chartTimestamps[]` (firmware µs) fed alongside angles; `ChartVisualization` shows 8 evenly-spaced time labels (ms/s) derived from `ts[i] − ts[0]`; falls back to frame index while buffer fills |
| History playback view | Clicking a history entry (with `rawData`) enters playback mode; `PlaybackPanel` replaces `ControlPanelTop` with scrub slider, play/pause, speed (0.25×–4×), CSV export, and exit; chart and gauge reflect current playback frame |
| 3D avatar bone animation | `AnimateModal` accepts `liveAngle` prop; `liveAngleRef` updated each render; Three.js animation loop applies `rotation.x = rad` to target bone every frame without re-triggering scene setup |
| JSON export | `handleExportJSON` in App.tsx — exports `{ metadata, frames[] }` JSON with relative timestamps; `CSV` + `JSON` + `Import` + `PDF` buttons in `ControlPanelTop` |
| JSON import | `handleImportJSON` — file picker accepts `.json`, validates format, reconstructs `HistorySession` with `rawData` and adds to history; Import button also in sidebar history header |
| PDF report generation | `handleGeneratePDF` using jsPDF v4; reports per-joint min/max/ROM/mean for current recording or active playback session |

---

---

### 📋 To Do

**Firmware**
| Item | Notes |
|---|---|
| FSM | `IDLE` ↔ `ACTIVE_READINGS` state machine with sensor power gating (§2.3) |
| Compiler optimization hardening | Race conditions in `webrtc.c` and `app_runtime.c` must be fixed first — see §7.5 |
| UDP socket backpressure at 400 Hz | `ENOBUFS` bursts from `socket.c:128` at max rate; investigate lwIP/FreeRTOS tick resolution — see §7.4 |

**Web App**
| Item | Notes |
|---|---|
| WebRTC Latency Monitor | Real-time RTT/ping display in UI using `getStats()` API |

---

## 7. Findings & Limitations

### 7.1 WebRTC Payload Batching — Implemented

The theoretical sampling ceiling is MTU-bound, not rate-bound:

*   **MTU Bound**: 1472 bytes per packet → max **46 timesteps** (4 sensors × 8 bytes × 46 = 1472 bytes).
*   **Packet Transmission Rate ($f_{tx}$)**: 10–400 Hz, default 60 Hz.
*   **Total Maximum Readings per Second**: 46 timesteps × 400 Hz = **18,400 Hz** (hardware acquires at 50,000 Hz; network is the ceiling).

**What was done:**
- `streaming_task` now accumulates `g_frames_per_packet` (1–46) timesteps into a static batch buffer and calls `peer_connection_datachannel_send()` once per packet interval, reducing DTLS/SCTP encapsulation overhead by up to 46×.
- `/api/start` accepts `{ frames_per_packet: N, packet_freq_hz: F }`.
- Web app replaced the single rate slider with two independent sliders (Frames/pkt, Pkt freq) and a calculated Total readings/s display.
- The web-side binary parser (`parsePacket`) handles variable-length packets (N × 32 bytes), dispatching one `onPacket` call per timestep.

### 7.2 libpeer Ring Buffer — Bypassed

**Root cause of `ERROR buffer.c:45 no enough space`:**

`libpeer`'s internal `data_rb` TX ring buffer (32 KB, set by `CONFIG_DATA_BUFFER_SIZE=32768` in `components/libpeer/CMakeLists.txt`) was only drained inside `case PEER_CONNECTION_COMPLETED:` in `peer_connection_loop`. Any ICE state regression — connectivity check timeout, DTLS keepalive, browser losing focus — caused draining to stop while `streaming_task` kept pushing. The 32 KB buffer filled in ~15 seconds and every subsequent write failed.

**Fundamental mismatch:** The ring buffer is designed for reliable media delivery (buffer → drain → retry later). The LFLL data channel uses `maxRetransmits: 0` (unreliable, drop-if-busy). These are incompatible. The ring buffer is the correct architecture for the HFHL TCP/WebSocket path (§2.2, §3.3), not for WebRTC LFLL.

**Fix:** Set `CONFIG_DATA_BUFFER_SIZE=0` in `components/libpeer/CMakeLists.txt`. With zero, `peer_connection_datachannel_send` bypasses the ring buffer and calls `sctp_outgoing_data` directly. Packets are sent immediately or dropped silently at the DTLS level — correct LFLL behaviour. No ring buffer means no fill-and-deadlock possible.

Note: the custom SCTP implementation (`CONFIG_USE_USRSCTP=0`) always returns success from `sctp_outgoing_data` regardless of DTLS send outcome, so Fix #4 (backpressure on return code) is not applicable in this code path.

### 7.3 libpeer SCTP Stream ID Bug — Fixed

When `CONFIG_DATA_BUFFER_SIZE=0` was first applied, the browser received no data despite the firmware reporting successful sends.

**Root cause:** `peer_connection_datachannel_send` always passes `sid=0` to `peer_connection_datachannel_send_sid`. In the buffered path, this `sid` was ignored — the drain code at `peer_connection.c:498` used `pc->sctp.data_sid` (the SID negotiated during `DATA_CHANNEL_OPEN`, which is 1). In the direct path (`#else` branch), `sid=0` was passed literally to `sctp_outgoing_data`. The browser discarded all SID-0 packets because no data channel is associated with SID 0.

**Fix:** The `#else` branch in `peer_connection_datachannel_send_sid` (`peer_connection.c`) was updated to use `pc->sctp.data_sid` instead of the caller-supplied `sid`, making it consistent with the buffered path.

### 7.4 UDP Socket Backpressure at Maximum Rate — To Investigate

At `fpp=1, pkt_hz=400` (400 packets/s × 32 bytes = 12.8 KB/s) the following error appears in bursts from `socket.c:128`:

```
ERROR   ./components/libpeer/src/socket.c   128   Failed to sendto: Not enough space
```

`ENOBUFS` / "Not enough space" from `sendto` indicates the lwIP UDP send buffer is transiently full — the kernel-side socket TX queue is exhausted before the radio can drain it. The streaming stats show normal packet counts resume after each burst, so the connection does not drop; packets are silently discarded.

**Observations:**
- Only occurs at or near 400 Hz. Lower rates (≤200 Hz) appear clean.
- Bursts of ~20–100 errors appear roughly every 2–8 seconds, then clear.
- Total packet count still increments between error bursts, meaning the task keeps running.

**Possible causes to investigate:**
- lwIP UDP TX buffer size (`CONFIG_LWIP_UDP_RECVMBOX_SIZE` / socket send buffer) too small for sustained 400 Hz bursts.
- `streaming_task` vTaskDelay tick resolution: at 400 Hz the delay is 2.5 ms; FreeRTOS tick rate (default 100 Hz = 10 ms) cannot resolve this, so the task fires in bursts rather than evenly, creating short-duration floods.
- Wi-Fi driver TX queue (`CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM`) saturation on the ESP32-C3 radio side.

**Workaround:** Keep `pkt_hz` ≤ 200 Hz for stable operation. Increasing `frames_per_packet` achieves the same total throughput with fewer UDP sends (e.g. fpp=2, pkt_hz=200 = same 400 readings/s, half the send rate).

---

### 7.5 Ring Buffer Heap Conflict with WebRTC DTLS — Fixed

The original ring buffer used a static array (`static uint8_t s_buf[2500][32]` = 80 KB in `.bss`). This permanently reduced available heap at boot, leaving insufficient memory for mbedTLS to allocate the DTLS SSL context when WebRTC attempted a peer connection:

```
ERROR ./components/libpeer/src/dtls_srtp.c  234  mbedtls_ssl_setup failed -0x7f00
ERROR ./components/libpeer/src/peer_connection.c  355  dtls_srtp_init failed -0x7f00
```

`-0x7f00` is `MBEDTLS_ERR_SSL_ALLOC_FAILED` — the mbedTLS heap allocator returned NULL.

**Fix:** The ring buffer is now heap-allocated (`malloc`) only when a WebSocket client connects (`rb_alloc()`), and freed (`rb_free()`) when it disconnects. Capacity was reduced from 2500 to 500 samples (80 KB → 16 KB) — sufficient burst absorption while leaving ample heap for the DTLS context in live mode.

**Browser-side coordination:** When Record is pressed, the web app calls `POST /api/stop` (firmware tears down WebRTC stream), then waits 600 ms before opening the WebSocket. This gap ensures libpeer has finished freeing its DTLS/SRTP heap before `rb_alloc()` is called. When Stop is pressed, the WebSocket closes and `rb_free()` is called immediately, restoring the heap for the next WebRTC session.

---

### 7.6 Compiler Optimization — Not Yet Hardened

Current build uses `-Og` (debug-friendly). The target is `-Os` (optimize for size), which would reclaim an estimated 100–200 KB of flash code space. The following race conditions exist in the codebase and are dormant under `-Og` but will manifest under `-Os` or `-O2`:

**Critical**
- `webrtc.c:37–39` — `g_tx_packets`, `g_tx_bytes`, `g_rx_messages` are plain `static uint32_t`. Written in `on_dc_message()` and reset in `on_dc_open()`, but read in `streaming_task()` with no mutex and no `volatile`. The compiler may cache these in registers. Fix: add `volatile`.
- `webrtc.c:150–152` — Same counters are reset inside `on_dc_open()` (called from `peer_main_task`) while `streaming_task()` may be incrementing them simultaneously. Fix: wrap the reset in `g_pc_mutex` or use atomic operations.

**High**
- `webrtc.c:217–220` — `g_frames_per_packet` and `g_packet_freq_hz` are snapshotted as two separate volatile reads; not atomic. A task could change one parameter between the two reads, producing an inconsistent batch size. Fix: snapshot both under `g_pc_mutex` or pack into a single struct written atomically.
- `webrtc.c:59` — `g_soc_pct` is `volatile float`. Volatile does not guarantee atomicity on floats; a torn write from `webrtc_set_soc()` during a read in `pack_frame()` can corrupt the value. Fix: read under mutex or copy to a local `uint8_t` before the cast.
- `app_runtime.c:61,73` — `last_ts == 0` used as first-run sentinel. Under aggressive optimization the compiler may hoist or reorder this check. Fix: use a dedicated `bool first_sample` flag instead.

Once all fixes are applied, switch via `idf.py menuconfig` → *Compiler options* → *Optimization Level* → **Optimize for size**, which sets `CONFIG_COMPILER_OPTIMIZATION_SIZE=y` in `sdkconfig`.
