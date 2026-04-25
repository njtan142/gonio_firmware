# Electrogoniometer — Implementation Overview

This document describes what the system needs to implement based on the thesis
(Chapter 3: Methodology). Use this as the source of truth before writing any code.

---

## 0. Repository Layout

| Repo | URL | Contents |
|---|---|---|
| Firmware | *(local, no remote)* | ESP32-C3 firmware — this repo (`ssd1306_test`) |
| Web App | https://github.com/njtan142/goniometer-web-app | Single codebase built in two modes via ENV vars: 1) Local SPIFFS (ESP32 HTTP), 2) Vercel PWA (offline JSON playback) |

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
- User triggers calibration via HTTP API while limb is in neutral anatomical pose
- Firmware captures current raw sensor value as offset constant (`θ_offset`)
- Corrected angle uses modulo arithmetic to handle 360°→0° wrap-around:
  `θ_corrected = ((θ_raw - θ_offset) % 16384 + 16384) % 16384`
- Then converts to degrees: `degrees = θ_corrected × (360.0 / 16384.0)`

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
| `/api/zero` | POST | JSON body: `{ sensor: 0–3 }` — zero that sensor |
| `/api/start` | POST | JSON body: `{ mode: "udp"\|"tcp", rate: 10–100 }` — begin streaming |
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
| Live gauge (current joint angle) | UI done, needs real data feed |
| Live chart (rolling 29-point buffer) | UI done, needs real data feed |
| Normative range overlay + warnings | Done |
| Hold / freeze stream | Done (JS mock) |
| Set Zero (calibrate) | Done in JS; needs `POST /api/zero` call to firmware |
| Record / Stop | Done in JS; should trigger `POST /api/start` / `POST /api/stop` |
| Export CSV | Done |
| Battery % in sidebar | Hardcoded 92%; needs real SoC from packet or `/api/status` |
| Live indicator in sidebar | Always green; should reflect WebSocket connection state |
| 3D avatar animation | Three.js model loaded; Play button is a no-op — needs bone rotation from live data |
| History persistence | In-memory only; thesis strategy updated to JSON export (local app) -> JSON import (PWA mirror) |
| Sampling rate control | Slider exists; should send rate to firmware via `/api/start` |
| Mode selector (UDP/TCP) | Not in UI yet |
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
| WebRTC streaming | LFLL live data channel at 10–100 Hz; real sensor data, verified responsive in browser |

**Web App**
| Item | Notes |
|---|---|
| Live gauge | Fed by WebRTC real data |
| Live chart | Rolling 29-point buffer, fed by WebRTC real data |
| Normative range overlay + warnings | Done |
| Hold / freeze stream | Done |
| Export CSV | Done |

---

### 🔧 In Progress

**Web App**
| Item | Blocking on |
|---|---|
| Set Zero button | JS UI done — needs `POST /api/zero` wired to firmware calibration |
| Record / Stop | JS UI done — needs `POST /api/start` / `POST /api/stop` |
| Battery % in sidebar | Hardcoded 92% — needs real SoC from packet or `/api/status` |
| Live indicator | Always green — needs to reflect actual WebSocket/WebRTC connection state |

---

### 📋 To Do

**Firmware**
| Item | Notes |
|---|---|
| Zero-position calibration | `/api/zero` handler + per-sensor `θ_offset` in NVS; corrected angle formula in §2.5 |
| HTTP API | `/api/start`, `/api/stop`, `/api/status` endpoints |
| TCP/WebSocket HFHL endpoint | `/ws` on port 80 — bulk-flush recording mode; no latency requirement but lossless |
| Ring buffer | ~80 KB DRAM, ~2500 frames, drop-oldest, decouples sensor task from network flush |
| FSM | `IDLE` ↔ `ACTIVE_READINGS` state machine with sensor power gating (§2.3) |
| UDP socket task | LFLL native-client path; browser uses WebRTC data channel instead |
| Dynamic Payload Batching | Implement `/api/start` parameters for `frames_per_packet` (1-120) and `packet_freq_hz` (10-400). Adjust `streaming_task` to batch frames into a single payload buffer before transmitting. |

**Web App**
| Item | Notes |
|---|---|
| WebSocket connection + binary parser | Required for HFHL recording mode (`ws://192.168.4.1/ws`) |
| Mode selector UI | UDP / TCP toggle before starting a session |
| 3D avatar animation | Three.js bone rotation from live angle data; Play button currently no-op |
| Offline History (JSON Export/Import) | Requires JSON export button on ESP32 web app, and JSON import on PWA mirror |
| PDF report generation | Not started; thesis specifies jsPDF |
| WebRTC Latency Monitor | Real-time RTT/ping display in UI using `getStats()` API |
| Advanced Sampling Rate UI | Replace single slider with two: 1) Readings per packet (increments of 4, max 120), 2) Packet transmission frequency (10Hz-400Hz). Display the resulting "Total Readings per Second". |

---

## 7. Findings & Limitations

### 7.1 WebRTC Max Frequency & Buffer Exhaustion
The thesis outlines a "Low Fidelity, Low Latency" (LFLL) mode utilizing UDP payload batching to achieve a high volume of readings per second. Specifically, the theoretical sampling ceiling is bounded by the network MTU, not just the packet transmission rate:

*   **MTU Bound**: 1472 bytes per packet.
*   **Frame Size**: 12 bytes per frame.
*   **Maximum Readings per Packet ($N_{UDP}$)**: $1472 / 12 \approx 122$ frames (rounded to 120 max).
*   **Packet Transmission Rate ($f_{tx}$)**: 60 Hz (16.6 ms window) default to match standard video capture synchronization.
*   **Total Maximum Readings per Second**: $\approx 7,349 \text{ Hz}$ (calculated as 122 frames over a 16.6ms window, or exactly $122 \times 60 = 7,320$ readings/s).

Therefore, $7,349 \text{ Hz}$ is the **Total Maximum Readings per second**, *not* the packet send rate. The hardware can acquire data at 50,000Hz ($20\mu s$ per read), but the network bottleneck requires batching.

However, the **current implementation** sends exactly 1 frame (32 bytes representing 4 sensors) per `peer_connection_datachannel_send()` call. 
When attempting to push the unbatched transmission frequency significantly above ~100-200Hz, the firmware generates thousands of individual WebRTC packets per second. This massive protocol overhead immediately overwhelms the ESP32 network stack, leading to exhaustion of the `libpeer` and lwIP internal buffers. This produces the following logs:

```text
ERROR   ./components/libpeer/src/buffer.c       45      no enough space
W (174438) webrtc: dc tx failed rc=-1
ERROR   ./components/libpeer/src/buffer.c       45      no enough space
W (174448) webrtc: dc tx failed rc=-1
ERROR   ./components/libpeer/src/buffer.c       45      no enough space
W (174458) webrtc: dc tx failed rc=-1
ERROR   ./components/libpeer/src/buffer.c       45      no enough space
W (174468) webrtc: dc tx failed rc=-1
ERROR   ./components/libpeer/src/buffer.c       45      no enough space
W (174478) webrtc: dc tx failed rc=-1
ERROR   ./components/libpeer/src/buffer.c       45      no enough space
W (174488) webrtc: dc tx failed rc=-1
```

**Recommended Approaches to Fix or Gracefully Handle:**
1. **Implement Dynamic Payload Batching UI (Thesis Method)**: Update the Web App to utilize **two sliders**:
   - **Readings per Packet**: Increments of 4. Minimum of 1 (or 4 depending on active sensors). Maximum of 120.
   - **Packet Transmission Frequency**: Minimum 10 Hz. Maximum 400 Hz (which assumes a 2.4ms max fill time of 122 readings times 20us per sensor reading). The maximum frequency slider must dynamically update based on the number of readings set per packet(if readings per packet is lower than 120 then max frequency should be higher) to avoid exceeding hardware limits. Default is 60 Hz.
   - The UI must calculate and display **Total Readings per Second** (`Readings per Packet` × `Packet Frequency`).
2. **Backend Batching**: Update the backend `streaming_task` to accumulate readings into a large array (up to 120 frames) and send them as a single packet. This drastically reduces DTLS/SCTP encapsulation overhead and aligns with the LFLL design.
3. **Network Buffer Tuning**: Increase the lwIP UDP TX/RX buffers in `sdkconfig` and increase SCTP/DataChannel buffer sizes in `libpeer`. (Note: This only delays buffer-bloat and does not solve the CPU/network overhead bottleneck of unbatched frames).
4. **Backpressure & Drop Policies**: Monitor the return code of `peer_connection_datachannel_send()`. If it returns `-1` (failure), intelligently drop frames at the sensor acquisition level rather than flooding the socket.
