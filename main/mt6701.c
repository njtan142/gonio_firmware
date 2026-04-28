#include "mt6701.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/spi_struct.h" /* GPSPI2 — ESP32-C3 SPI2 peripheral registers */

// MT6701 sensors share the SPI2 bus with the W25Q64 external flash.
// ext_flash.c initialises the bus (GPIO 4=CLK, 5=MISO, 6=MOSI); each sensor
// gets its own CS. The SPI master driver arbitrates bus access per device.
#define MT6701_SPI_HOST SPI2_HOST
#define MT6701_FREQ_HZ                                                         \
  (4 * 1000 * 1000) // 4 MHz — MT6701 supports up to ~15 MHz
#define MT6701_PROBE_TRIES                                                     \
  5 // reads used at boot to decide if a sensor is wired

// SSI frame bit layout (24 bits, MSB first):
//   [23:10] 14-bit angle     (0–16383 → 0–360°)
//   [ 9: 6]  4-bit mag status
//   [ 5: 0]  6-bit CRC
#define ANGLE_SHIFT 10
#define ANGLE_MASK 0x3FFF
#define STATUS_SHIFT 6
#define STATUS_MASK 0x0F
#define CRC_MASK 0x3F

// Mag-status bits that indicate a sensor fault.
#define MAG_WEAK_BIT (1 << 2)
#define MAG_STRONG_BIT (1 << 3)

static const char *TAG = "mt6701";

// Human-readable joint names matching the default gait mapping.
static const char *k_joint_name[MT6701_NUM_SENSORS] = {
    "Knee (central)",
    "Hip-yaw",
    "Hip-pitch",
    "Ankle-yaw",
};

// Hardware mapping array linking each logical sensor index (0-3) to its
// corresponding physical Chip Select (CS) GPIO pin as defined in app_config.h.
static const int k_cs_pins[MT6701_NUM_SENSORS] = {
    MT6701_CS0_IO,
    MT6701_CS1_IO,
    MT6701_CS2_IO,
    MT6701_CS3_IO,
};

// Array of opaque driver handles returned by spi_bus_add_device().
// Used for standard, non-fast-path polling transactions.
static spi_device_handle_t s_devs[MT6701_NUM_SENSORS];

// Caches the last successfully decoded floating-point angle (0-360°) for each sensor.
// Returned as a fallback if a subsequent read fails CRC or physical probing.
static float s_last_deg[MT6701_NUM_SENSORS];

// Tracks whether each sensor electrically responded to probing at boot.
// Missing sensors are skipped in the fast-path loop to prevent bus timeouts.
static bool s_present[MT6701_NUM_SENSORS];

// Latches any runtime faults for each sensor (e.g., CRC corruption or magnetic strength warnings).
// Used by the WebRTC telemetry loop to tag invalid data frames.
static bool s_error[MT6701_NUM_SENSORS];

/* Snapshot of GPSPI2.misc.val captured during mt6701_acquire_bus() for each
 * sensor.  The snapshot encodes which CSn_dis bits are clear (i.e. which
 * hardware CS signal is routed to the GPIO matrix for that sensor) so the
 * fast path can restore it before raising cmd.usr.  Captured once per bus
 * acquire, since cs_pin_id / cs polarity / ck_idle_edge never change at
 * runtime for these devices. */
static uint32_t s_misc_snapshot[MT6701_NUM_SENSORS];
static bool s_fast_armed; /* true between acquire_bus/release_bus  */

/* Fast-path diagnostic counters — dumped on release_bus so the log line
 * doesn't sit inside the 2.5 ms batch window.  Kept as plain uint32_t: only
 * the acquisition task writes them. */
static uint32_t s_fast_ok[MT6701_NUM_SENSORS];
static uint32_t s_fast_crc_fail[MT6701_NUM_SENSORS];
static uint32_t s_fast_mag_warn[MT6701_NUM_SENSORS];

/**
 * @brief Computes a 6-bit CRC over 18 bits of sensor data.
 *
 * Uses polynomial G(x) = x^6 + x + 1 as specified in Datasheet §6.8.2.
 * The 6-bit feedback term (excluding the implicit leading x^6) is 0x03.
 *
 * @param data18 The 18 data bits to compute the CRC for (raw >> 6).
 * @return uint8_t The computed 6-bit CRC value.
 */
static uint8_t crc6_compute(uint32_t data18) {
  uint8_t crc = 0;
  // Process each bit of the 18-bit data, starting from the Most Significant Bit
  // (MSB).
  for (int i = 17; i >= 0; i--) {
    // Extract the current input bit.
    uint8_t in = (data18 >> i) & 1;
    // Extract the feedback bit, which is the MSB of the current 6-bit CRC (bit
    // 5).
    uint8_t fb = (crc >> 5) & 1;
    // Shift the CRC left by 1, dropping the old MSB and making room for the
    // next bit. We mask with 0x3F to ensure the CRC remains strictly 6 bits.
    crc = (crc << 1) & 0x3F;
    // If the input bit and feedback bit differ, we need to apply the
    // polynomial.
    if (in ^ fb)
      // XOR with 0x03 (binary 000011). This represents the x^1 and x^0 terms
      // of the polynomial G(x) = x^6 + x + 1. The x^6 term is implicit.
      crc ^= 0x03;
  }
  return crc;
}

/**
 * @brief Probes a specific sensor to check for electrical presence.
 *
 * Attempts up to MT6701_PROBE_TRIES reads and looks for a valid CRC.
 * A passing CRC proves the sensor is electrically responding; magnetic status
 * faults (e.g., weak/strong field) are handled at runtime instead.
 *
 * @param i Sensor index to probe (0 to MT6701_NUM_SENSORS - 1).
 * @return true if the sensor responds with a valid CRC, false otherwise.
 */
static bool probe_sensor(int i) {
  ESP_LOGI(TAG, "probing sensor %d (CS GPIO %d) ...", i, k_cs_pins[i]);
  for (int t = 0; t < MT6701_PROBE_TRIES; t++) {
    // We only care about receiving data. The MT6701 is an SSI (Synchronous Serial Interface) 
    // device which essentially means it acts like a read-only SPI slave.
    uint8_t rx[3] = {0};
    
    // Configure the SPI transaction to clock out exactly 24 bits.
    // Since we are not sending any commands to the sensor (MOSI is don't-care), 
    // we only need to provide an rx_buffer to catch the 24 bits clocked in on MISO.
    spi_transaction_t txn = {
        .length = 24,   // Total clock cycles to generate
        .rxlength = 24, // Total bits to capture into the rx_buffer
        .rx_buffer = rx,
    };
    
    // Execute a synchronous (blocking) SPI transmission.
    if (spi_device_transmit(s_devs[i], &txn) != ESP_OK) {
      ESP_LOGW(TAG, "  [%d/%d] SPI transmit failed", t + 1, MT6701_PROBE_TRIES);
      continue;
    }
    // Assemble the 3 received bytes into a single 24-bit integer. (rx[0] is
    // MSB, rx[2] is LSB)
    uint32_t raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];

    // Extract the 14-bit angle data (bits 23:10) by shifting right and applying
    // the mask.
    uint16_t angle = (raw >> ANGLE_SHIFT) & ANGLE_MASK;

    // Extract the 4-bit magnetic status (bits 9:6) by shifting right and
    // applying the mask.
    uint8_t status = (raw >> STATUS_SHIFT) & STATUS_MASK;

    // Extract the 6-bit received CRC (bits 5:0). No shift needed, just mask the
    // lowest bits.
    uint8_t crc_recv = raw & CRC_MASK;

    // Calculate the expected CRC using the upper 18 bits of the payload (angle
    // + status).
    uint8_t crc_calc = crc6_compute(raw >> STATUS_SHIFT);

    // 0xFFFFFF / 0x000000 are floating-bus signatures: MISO idles high or
    // low when no sensor is driving it, and all-ones/all-zeros happen to
    // produce a passing CRC. Reject both regardless of CRC result.
    bool ok = (crc_calc == crc_recv) && (raw != 0xFFFFFF) && (raw != 0x000000);
    ESP_LOGI(TAG,
             "  [%d/%d] raw=0x%06lx  angle=%u  status=0x%x  crc_recv=0x%02x  "
             "crc_calc=0x%02x  %s",
             t + 1, MT6701_PROBE_TRIES, (unsigned long)raw, angle, status,
             crc_recv, crc_calc, ok ? "PASS" : "FAIL");
    if (ok)
      return true;
  }
  ESP_LOGW(TAG, "  sensor %d: all %d probes failed", i, MT6701_PROBE_TRIES);
  return false;
}

/**
 * @brief Initializes the SPI bus and probes all MT6701 magnetic encoders.
 *
 * Configures the SPI interface for Mode 3 as required by the sensor to ensure
 * stable sampling. Iterates through all possible sensor slots, probing each
 * to establish presence and logging any missing sensors.
 *
 * @return ESP_OK on successful initialization.
 */
esp_err_t mt6701_init(void) {
  for (int i = 0; i < MT6701_NUM_SENSORS; i++) {
    spi_device_interface_config_t dev_cfg = {
        // Datasheet §6.8.1: CLK idles HIGH (CPOL=1). The sensor drives each
        // bit on the rising edge and holds it until the next rising edge.
        // A master sampling on the rising edge (CPHA=1) reads the stable
        // value set up on the previous cycle → SPI Mode 3 (CPOL=1, CPHA=1).
        // Mode 2 (sample on falling) hits the sensor mid-transition and
        // produces corrupt frames.
        .mode = 3,
        .clock_speed_hz = MT6701_FREQ_HZ,
        .spics_io_num = k_cs_pins[i],
        .queue_size = 1,
    };
    // Register the sensor on the shared SPI bus using its specific Chip Select
    // (CS) pin. This gives us a handle (s_devs[i]) that we can use for standard
    // driver-based reads.
    esp_err_t err = spi_bus_add_device(MT6701_SPI_HOST, &dev_cfg, &s_devs[i]);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "sensor %d: spi_bus_add_device failed: %s", i,
               esp_err_to_name(err));
      return err;
    }

    // Reset the runtime state for this sensor slot before attempting hardware
    // detection.
    s_last_deg[i] = 0.0f;
    s_error[i] = false;

    // Probe the sensor by attempting a few initial SPI reads to verify
    // electrical presence.
    s_present[i] = probe_sensor(i);

    // Handle the case where the sensor fails to respond to probes.
    if (!s_present[i]) {
      // Permanently flag the slot as errored so downstream telemetry knows the
      // data is invalid.
      s_error[i] = true;

      // Sensor 0 is structurally critical (central knee node). If it's missing,
      // log a hard error.
      if (i == 0)
        // The system will still boot, but the core joint will stream zeros.
        ESP_LOGE(TAG, "sensor 0 (%s) not detected — check wiring",
                 k_joint_name[0]);
      else
        // For peripheral joints (hips/ankles), a missing sensor is treated as a
        // warning. The slot is simply disabled and will output 0°.
        ESP_LOGW(TAG, "sensor %d (%s) not detected — slot will output 0°", i,
                 k_joint_name[i]);
    } else {
      // The sensor responded correctly. Log its presence and its assigned GPIO.
      ESP_LOGI(TAG, "sensor %d (%s) present on CS GPIO %d", i, k_joint_name[i],
               k_cs_pins[i]);
    }
  }
  return ESP_OK;
}

/**
 * @brief Reads the current absolute angle from the specified sensor.
 *
 * Performs a 24-bit SPI read and decodes the angle, status, and CRC fields.
 * If the sensor is absent, or if the CRC or magnetic status checks fail,
 * it sets an error flag and returns the last known good angle.
 *
 * @param sensor The sensor index to read.
 * @return float The current angle in degrees (0.0 to 360.0).
 */
float mt6701_get_degrees(int sensor) {
  if (sensor < 0 || sensor >= MT6701_NUM_SENSORS)
    return 0.0f;

  // Skip the SPI round-trip entirely for absent sensors.
  if (!s_present[sensor])
    return 0.0f;

  uint8_t rx[3] = {0};
  spi_transaction_t t = {
      .length = 24, // clock out 24 bits; MOSI don't-care for read-only SSI
      .rxlength = 24,
      .rx_buffer = rx,
  };

  if (spi_device_polling_transmit(s_devs[sensor], &t) != ESP_OK) {
    // Set error flag and return the last known good position due to
    // transmission failure.
    s_error[sensor] = true;
    return s_last_deg[sensor];
  }

  // Assemble the 3 received bytes into a single 24-bit integer. (rx[0] is MSB,
  // rx[2] is LSB)
  uint32_t raw = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];

  // Extract the 14-bit angle data (bits 23:10) by shifting right and applying
  // the mask.
  uint16_t angle = (raw >> ANGLE_SHIFT) & ANGLE_MASK;

  // Extract the 4-bit magnetic status (bits 9:6) by shifting right and applying
  // the mask.
  uint8_t status = (raw >> STATUS_SHIFT) & STATUS_MASK;

  // Extract the 6-bit received CRC (bits 5:0). No shift needed, just mask the
  // lowest bits.
  uint8_t crc = raw & CRC_MASK;

  if (crc6_compute(raw >> STATUS_SHIFT) != crc) {
    ESP_LOGD(TAG, "sensor %d CRC fail raw=0x%06lx", sensor, (unsigned long)raw);
    // Set error flag and return the last known good position due to data
    // corruption.
    s_error[sensor] = true;
    return s_last_deg[sensor];
  }

  s_error[sensor] = !!(status & (MAG_WEAK_BIT | MAG_STRONG_BIT));
  s_last_deg[sensor] = angle * (360.0f / 16384.0f);
  return s_last_deg[sensor];
}

/**
 * @brief Checks if a specific sensor was detected during initialization.
 *
 * @param sensor The sensor index to check.
 * @return true if the sensor is present, false otherwise.
 */
bool mt6701_is_present(int sensor) {
  if (sensor < 0 || sensor >= MT6701_NUM_SENSORS)
    return false;
  return s_present[sensor];
}

/**
 * @brief Checks if the specified sensor has encountered a recent read error.
 *
 * This includes SPI transmission failures, CRC mismatches, or magnetic field
 * strength warnings reported by the sensor hardware.
 *
 * @param sensor The sensor index to check.
 * @return true if an error has occurred, false otherwise.
 */
bool mt6701_has_error(int sensor) {
  if (sensor < 0 || sensor >= MT6701_NUM_SENSORS)
    return true;
  return s_error[sensor];
}

/**
 * @brief Acquire the SPI2 bus exclusively for fast multi-sensor bulk reads.
 *
 * Locks the bus via sensor 0's device handle (any handle works — they share
 * the same bus mutex) and then performs one driver-based "priming" read per
 * present sensor.  Priming has two jobs:
 *   1. Let spi_hal_setup_device() configure GPSPI2.ctrl / user1 / timing
 *      registers for each sensor's mode, polarity, and clock divider.
 *   2. Capture GPSPI2.misc.val at the moment each sensor is active — this
 *      snapshot contains the CSn_dis bit pattern that routes the hardware
 *      CS signal to that sensor's GPIO.  We need per-sensor snapshots
 *      because the fast read bypasses spi_hal_setup_device() entirely.
 *
 * After this function returns, mt6701_get_degrees_fast() can switch between
 * sensors just by writing GPSPI2.misc.val = s_misc_snapshot[sensor] — a
 * single-register write that takes a few nanoseconds.
 */
void mt6701_acquire_bus(void) {
  spi_device_acquire_bus(s_devs[0], portMAX_DELAY);

  /* Prime every present sensor in order, recording the hardware-CS config
   * the driver ends up with each time.  Absent sensors are skipped — their
   * snapshot stays zero and get_degrees_fast() short-circuits on them. */
  uint8_t rx[3] = {0};
  spi_transaction_t prime = {.length = 24, .rxlength = 24, .rx_buffer = rx};
  for (int i = 0; i < MT6701_NUM_SENSORS; i++) {
    if (!s_present[i])
      continue;
    /* polling_transmit runs spi_hal_setup_device(), which calls
     * spi_ll_master_select_cs() → writes GPSPI2.misc appropriately. */
    esp_err_t err = spi_device_polling_transmit(s_devs[i], &prime);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "acquire_bus: sensor %d prime failed: %s", i,
               esp_err_to_name(err));
      s_misc_snapshot[i] = 0;
      continue;
    }
    s_misc_snapshot[i] = GPSPI2.misc.val;
    ESP_LOGD(TAG, "acquire_bus: sensor %d misc=0x%08lx (cs_dis=%c%c%c%c%c%c)",
             i, (unsigned long)s_misc_snapshot[i],
             GPSPI2.misc.cs0_dis ? '1' : '0', GPSPI2.misc.cs1_dis ? '1' : '0',
             GPSPI2.misc.cs2_dis ? '1' : '0', GPSPI2.misc.cs3_dis ? '1' : '0',
             GPSPI2.misc.cs4_dis ? '1' : '0', GPSPI2.misc.cs5_dis ? '1' : '0');
  }

  /* CRITICAL: the SPI2 bus is DMA-enabled (ext_flash_init() passed
   * SPI_DMA_CH_AUTO).  After the priming read, GPSPI2.dma_conf.dma_rx_ena
   * is still 1 and the DMA in-link points to the now-out-of-scope stack
   * buffer `rx[3]`.  If we leave DMA armed, fast-path reads clock MISO
   * into that stale DMA target instead of data_buf[0] — producing the
   * all-zeros frames we were chasing.  Disable DMA for the duration of
   * the fast-path burst and let the peripheral use its CPU-visible
   * work registers (data_buf[]) as fallback.  release_bus() does not
   * re-enable DMA explicitly; the next driver-based transaction (e.g.
   * mt6701_get_degrees() or an ext_flash access) will re-arm it via
   * s_spi_dma_prepare_data(). */
  GPSPI2.dma_conf.dma_rx_ena = 0;
  GPSPI2.dma_conf.dma_tx_ena = 0;
  /* Reset the RX AFIFO so any in-flight DMA-fed bits are flushed before
   * the first CPU-mode transaction — otherwise the hardware shift chain
   * may still hold junk from the priming transfer. */
  GPSPI2.dma_conf.rx_afifo_rst = 1;
  GPSPI2.dma_conf.buf_afifo_rst = 1;

  s_fast_armed = true;
}

/**
 * @brief Release the bus and log accumulated fast-read diagnostics.
 *
 * The stats counters are zeroed on each release so the next batch starts
 * fresh.  We only emit a log line at DEBUG level to keep steady-state
 * 60 Hz streaming quiet; bump esp_log_level_set("mt6701", ESP_LOG_DEBUG)
 * to see per-batch CRC/mag-warn counts.
 */
void mt6701_release_bus(void) {
  s_fast_armed = false;
  /* Aggregate 1-second windows of fast-read stats so we don't spam the UART
   * once per 2.5 ms batch.  The acquisition task calls release_bus() every
   * batch; we only emit when either a second has elapsed or an error was
   * seen, so a clean 60 Hz stream produces exactly one line per second. */
  static int64_t s_stats_window_start_us = 0;
  static uint32_t s_win_ok[MT6701_NUM_SENSORS];
  static uint32_t s_win_crc[MT6701_NUM_SENSORS];
  static uint32_t s_win_mag[MT6701_NUM_SENSORS];

  int64_t now_us = esp_timer_get_time();
  if (s_stats_window_start_us == 0)
    s_stats_window_start_us = now_us;

  for (int i = 0; i < MT6701_NUM_SENSORS; i++) {
    s_win_ok[i] += s_fast_ok[i];
    s_win_crc[i] += s_fast_crc_fail[i];
    s_win_mag[i] += s_fast_mag_warn[i];
    s_fast_ok[i] = s_fast_crc_fail[i] = s_fast_mag_warn[i] = 0;
  }

  if (now_us - s_stats_window_start_us >= 1000000) {
    for (int i = 0; i < MT6701_NUM_SENSORS; i++) {
      if (!s_present[i])
        continue;
      /* Log as WARN only if something went wrong this window; otherwise
       * keep it at DEBUG so a clean 18 kHz stream stays silent.  The
       * `ok` count alone would spam the UART at 60 Hz. */
      esp_log_level_t lvl =
          (s_win_crc[i] || s_win_mag[i]) ? ESP_LOG_WARN : ESP_LOG_DEBUG;
      ESP_LOG_LEVEL(lvl, TAG,
                    "fast stats 1s: sensor %d ok=%lu crc_fail=%lu mag_warn=%lu",
                    i, (unsigned long)s_win_ok[i], (unsigned long)s_win_crc[i],
                    (unsigned long)s_win_mag[i]);
      s_win_ok[i] = s_win_crc[i] = s_win_mag[i] = 0;
    }
    s_stats_window_start_us = now_us;
  }

  spi_device_release_bus(s_devs[0]);
}

/**
 * @brief Direct-register read of one MT6701 sensor (~7 µs).
 *
 * Must be called between mt6701_acquire_bus() / mt6701_release_bus().
 * Switches the hardware CS to @p sensor by restoring the snapshot captured
 * during acquire_bus(), re-arms the transfer length and MISO capture flag
 * (the driver's post-transaction cleanup clears usr_miso), then fires
 * cmd.usr and busy-polls for completion.  Total cost is dominated by the
 * 24 clocks @ 4 MHz = 6 µs bit-bang time; the register pokes add <200 ns.
 *
 * @return Angle in degrees [0, 360) on success, or the last valid reading
 *         on CRC failure (with s_error[sensor] set).
 */
float mt6701_get_degrees_fast(int sensor) {
  if (sensor < 0 || sensor >= MT6701_NUM_SENSORS)
    return 0.0f;
  if (!s_present[sensor])
    return 0.0f;
  if (!s_fast_armed) {
    /* Misuse — fall back to the driver path so we still return a sane
     * value.  Log once; spamming here would tank performance. */
    static bool warned = false;
    if (!warned) {
      ESP_LOGE(TAG, "get_degrees_fast called without acquire_bus — "
                    "falling back to driver path");
      warned = true;
    }
    return mt6701_get_degrees(sensor);
  }

  /* Route the hardware CS to this sensor's pin.  GPSPI2.misc also carries
   * ck_idle_edge and master_cs_pol — we captured them intact so this write
   * preserves mode-3 clock polarity and active-low CS. */
  GPSPI2.misc.val = s_misc_snapshot[sensor];

  /* Re-arm MISO capture and transfer length before each transaction.
   * spi_post_trans_cleanup() clears usr_miso after the priming read in
   * acquire_bus(), leaving the peripheral ready to clock SCK but not
   * capture MISO — data_buf would stay stale.
   *
   * IMPORTANT: the device was added to the driver without
   * SPI_DEVICE_HALFDUPLEX, so GPSPI2.user.doutdin is set to 1 (full duplex). In
   * full-duplex mode the peripheral only transfers when usr_mosi is enabled —
   * writing usr_mosi=0 here would cause cmd.usr to self-clear without actually
   * clocking SCK, and data_buf would stay stale (reading 0 repeatedly on first
   * acquire, then the last successful frame forever after).  Keep usr_mosi=1:
   * the MT6701 ignores MOSI (it's a read-only SSI device) and the outgoing bits
   * are don't-care. */
  /* Clear trans_done int so the next transaction has a clean completion
   * flag — even though we poll cmd.usr (which auto-clears on ESP32-C3),
   * the driver's assert() in spi_hal_user_start checks trans_done.  We
   * don't call that path, but clearing it matches the driver's ordering
   * and protects against future register-state leaks. */
  GPSPI2.dma_int_raw.trans_done = 0;

  GPSPI2.ms_dlen.ms_data_bitlen = 23; /* 24-bit transfer (N-1 encoding) */
  /* Write the whole user register in one shot to guarantee a consistent
   * state — bit-field writes would be three separate RMW cycles each of
   * which could race with a lingering register shadow.  We want:
   *   doutdin=1 (full duplex, as driver configured during priming),
   *   usr_miso=1, usr_mosi=1, everything else 0. */
  {
    typeof(GPSPI2.user) u = {.val = 0};
    u.doutdin = 1;
    u.usr_miso = 1;
    u.usr_mosi = 1;
    GPSPI2.user = u;
  }
  GPSPI2.data_buf[0] = 0; /* MOSI payload — don't-care */
  GPSPI2.cmd.update = 1;  /* latch user/ms_dlen into shadow regs */
  while (GPSPI2.cmd.update)
    ;
  GPSPI2.cmd.usr = 1;
  while (GPSPI2.cmd.usr)
    ; /* busy-poll — completes in ~6 µs at 4 MHz */

  /* data_buf[0] stores received bytes little-endian: first received byte at
   * bits [7:0].  MT6701 sends MSByte first → reconstruct as big-endian 24-bit.
   */
  uint32_t buf = GPSPI2.data_buf[0];
  uint32_t raw =
      ((buf & 0x0000FFu) << 16) | (buf & 0x00FF00u) | ((buf & 0xFF0000u) >> 16);

  uint8_t crc_recv = raw & CRC_MASK;
  uint8_t crc_calc = crc6_compute(raw >> STATUS_SHIFT);
  if (crc_calc != crc_recv) {
    s_fast_crc_fail[sensor]++;
    /* Verbose-only: one line per failure would flood the UART at 60 Hz.
     * Promote to WARN if every read for this sensor is failing. */
    ESP_LOGV(TAG, "fast sensor %d CRC fail raw=0x%06lx recv=0x%02x calc=0x%02x",
             sensor, (unsigned long)raw, crc_recv, crc_calc);
    s_error[sensor] = true;
    return s_last_deg[sensor];
  }

  // Extract the 4-bit magnetic status field from the 24-bit raw SPI payload
  uint8_t status = (raw >> STATUS_SHIFT) & STATUS_MASK;

  // Check if either the weak or strong magnetic field warning flags are set by
  // the hardware
  bool mag_fault = !!(status & (MAG_WEAK_BIT | MAG_STRONG_BIT));

  // Update diagnostic counters for telemetry monitoring
  if (mag_fault)
    s_fast_mag_warn[sensor]++;
  else
    s_fast_ok[sensor]++;

  // Latch the fault state so subsequent reads or queries know the sensor is
  // reporting degraded performance
  s_error[sensor] = mag_fault;

  // Extract the 14-bit angle, convert it to a floating point value, and scale
  // it to 360 degrees (16384 ticks per revolution)
  s_last_deg[sensor] =
      (float)((raw >> ANGLE_SHIFT) & ANGLE_MASK) * (360.0f / 16384.0f);

  // Return the successfully decoded angle
  return s_last_deg[sensor];
}
