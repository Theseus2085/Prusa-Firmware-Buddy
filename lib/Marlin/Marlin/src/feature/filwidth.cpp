/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(FILAMENT_WIDTH_SENSOR)

#include "filwidth.h"

#if ENABLED(FILWIDTH_SENSOR_USE_I2C)
  #include <atomic>
  #include "i2c.hpp"

  namespace {

    constexpr uint8_t sensor_address = FILWIDTH_SENSOR_I2C_ADDRESS;
    // Match io-expander style transactions on this shared optional-device path.
    constexpr uint32_t sensor_i2c_timeout_ms = 5;
    constexpr uint8_t scd41_init_retries = 3;

    // Poll cadence for data-ready checks; SCD41 updates every ~5s, but we probe faster.
    constexpr millis_t scd41_poll_interval_ms = 1000UL;
    // Throttle serial output to avoid flooding.
    constexpr millis_t scd41_print_interval_ms = 3000UL;
    // Throttle warning logs to keep UART readable.
    constexpr millis_t scd41_warn_interval_ms = 3000UL;
    // First read needs a full measurement period after start command.
    constexpr millis_t scd41_first_sample_delay_ms = 5000UL;

    // SCD41 command words (MSB-first).
    constexpr uint16_t scd41_cmd_start_periodic = 0x21B1;
    constexpr uint16_t scd41_cmd_get_data_ready = 0xE4B8;
    constexpr uint16_t scd41_cmd_read_measurement = 0xEC05;

    std::atomic<bool> filament_update_pending { false };
    // Tracks whether periodic measurement mode is armed on the sensor.
    bool scd41_started = false;
    // Schedule points for polling, warnings, and serial output.
    millis_t next_poll_ms = 0;
    millis_t next_warn_ms = 0;
    millis_t last_print_ms = 0;

    // Sensirion CRC8: polynomial 0x31, init 0xFF, 8-bit width.
    uint8_t sensirion_crc8(const uint8_t *data, const uint8_t len) {
      uint8_t crc = 0xFF;
      for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b)
          crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x31) : uint8_t(crc << 1);
      }
      return crc;
    }

    bool scd41_should_warn(const millis_t now) {
      if (!ELAPSED(now, next_warn_ms)) return false;
      next_warn_ms = now + scd41_warn_interval_ms;
      return true;
    }

    i2c::Result scd41_send_command(I2C_HandleTypeDef &hi2c, const uint16_t command) {
      uint8_t cmd[2] = { uint8_t(command >> 8), uint8_t(command & 0xFF) };
      return i2c::Transmit(hi2c, uint16_t(sensor_address << 1), cmd, sizeof(cmd), sensor_i2c_timeout_ms);
    }

    // Reset state on any I2C/CRC error to force a clean re-sync next cycle.
    // This prevents stale state if the device was hot-plugged or wedged.
    bool scd41_fail_and_retry(const millis_t now, const char *msg, const int code) {
      scd41_started = false;
      next_poll_ms = now + scd41_poll_interval_ms;

      if (scd41_should_warn(now)) {
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM(" SCD41: ");
        SERIAL_ECHO(msg);
        if (code >= 0) {
          SERIAL_ECHOPGM("=");
          SERIAL_ECHO(code);
        }
        SERIAL_EOL();
      }
      return false;
    }

  } // namespace

#endif

FilamentWidthSensor filwidth;
bool FilamentWidthSensor::enabled = true; // (M405-M406) Filament Width Sensor ON/OFF.
float FilamentWidthSensor::nominal_mm = DEFAULT_NOMINAL_FILAMENT_DIA,   // (M104) Nominal filament width
      FilamentWidthSensor::measured_mm = DEFAULT_MEASURED_FILAMENT_DIA, // Measured filament diameter
      FilamentWidthSensor::e_count = 0,
      FilamentWidthSensor::delay_dist = 0;
uint8_t FilamentWidthSensor::meas_delay_cm = MEASUREMENT_DELAY_CM;      // Distance delay setting
int8_t FilamentWidthSensor::ratios[MAX_MEASUREMENT_DELAY + 1],          // Ring buffer to delay measurement. (Extruder factor minus 100)
       FilamentWidthSensor::index_r,                                    // Indexes into ring buffer
       FilamentWidthSensor::index_w;

#if DISABLED(FILWIDTH_SENSOR_USE_I2C)
uint32_t FilamentWidthSensor::accum; // = 0                             // ADC accumulator
uint16_t FilamentWidthSensor::raw; // = 0                               // Measured filament diameter - one extruder only
#endif

void FilamentWidthSensor::init() {
  const int8_t ratio = sample_to_size_ratio();
  for (uint8_t i = 0; i < COUNT(ratios); ++i) ratios[i] = ratio;
  index_r = index_w = 0;
}

#if ENABLED(FILWIDTH_SENSOR_USE_I2C)

void FilamentWidthSensor::schedule_update() {
  filament_update_pending.store(true, std::memory_order_relaxed);
}

void FilamentWidthSensor::service_update() {
  if (!filament_update_pending.exchange(false, std::memory_order_acq_rel) || !enabled) return;

  update_from_sensor();
}

void FilamentWidthSensor::get_and_publish_scd41() {
  (void)update_from_sensor();
}

bool FilamentWidthSensor::update_from_sensor() {
  I2C_HandleTypeDef &hi2c = I2C_HANDLE_FOR(io_expander2);
  const millis_t now = millis();

  if (!ELAPSED(now, next_poll_ms))
    return false;

  if (!scd41_started) {
    i2c::Result init_result = i2c::Result::error;
    bool device_ack_seen = false;

    // Retry a few times to absorb transient bus arbitration failures on shared I2C2.
    for (uint8_t attempt = 0; attempt < scd41_init_retries; ++attempt) {
      init_result = i2c::IsDeviceReady(hi2c, uint16_t(sensor_address << 1), 1, sensor_i2c_timeout_ms);
      if (init_result != i2c::Result::ok) continue;

      device_ack_seen = true;
      init_result = scd41_send_command(hi2c, scd41_cmd_start_periodic);
      if (init_result == i2c::Result::ok) {
        scd41_started = true;
        next_poll_ms = now + scd41_first_sample_delay_ms;
        return false;
      }
    }

    next_poll_ms = now + scd41_poll_interval_ms;
    if (scd41_should_warn(now)) {
      SERIAL_ECHO_START();
      if (device_ack_seen)
        SERIAL_ECHOLNPAIR(" SCD41: start_periodic_failed=", int(init_result));
      else
        SERIAL_ECHOLNPAIR(" SCD41: device_not_ready=", int(init_result));
    }
    return false;
  }

  // Command sequence: GetDataReadyStatus -> read 2B + CRC -> ReadMeasurement.
  const auto ready_cmd_result = scd41_send_command(hi2c, scd41_cmd_get_data_ready);
  if (ready_cmd_result != i2c::Result::ok)
    return scd41_fail_and_retry(now, "data_ready_cmd_failed", int(ready_cmd_result));

  uint8_t ready_buf[3] = { 0 };
  const auto ready_read_result = i2c::Receive(hi2c, uint16_t((sensor_address << 1) | 0x1), ready_buf, sizeof(ready_buf), sensor_i2c_timeout_ms);
  if (ready_read_result != i2c::Result::ok)
    return scd41_fail_and_retry(now, "data_ready_read_failed", int(ready_read_result));

  // Data-ready response is a single 16-bit word plus CRC.
  if (sensirion_crc8(ready_buf, 2) != ready_buf[2])
    return scd41_fail_and_retry(now, "data_ready_crc_mismatch", -1);

  const uint16_t ready_word = (uint16_t(ready_buf[0]) << 8) | uint16_t(ready_buf[1]);
  if ((ready_word & 0x07FFU) == 0) {
    next_poll_ms = now + scd41_poll_interval_ms;
    return false;
  }

  const auto read_cmd_result = scd41_send_command(hi2c, scd41_cmd_read_measurement);
  if (read_cmd_result != i2c::Result::ok)
    return scd41_fail_and_retry(now, "read_cmd_failed", int(read_cmd_result));

  uint8_t buf[9] = { 0 };
  const auto read_result = i2c::Receive(hi2c, uint16_t((sensor_address << 1) | 0x1), buf, sizeof(buf), sensor_i2c_timeout_ms);
  if (read_result != i2c::Result::ok)
    return scd41_fail_and_retry(now, "read_failed", int(read_result));

  // Each 16-bit word (CO2, temp, RH) is followed by its own CRC byte.
  for (uint8_t i = 0; i < 3; ++i) {
    const uint8_t *word = &buf[i * 3];
    if (sensirion_crc8(word, 2) != word[2])
      return scd41_fail_and_retry(now, "measurement_crc_mismatch", -1);
  }

  const uint16_t co2_ppm = (uint16_t(buf[0]) << 8) | uint16_t(buf[1]);
  const uint16_t raw_t = (uint16_t(buf[3]) << 8) | uint16_t(buf[4]);
  const uint16_t raw_rh = (uint16_t(buf[6]) << 8) | uint16_t(buf[7]);

  const float temp_c = -45.0f + 175.0f * (float(raw_t) / 65536.0f);
  const float rh_pct = 100.0f * (float(raw_rh) / 65536.0f);

  if (ELAPSED(now, last_print_ms + scd41_print_interval_ms)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(" SCD41 co2_ppm=");
    SERIAL_ECHO(int(co2_ppm));
    SERIAL_ECHOPGM(" temp_c=");
    SERIAL_ECHO(temp_c);
    SERIAL_ECHOPGM(" rh_pct=");
    SERIAL_ECHOLN(rh_pct);
    last_print_ms = now;
  }

  next_poll_ms = now + scd41_poll_interval_ms;
  return true;
}

#endif

#endif // FILAMENT_WIDTH_SENSOR
