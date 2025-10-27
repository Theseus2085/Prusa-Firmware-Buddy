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

  constexpr uint8_t sensor_address = 0x62; // SCD41 I2C 7-bit address
    constexpr uint8_t sensor_digits = FILWIDTH_SENSOR_DIGITS;
    constexpr uint32_t sensor_timeout_ms = FILWIDTH_SENSOR_TIMEOUT_MS;

    bool decode_sensor_digits(const uint8_t *buffer, const uint8_t length, float &out_mm) {
      if (!buffer) return false;

      uint8_t digits_collected = 0;
      uint8_t digits[sensor_digits] = { 0 };

      for (uint8_t i = 0; i < length && digits_collected < sensor_digits; ++i) {
        const uint8_t value = buffer[i];

        if (value == '\n' || value == '\r' || value == ' ') continue;
        if (value == '.') continue;

        uint8_t digit;
        if (value <= 9) {
          digit = value;
        }
        else if (value >= '0' && value <= '9') {
          digit = value - '0';
        }
        else {
          return false;
        }

        digits[digits_collected++] = digit;
      }

      if (digits_collected != sensor_digits)
        return false;

      float result = digits[0];
      float scale = 0.1f;
      for (uint8_t i = 1; i < sensor_digits; ++i, scale *= 0.1f)
        result += digits[i] * scale;

      if (!WITHIN(result, 0.5f, 3.5f))
        return false;

      out_mm = result;
      return true;
    }

  } // namespace

  namespace {
    std::atomic<bool> filament_update_pending { false };
  }

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
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth schedule_update");
  filament_update_pending.store(true, std::memory_order_relaxed);
}

void FilamentWidthSensor::service_update() {
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth service_update");
  if (!filament_update_pending.exchange(false, std::memory_order_acq_rel)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(" filwidth service_update skip-no-pending");
    return;
  }

  if (!enabled) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(" filwidth service_update skip-disabled");
    return;
  }

  update_from_sensor();
}

void FilamentWidthSensor::get_and_publish_scd41() {
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth get_and_publish_scd41");

  I2C_HandleTypeDef &hi2c = I2C_HANDLE_FOR(io_expander2);

  const auto ready = i2c::IsDeviceReady(hi2c, uint16_t(sensor_address << 1), 1, sensor_timeout_ms);
  if (ready != i2c::Result::ok) {
    SERIAL_ECHO_START(); SERIAL_ECHOLNPAIR(" SCD41: not ready=", int(ready));
    return;
  }

  // helper: Sensirion CRC8 (polynomial 0x31 init 0xFF)
  auto sensirion_crc8 = [](const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (uint8_t b = 0; b < 8; ++b) {
        crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x31) : uint8_t(crc << 1);
      }
    }
    return crc;
  };

  // Request a measurement read (SCD4x read command 0xEC05)
  uint8_t cmd[2] = { 0xEC, 0x05 };
  if (i2c::Transmit(hi2c, uint16_t(sensor_address << 1), cmd, 2, sensor_timeout_ms) != i2c::Result::ok) {
    SERIAL_ECHO_START(); SERIAL_ECHOLNPGM(" SCD41: failed to send read command");
    return;
  }

  // Read 9 bytes: CO2(2)+CRC, T(2)+CRC, RH(2)+CRC
  uint8_t buf[9] = {0};
  if (i2c::Receive(hi2c, uint16_t((sensor_address << 1) | 0x1), buf, sizeof(buf), sensor_timeout_ms) != i2c::Result::ok) {
    SERIAL_ECHO_START(); SERIAL_ECHOLNPGM(" SCD41: read failed (i2c receive)");
    return;
  }

  // Validate CRCs
  for (int i = 0; i < 3; ++i) {
    const uint8_t *w = &buf[i * 3];
    if (sensirion_crc8(w, 2) != w[2]) {
      SERIAL_ECHO_START(); SERIAL_ECHOLNPGM(" SCD41: CRC mismatch");
      return;
    }
  }

  uint16_t co2 = (uint16_t(buf[0]) << 8) | uint16_t(buf[1]);
  uint16_t raw_t = (uint16_t(buf[3]) << 8) | uint16_t(buf[4]);
  uint16_t raw_rh = (uint16_t(buf[6]) << 8) | uint16_t(buf[7]);

  float temp_c = -45.0f + 175.0f * (float(raw_t) / 65536.0f);
  float rh = 100.0f * (float(raw_rh) / 65536.0f);

  SERIAL_ECHO_START(); SERIAL_ECHOLNPGM(" SCD41 measurement:");
  SERIAL_ECHO_START(); SERIAL_ECHOLNPAIR("  co2_ppm=", int(co2));
  SERIAL_ECHO_START(); SERIAL_ECHOLNPAIR("  temp_c=", temp_c);
  SERIAL_ECHO_START(); SERIAL_ECHOLNPAIR("  rh_pct=", rh);
}

bool FilamentWidthSensor::update_from_sensor() {
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth update_from_sensor");
  uint8_t buffer[sensor_digits] = { 0 };

  const auto ready = i2c::IsDeviceReady(hi2c2, sensor_address << 1, 1, sensor_timeout_ms);
  if (ready != i2c::Result::ok) {
    SERIAL_ECHO_START();
    switch (ready) {
      case i2c::Result::error:
        SERIAL_ECHOLNPAIR(" filwidth device_not_ready=errorinecho", int(ready));
        break;
      case i2c::Result::busy_after_retries:
        SERIAL_ECHOLNPGM(" filwidth device_not_ready=busy");
        break;
      case i2c::Result::timeout:
        SERIAL_ECHOLNPGM(" filwidth device_not_ready=timeout");
        break;
      default:
        SERIAL_ECHOLNPAIR(" filwidth device_not_ready=realerror", int(ready));
        break;
    }
    return false;
  }

  const auto result = i2c::Receive(hi2c2, (sensor_address << 1) | 0x1, buffer, sensor_digits, sensor_timeout_ms);
  if (result != i2c::Result::ok) {
    SERIAL_ECHO_START();
    switch (result) {
      case i2c::Result::error:
        SERIAL_ECHOLNPGM(" filwidth i2c failure=errorrrrrr");
        break;
      case i2c::Result::busy_after_retries:
        SERIAL_ECHOLNPGM(" filwidth i2c failure=busy");
        break;
      case i2c::Result::timeout:
        SERIAL_ECHOLNPGM(" filwidth i2c failure=timeout");
        break;
      default:
        SERIAL_ECHOLNPAIR(" filwidth i2c failure=", int(result));
        break;
    }
    return false;
  }

  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth i2c ok");

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM(" filwidth raw:");
  for (uint8_t i = 0; i < sensor_digits; ++i) {
    SERIAL_ECHOPGM(" ");
    SERIAL_ECHO(int(buffer[i]));
  }
  SERIAL_ECHOPGM(" ascii:'");
  for (uint8_t i = 0; i < sensor_digits; ++i)
    SERIAL_CHAR((buffer[i] >= 32 && buffer[i] <= 126) ? buffer[i] : '.');
  SERIAL_CHAR('\'');
  SERIAL_EOL();

  float measured_value = measured_mm;
  if (!decode_sensor_digits(buffer, sensor_digits, measured_value)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(" filwidth decode failed");
    return false;
  }

  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth decode ok");
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPAIR(" filwidth decoded=", measured_value);
  measured_mm = measured_value;
  return true;
}

#endif

#endif // FILAMENT_WIDTH_SENSOR
