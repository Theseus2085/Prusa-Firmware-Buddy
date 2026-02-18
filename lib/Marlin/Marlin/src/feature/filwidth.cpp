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

#include <cmath>
#include <algorithm>

#include <atomic>
#include "i2c.hpp"
#include "../core/serial.h"

FilamentWidthSensor filwidth;

bool   FilamentWidthSensor::enabled         = true;
float  FilamentWidthSensor::nominal_mm      = DEFAULT_NOMINAL_FILAMENT_DIA;
float  FilamentWidthSensor::measured_mm     = DEFAULT_MEASURED_FILAMENT_DIA;
float  FilamentWidthSensor::nominal_area    = 0.0f;
float  FilamentWidthSensor::e_count         = 0.0f;
float  FilamentWidthSensor::delay_dist      = 0.0f;
uint8_t FilamentWidthSensor::meas_delay_cm  = MEASUREMENT_DELAY_CM;
float  FilamentWidthSensor::ratios[MMD_CM]  = { 0.0f };
int8_t FilamentWidthSensor::index_r         = 0;
int8_t FilamentWidthSensor::index_w         = 0;

float  FilamentWidthSensor::sensor_queue[sensor_count][FILWIDTH_SENSOR_QUEUE_CAPACITY]      = { { 0 } };
float  FilamentWidthSensor::sensor_targets_mm[sensor_count][FILWIDTH_SENSOR_QUEUE_CAPACITY] = { { 0 } };
uint8_t FilamentWidthSensor::sensor_head[sensor_count] = { 0 };
uint8_t FilamentWidthSensor::sensor_size[sensor_count] = { 0 };
float  FilamentWidthSensor::sensor_offsets_mm[sensor_count] = {
  FILWIDTH_SENSOR1_OFFSET_MM,
  FILWIDTH_SENSOR2_OFFSET_MM
};
float  FilamentWidthSensor::filament_position_mm = 0.0f;
float  FilamentWidthSensor::latest_axes_mm[sensor_count] = {
  DEFAULT_NOMINAL_FILAMENT_DIA,
  DEFAULT_NOMINAL_FILAMENT_DIA
};

namespace {
  constexpr uint8_t sensor_axes              = 2;
  constexpr uint8_t sensor_digits_per_axis   = 5;
  constexpr uint8_t sensor_payload_bytes     = sensor_axes * sensor_digits_per_axis;
  constexpr uint8_t sensor_address           = FILWIDTH_SENSOR_I2C_ADDRESS;
  constexpr uint32_t sensor_i2c_timeout_ms   = 5;
  constexpr uint8_t sensor_ready_retries     = 3;
  constexpr millis_t sensor_poll_interval_ms = 250UL;
  constexpr millis_t sensor_warn_interval_ms = 3000UL;

  bool decode_axis_digits(const uint8_t *buffer, const uint8_t length, float &out_mm) {
    if (!buffer || length == 0) return false;

    uint8_t digits_collected = 0;
    uint8_t digits[sensor_digits_per_axis] = { 0 };

    for (uint8_t i = 0; i < length && digits_collected < sensor_digits_per_axis; ++i) {
      const uint8_t value = buffer[i];
      if (value == '\n' || value == '\r' || value == ' ') continue;
      if (value == '.') continue;

      uint8_t digit;
      if (value <= 9) digit = value;
      else if (value >= '0' && value <= '9') digit = value - '0';
      else return false;

      digits[digits_collected++] = digit;
    }

    if (digits_collected != sensor_digits_per_axis)
      return false;

    float result = digits[0];
    float scale = 0.1f;
    for (uint8_t i = 1; i < sensor_digits_per_axis; ++i, scale *= 0.1f)
      result += digits[i] * scale;

    if (!WITHIN(result, 0.5f, 3.5f))
      return false;

    out_mm = result;
    return true;
  }

  bool decode_sensor_payload(const uint8_t *buffer, const uint8_t length, float (&axes_mm)[sensor_axes]) {
    if (!buffer || length < sensor_payload_bytes) return false;
    for (uint8_t axis = 0; axis < sensor_axes; ++axis) {
      const uint8_t *axis_ptr = buffer + (axis * sensor_digits_per_axis);
      if (!decode_axis_digits(axis_ptr, sensor_digits_per_axis, axes_mm[axis]))
        return false;
    }
    return true;
  }

  std::atomic<bool> filament_update_pending { false };
  millis_t next_poll_ms = 0;
  millis_t next_warn_ms = 0;

  bool should_warn(const millis_t now) {
    if (!ELAPSED(now, next_warn_ms)) return false;
    next_warn_ms = now + sensor_warn_interval_ms;
    return true;
  }

  bool fail_and_reschedule(const millis_t now, const char *msg, const int code) {
    next_poll_ms = now + sensor_poll_interval_ms;
    if (should_warn(now)) {
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM(" FilWidth I2C: ");
      SERIAL_ECHO(msg);
      if (code >= 0) {
        SERIAL_ECHOPGM("=");
        SERIAL_ECHO(code);
      }
      SERIAL_EOL();
    }
    return false;
  }

  i2c::Result wait_device_ready(I2C_HandleTypeDef &i2c_handle) {
    i2c::Result ready_result = i2c::Result::error;
    for (uint8_t attempt = 0; attempt < sensor_ready_retries; ++attempt) {
      ready_result = i2c::IsDeviceReady(i2c_handle, uint16_t(sensor_address << 1), 1, sensor_i2c_timeout_ms);
      if (ready_result == i2c::Result::ok) break;
    }
    return ready_result;
  }
}

void FilamentWidthSensor::init() {
  refresh_nominal_area();
  measured_mm = nominal_mm;
  e_count = 0.0f;
  delay_dist = 0.0f;
  index_r = index_w = 0;

  const float ratio = sample_to_size_ratio();
  for (uint8_t i = 0; i < COUNT(ratios); ++i) ratios[i] = ratio;

  filament_position_mm = 0.0f;
  for (uint8_t axis = 0; axis < sensor_count; ++axis) {
    sensor_head[axis] = 0;
    sensor_size[axis] = 0;
    latest_axes_mm[axis] = nominal_mm;
    for (uint8_t slot = 0; slot < FILWIDTH_SENSOR_QUEUE_CAPACITY; ++slot) {
      sensor_queue[axis][slot] = nominal_mm;
      sensor_targets_mm[axis][slot] = 0.0f;
    }
  }

  reset_poll_state();
}

void FilamentWidthSensor::reset_poll_state() {
  next_poll_ms = 0;
  next_warn_ms = 0;
}

float FilamentWidthSensor::sample_to_size_ratio() {
  const float axis_a = std::max(latest_axes_mm[0], 0.01f);
  const float axis_b = std::max(latest_axes_mm[1], 0.01f);
  const float area = get_area_mm2(axis_a, axis_b);
  if (nominal_area <= 0.0f) return 0.0f;
  return (area / nominal_area) * 100.0f - 100.0f;
}

void FilamentWidthSensor::schedule_update() {
  filament_update_pending.store(true, std::memory_order_release);
}

void FilamentWidthSensor::service_update() {
  if (!filament_update_pending.exchange(false, std::memory_order_acq_rel) || !enabled) return;

  const millis_t now = millis();
  if (PENDING(now, next_poll_ms)) return;

  (void)update_from_sensor();
}

bool FilamentWidthSensor::update_from_sensor() {
  const millis_t now = millis();
  if (PENDING(now, next_poll_ms)) return false;

  I2C_HandleTypeDef &i2c_handle = I2C_HANDLE_FOR(io_expander2);
  const auto ready_result = wait_device_ready(i2c_handle);
  if (ready_result != i2c::Result::ok)
    return fail_and_reschedule(now, "device_not_ready", int(ready_result));

  uint8_t buffer[sensor_payload_bytes] = { 0 };
  const auto result = i2c::Receive(
    i2c_handle,
    uint16_t((sensor_address << 1) | 0x1),
    buffer,
    sensor_payload_bytes,
    sensor_i2c_timeout_ms
  );

  if (result != i2c::Result::ok)
    return fail_and_reschedule(now, "read_failed", int(result));

  float axes_mm[sensor_axes] = { latest_axes_mm[0], latest_axes_mm[1] };
  if (!decode_sensor_payload(buffer, sensor_payload_bytes, axes_mm))
    return fail_and_reschedule(now, "decode_failed", -1);

  enqueue_sensor_sample(0, axes_mm[0]);
  enqueue_sensor_sample(1, axes_mm[1]);
  process_ready_samples();

  next_poll_ms = now + sensor_poll_interval_ms;
  return true;
}

void FilamentWidthSensor::enqueue_sensor_sample(const uint8_t sensor_index, const float diameter_mm) {
  if (sensor_index >= sensor_count) return;

  const uint8_t capacity = FILWIDTH_SENSOR_QUEUE_CAPACITY;
  const uint8_t tail = (sensor_head[sensor_index] + sensor_size[sensor_index]) % capacity;

  sensor_queue[sensor_index][tail] = diameter_mm;
  sensor_targets_mm[sensor_index][tail] = filament_position_mm + sensor_offsets_mm[sensor_index];

  if (sensor_size[sensor_index] < capacity) {
    ++sensor_size[sensor_index];
  }
  else {
    sensor_head[sensor_index] = (sensor_head[sensor_index] + 1) % capacity;
  }
}

bool FilamentWidthSensor::try_pop_aligned_sample(float &axis_a_mm, float &axis_b_mm) {
  if (!sensor_size[0] || !sensor_size[1]) return false;

  const uint8_t head_a = sensor_head[0];
  const uint8_t head_b = sensor_head[1];

  if (filament_position_mm < sensor_targets_mm[0][head_a]) return false;
  if (filament_position_mm < sensor_targets_mm[1][head_b]) return false;

  axis_a_mm = sensor_queue[0][head_a];
  axis_b_mm = sensor_queue[1][head_b];

  sensor_head[0] = (head_a + 1) % FILWIDTH_SENSOR_QUEUE_CAPACITY;
  sensor_head[1] = (head_b + 1) % FILWIDTH_SENSOR_QUEUE_CAPACITY;
  --sensor_size[0];
  --sensor_size[1];
  return true;
}

void FilamentWidthSensor::process_ready_samples() {
  float axis_a = latest_axes_mm[0];
  float axis_b = latest_axes_mm[1];
  while (try_pop_aligned_sample(axis_a, axis_b)) {
    latest_axes_mm[0] = axis_a;
    latest_axes_mm[1] = axis_b;
    measured_mm = compute_equivalent_diameter(axis_a, axis_b);
  }
}

float FilamentWidthSensor::compute_equivalent_diameter(const float axis_a_mm, const float axis_b_mm) {
  const float safe_product = std::max(axis_a_mm * axis_b_mm, 0.0001f);
  return sqrtf(safe_product);
}

void FilamentWidthSensor::advance_e(const float &e_move) {
#if ENABLED(FILWIDTH_SENSOR_USE_I2C)
  filament_position_mm += e_move;
  process_ready_samples();
#endif

  e_count += e_move;
  delay_dist += e_move;

  if (!UNEAR_ZERO(e_count)) {
    while (delay_dist >= MMD_MM) delay_dist -= MMD_MM;

    index_r = int8_t(delay_dist * 0.1f);

    if (index_r != index_w) {
      e_count = 0;
      const float meas_sample = sample_to_size_ratio();
      do {
        if (++index_w >= MMD_CM) index_w = 0;
        ratios[index_w] = meas_sample;
      } while (index_r != index_w);
    }
  }
}

#endif // FILAMENT_WIDTH_SENSOR
