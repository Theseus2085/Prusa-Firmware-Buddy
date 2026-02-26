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
uint16_t FilamentWidthSensor::sensor_head[sensor_count] = { 0 };
uint16_t FilamentWidthSensor::sensor_size[sensor_count] = { 0 };
float  FilamentWidthSensor::sensor_offsets_mm[sensor_count] = {
  FILWIDTH_SENSOR1_OFFSET_MM,
  FILWIDTH_SENSOR2_OFFSET_MM
};
float  FilamentWidthSensor::filament_position_mm = 0.0f;
float  FilamentWidthSensor::retraction_debt_mm = 0.0f;
float  FilamentWidthSensor::next_enqueue_mm = 0.0f;
float  FilamentWidthSensor::latest_axes_mm[sensor_count] = {
  DEFAULT_NOMINAL_FILAMENT_DIA,
  DEFAULT_NOMINAL_FILAMENT_DIA
};

namespace {
  static_assert(FILWIDTH_SENSOR_DIGITS == 5, "Filwidth wire protocol expects 5 digits per axis.");
  static_assert(FILWIDTH_SENSOR_QUEUE_BIN_MM > 0, "FILWIDTH_SENSOR_QUEUE_BIN_MM must be positive.");
  static_assert(FILWIDTH_SENSOR_QUEUE_CAPACITY > 0 && FILWIDTH_SENSOR_QUEUE_CAPACITY <= 65535, "FILWIDTH_SENSOR_QUEUE_CAPACITY must fit queue index type.");

  constexpr uint8_t sensor_axes              = 2;
  constexpr uint8_t sensor_digits_per_axis   = FILWIDTH_SENSOR_DIGITS;
  constexpr uint8_t sensor_payload_bytes     = sensor_axes * sensor_digits_per_axis;
  constexpr uint8_t sensor_address           = FILWIDTH_SENSOR_I2C_ADDRESS;
  constexpr uint32_t sensor_i2c_timeout_ms   = FILWIDTH_SENSOR_TIMEOUT_MS;
  constexpr uint8_t sensor_ready_retries     = FILWIDTH_SENSOR_READY_RETRIES;
  constexpr millis_t sensor_poll_interval_ms = FILWIDTH_SENSOR_POLL_INTERVAL_MS;
  constexpr millis_t sensor_warn_interval_ms = FILWIDTH_SENSOR_WARN_INTERVAL_MS;
  constexpr millis_t sensor_rx_print_interval_ms = FILWIDTH_SENSOR_RX_PRINT_INTERVAL_MS;
  constexpr float sensor_fixed_scale_inv = 0.0001f;
  constexpr uint32_t sensor_min_valid_x10000 = 5000U;
  constexpr uint32_t sensor_max_valid_x10000 = 35000U;
  constexpr float sensor_queue_bin_mm = float(FILWIDTH_SENSOR_QUEUE_BIN_MM);

  /**
   * @brief Parses a fixed-length numeric string buffer into an integer representation.
   * 
   * Reads raw bytes (usually ASCII characters) ignoring whitespace and decimals.
   * Converts the resulting digit sequence into an unsigned integer scaled by 10,000.
   * 
   * @param buffer Pointer to the raw bytes.
   * @param length The expected number of digits to parse.
   * @param out_x10000 Reference to store the decoded and scaled integer.
   * @return true if successfully decoded within valid operational bounds.
   */
  bool decode_axis_digits(const uint8_t *buffer, const uint8_t length, uint32_t &out_x10000) {
    if (!buffer || length == 0) return false;

    uint8_t digits_collected = 0;
    uint32_t value_x10000 = 0;

    for (uint8_t i = 0; i < length && digits_collected < sensor_digits_per_axis; ++i) {
      const uint8_t value = buffer[i];
      if (value == '\n' || value == '\r' || value == ' ') continue;
      if (value == '.') continue;

      uint8_t digit;
      if (value <= 9) digit = value;
      else if (value >= '0' && value <= '9') digit = value - '0';
      else return false;

      value_x10000 = value_x10000 * 10U + digit;
      ++digits_collected;
    }

    if (digits_collected != sensor_digits_per_axis)
      return false;

    if (!WITHIN(value_x10000, sensor_min_valid_x10000, sensor_max_valid_x10000))
      return false;

    out_x10000 = value_x10000;
    return true;
  }

  /**
   * @brief Decodes the complete multi-axis payload from the I2C sensor.
   * 
   * Slices the incoming payload byte array into per-axis sequences and 
   * delegates parsing to `decode_axis_digits`.
   * 
   * @param buffer Raw data buffer received from the sensor.
   * @param length Total length of the payload array.
   * @param axes_x10000 Output array for the parsed values per axis.
   * @return true if all axes were successfully decoded.
   */
  bool decode_sensor_payload(const uint8_t *buffer, const uint8_t length, uint32_t (&axes_x10000)[sensor_axes]) {
    if (!buffer || length < sensor_payload_bytes) return false;
    for (uint8_t axis = 0; axis < sensor_axes; ++axis) {
      const uint8_t *axis_ptr = buffer + (axis * sensor_digits_per_axis);
      if (!decode_axis_digits(axis_ptr, sensor_digits_per_axis, axes_x10000[axis]))
        return false;
    }
    return true;
  }

  std::atomic<bool> filament_update_pending { false };
  millis_t next_poll_ms = 0;
  millis_t next_warn_ms = 0;
  millis_t next_rx_log_ms = 0;

  bool should_warn(const millis_t now) {
    if (!ELAPSED(now, next_warn_ms)) return false;
    next_warn_ms = now + sensor_warn_interval_ms;
    return true;
  }

  bool should_log_rx(const millis_t now) {
    if (!ELAPSED(now, next_rx_log_ms)) return false;
    next_rx_log_ms = now + sensor_rx_print_interval_ms;
    return true;
  }

  /**
   * @brief Handles I2C communication failures by rescheduling the next poll.
   * 
   * Also enforces rate-limited diagnostic logging to Serial to avoid spamming the console
   * during temporary sensor disconnects or noise.
   * 
   * @param now Current system time in milliseconds.
   * @param msg The diagnostic error message.
   * @param code An associated numerical error code.
   * @return unconditionally returns false to cascade the failure state.
   */
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

  /**
   * @brief Blocks briefly to ensure the I2C peripheral sensor is ready for a transaction.
   * 
   * @param i2c_handle The hardware I2C interface handle.
   * @return The resulting I2C operational status.
   */
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
  retraction_debt_mm = 0.0f;
  filament_position_mm = 0.0f;
  next_enqueue_mm = 0.0f;

  for (uint8_t axis = 0; axis < sensor_count; ++axis) {
    latest_axes_mm[axis] = nominal_mm;
  }

  clear_alignment_queues();
  refresh_sensor_offsets();

  const float ratio = sample_to_size_ratio();
  for (uint8_t i = 0; i < COUNT(ratios); ++i) ratios[i] = ratio;

  reset_poll_state();
}

void FilamentWidthSensor::set_delay_cm(uint8_t cm) {
  meas_delay_cm = _MIN(cm, MAX_MEASUREMENT_DELAY);
#if ENABLED(FILWIDTH_SENSOR_USE_I2C)
  refresh_sensor_offsets();
  clear_alignment_queues();
  next_enqueue_mm = filament_position_mm;
#endif
}

void FilamentWidthSensor::set_nominal_mm(float mm) {
  nominal_mm = mm;
  refresh_nominal_area();
}

void FilamentWidthSensor::clear_alignment_queues() {
  for (uint8_t axis = 0; axis < sensor_count; ++axis) {
    sensor_head[axis] = 0;
    sensor_size[axis] = 0;
  }
}

void FilamentWidthSensor::refresh_sensor_offsets() {
  sensor_offsets_mm[0] = meas_delay_cm * 10.0f;
  sensor_offsets_mm[1] = sensor_offsets_mm[0] + FILWIDTH_SENSOR_SPACING_MM;
}

void FilamentWidthSensor::reset_poll_state() {
  next_poll_ms = 0;
  next_warn_ms = 0;
  next_rx_log_ms = 0;
}

float FilamentWidthSensor::sample_to_size_ratio() {
  const float axis_a = std::max(latest_axes_mm[0], 0.01f);
  const float axis_b = std::max(latest_axes_mm[1], 0.01f);
  const float area = get_area_mm2(axis_a, axis_b);
  if (nominal_area <= 0.0f) return 0.0f;
  return (nominal_area / area) * 100.0f - 100.0f;
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

/**
 * @brief Executes an I2C transaction to fetch and decode the latest sensor measurements.
 * 
 * Invoked synchronously from the main loop when `filament_update_pending` is set.
 * Reads the raw payload, decodes the dual-axis geometry, and feeds it into the 
 * spatial alignment queues.
 * 
 * @return true if the measurement cycle completed and queued successfully.
 */
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

  uint32_t axes_x10000[sensor_axes] = { 0, 0 };
  if (!decode_sensor_payload(buffer, sensor_payload_bytes, axes_x10000))
    return fail_and_reschedule(now, "decode_failed", -1);

  const float axis_a_mm = axes_x10000[0] * sensor_fixed_scale_inv;
  const float axis_b_mm = axes_x10000[1] * sensor_fixed_scale_inv;

  if (should_log_rx(now)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(" FilWidth RX axis_a_mm=");
    SERIAL_ECHO(axis_a_mm);
    SERIAL_ECHOPGM(" axis_b_mm=");
    SERIAL_ECHO(axis_b_mm);
    SERIAL_ECHOPGM(" eq_mm=");
    SERIAL_ECHOLN(FilamentWidthSensor::compute_equivalent_diameter(axis_a_mm, axis_b_mm));
  }

  if (filament_position_mm + 0.0001f >= next_enqueue_mm) {
    enqueue_sensor_sample(0, axis_a_mm);
    enqueue_sensor_sample(1, axis_b_mm);
    process_ready_samples();
    next_enqueue_mm = filament_position_mm + sensor_queue_bin_mm;
  }

  next_poll_ms = now + sensor_poll_interval_ms;
  return true;
}

/**
 * @brief Appends a new measurement to the specified axis's circular buffer.
 * 
 * Records the diameter and calculates the absolute `filament_position_mm` at which
 * this specific physical cross-section will arrive at the extruder nozzle 
 * (current position + sensor offset).
 * 
 * @param sensor_index The logical axis identifier (0..sensor_count-1).
 * @param diameter_mm The measured diameter.
 */
void FilamentWidthSensor::enqueue_sensor_sample(const uint8_t sensor_index, const float diameter_mm) {
  if (sensor_index >= sensor_count) return;

  constexpr uint16_t capacity = FILWIDTH_SENSOR_QUEUE_CAPACITY;
  const uint16_t tail = (sensor_head[sensor_index] + sensor_size[sensor_index]) % capacity;

  sensor_queue[sensor_index][tail] = diameter_mm;
  sensor_targets_mm[sensor_index][tail] = filament_position_mm + sensor_offsets_mm[sensor_index];

  if (sensor_size[sensor_index] < capacity) {
    ++sensor_size[sensor_index];
  }
  else {
    sensor_head[sensor_index] = (sensor_head[sensor_index] + 1) % capacity;
  }
}

/**
 * @brief Verifies if the extruder has advanced enough to pop a synchronized sample pair.
 * 
 * A sample is only "ready" when the absolute `filament_position_mm` tracked by the 
 * stepper motors equals or exceeds the spatial target recorded during enqueueing.
 * Both axes must be ready to yield a valid elliptical cross-section.
 * 
 * @param axis_a_mm Output parameter for axis A's diameter.
 * @param axis_b_mm Output parameter for axis B's diameter.
 * @return true if a valid synchronized pair was dequeued.
 */
bool FilamentWidthSensor::try_pop_aligned_sample(float &axis_a_mm, float &axis_b_mm) {
  if (!sensor_size[0] || !sensor_size[1]) return false;

  const uint16_t head_a = sensor_head[0];
  const uint16_t head_b = sensor_head[1];

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

/**
 * @brief Dequeues all ready samples and computes the final effective diameter.
 * 
 * Loops over `try_pop_aligned_sample` until the queue is either empty or the next 
 * sample hasn't reached the nozzle yet. Updates the global `measured_mm`.
 */
void FilamentWidthSensor::process_ready_samples() {
  float axis_a = latest_axes_mm[0];
  float axis_b = latest_axes_mm[1];
  while (try_pop_aligned_sample(axis_a, axis_b)) {
    latest_axes_mm[0] = axis_a;
    latest_axes_mm[1] = axis_b;
    measured_mm = compute_equivalent_diameter(axis_a, axis_b);
  }
}

/**
 * @brief Transforms the dual-axis measurement into a circular equivalent diameter.
 * 
 * @param axis_a_mm The major/minor axis A.
 * @param axis_b_mm The major/minor axis B.
 * @return The ideal circular diameter possessing the identical cross-sectional area.
 */
float FilamentWidthSensor::compute_equivalent_diameter(const float axis_a_mm, const float axis_b_mm) {
  const float safe_product = std::max(axis_a_mm * axis_b_mm, 0.0001f);
  return sqrtf(safe_product);
}

/**
 * @brief Tracks physical extruder movement to synchronize sensor data with the nozzle.
 * 
 * Called continuously by the planner/stepper ISR whenever E-axis steps occur.
 * Handles retraction logic by accumulating "debt" on reverse moves, ensuring 
 * sensor data is only popped when new, unmeasured filament finally advances 
 * past the previous peak extrusion distance.
 * 
 * @param e_move The requested or actual physical movement of the E-axis in mm.
 */
void FilamentWidthSensor::advance_e(const float &e_move) {
  float correction_e_move = e_move;
#if ENABLED(FILWIDTH_SENSOR_USE_I2C)
  float forward_e_move = 0.0f;
  
  // Track retractions to prevent popping samples for filament that was pulled back
  if (e_move < 0.0f) {
    retraction_debt_mm += -e_move;
  }
  else if (e_move > 0.0f) {
    // Pay off retraction debt before advancing the absolute extrusion tracker
    const float debt_paid = _MIN(retraction_debt_mm, e_move);
    retraction_debt_mm -= debt_paid;
    forward_e_move = e_move - debt_paid;
  }

  correction_e_move = forward_e_move;
  
  // Only advance the global filament tracker and process new samples if we moved forward
  if (forward_e_move > 0.0f) {
    filament_position_mm += forward_e_move;
    process_ready_samples();
  }
#endif

  // Update analog delay trackers
  e_count += correction_e_move;
  delay_dist += correction_e_move;

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

float FilamentWidthSensor::get_averaged_size_ratio(float extrude_mm) {
  if (!enabled || nominal_area <= 0.0f) return 0.0f;

#if ENABLED(FILWIDTH_SENSOR_USE_I2C)
  /*
   * Performs a lookahead simulation over the synchronized `sensor_queue`.
   * Calculates the length-weighted cross-sectional area of the filament
   * over the specified extrusion distance (extrude_mm) without advancing 
   * the persistent read pointers.
   */
  if (extrude_mm <= 0.0f) return sample_to_size_ratio();

  float simulated_position_mm = filament_position_mm;
  const float end_position_mm = filament_position_mm + extrude_mm;

  uint16_t sim_head_a = sensor_head[0];
  uint16_t sim_head_b = sensor_head[1];
  uint16_t sim_size_a = sensor_size[0];
  uint16_t sim_size_b = sensor_size[1];

  float sim_latest_a = latest_axes_mm[0];
  float sim_latest_b = latest_axes_mm[1];

  float total_area_times_length = 0.0f;

  while (simulated_position_mm < end_position_mm) {
    bool can_pop = (sim_size_a > 0 && sim_size_b > 0);
    float target_a = can_pop ? sensor_targets_mm[0][sim_head_a] : end_position_mm + 1.0f;
    float target_b = can_pop ? sensor_targets_mm[1][sim_head_b] : end_position_mm + 1.0f;

    // Determine the next simulation boundary (either a queued measurement or the target end position)
    float next_event_mm = end_position_mm;
    if (can_pop) {
      next_event_mm = std::min(next_event_mm, std::max(target_a, target_b));
    }

    const float current_area = get_area_mm2(std::max(sim_latest_a, 0.01f), std::max(sim_latest_b, 0.01f));
    float step_length = next_event_mm - simulated_position_mm;
    
    // Prevent numerical instability leading to an infinite layout loop
    if (step_length <= 0.0f) step_length = 0.0001f;

    total_area_times_length += current_area * step_length;
    simulated_position_mm += step_length;

    // Advance simulated read pointers if we crossed a queued sample target
    if (can_pop && simulated_position_mm >= std::max(target_a, target_b)) {
      sim_latest_a = sensor_queue[0][sim_head_a];
      sim_latest_b = sensor_queue[1][sim_head_b];
      sim_head_a = (sim_head_a + 1) % FILWIDTH_SENSOR_QUEUE_CAPACITY;
      sim_head_b = (sim_head_b + 1) % FILWIDTH_SENSOR_QUEUE_CAPACITY;
      sim_size_a--;
      sim_size_b--;
    }
  }

  // Aggregate area / extrude_mm returns the average area, from which the size ratio is derived.
  const float average_area = total_area_times_length / extrude_mm;
  return (nominal_area / average_area) * 100.0f - 100.0f;
#else
  // Fallback for analog sensors: return the current ratio at index_r - meas_delay_cm
  int8_t read_index = index_r;
  read_index -= meas_delay_cm;
  if (read_index < 0) read_index += MMD_CM;
  LIMIT(read_index, 0, MAX_MEASUREMENT_DELAY);
  return ratios[read_index];
#endif
}

#endif // FILAMENT_WIDTH_SENSOR
