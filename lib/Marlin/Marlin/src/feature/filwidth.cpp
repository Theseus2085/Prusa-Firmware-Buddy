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
#include <cstring>

#include <atomic>
#include "i2c.hpp"
#include "../core/serial.h"

FilamentWidthSensor filwidth;

bool   FilamentWidthSensor::enabled         = true;
bool   FilamentWidthSensor::correction_enabled = true;
float  FilamentWidthSensor::nominal_mm      = DEFAULT_NOMINAL_FILAMENT_DIA;
float  FilamentWidthSensor::measured_mm     = DEFAULT_MEASURED_FILAMENT_DIA;
float  FilamentWidthSensor::nominal_area    = 0.0f;
uint8_t FilamentWidthSensor::meas_delay_cm  = MEASUREMENT_DELAY_CM;

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
float  FilamentWidthSensor::last_average_area = 0.0f;

uint16_t FilamentWidthSensor::log_count = 0;
uint16_t FilamentWidthSensor::log_dropped = 0;
bool FilamentWidthSensor::logging = false;
FilLogEntry FilamentWidthSensor::log_array[FilamentWidthSensor::log_capacity] = { {0} };

uint16_t FilamentWidthSensor::flush_log(const uint16_t max_entries) {
  if (!log_count || !max_entries) return 0;

  const uint16_t n = _MIN(max_entries, log_count);
  for (uint16_t i = 0; i < n; ++i) {
    const FilLogEntry &entry = log_array[i];
    SERIAL_ECHOPGM("FWLOG,");
    SERIAL_ECHO(entry.time_ms);
    SERIAL_ECHOPAIR(",", entry.x);
    SERIAL_ECHOPAIR(",", entry.y);
    SERIAL_ECHOPAIR(",", entry.e_factor);
    SERIAL_ECHOPAIR(",", entry.msteps_raw);
    SERIAL_ECHOPAIR(",", entry.msteps_corr);
    SERIAL_ECHOPAIR(",", entry.axis_a_mm);
    SERIAL_ECHOPAIR(",", entry.axis_b_mm);
    SERIAL_ECHOPAIR(",", entry.ellipse_area);
    SERIAL_EOL();
  }

  if (log_count > n) {
    memmove(log_array, log_array + n, (log_count - n) * sizeof(FilLogEntry));
  }
  log_count -= n;
  return n;
}

namespace {
  static_assert(FILWIDTH_SENSOR_DIGITS == 5, "Filwidth wire protocol expects 5 digits per axis.");
  static_assert(FILWIDTH_SENSOR_QUEUE_BIN_MM > 0, "FILWIDTH_SENSOR_QUEUE_BIN_MM must be positive.");
  static_assert(FILWIDTH_SENSOR_QUEUE_CAPACITY > 0 && FILWIDTH_SENSOR_QUEUE_CAPACITY <= 65535, "FILWIDTH_SENSOR_QUEUE_CAPACITY must fit queue index type.");
  static_assert(FILWIDTH_SENSOR_QUEUE_CAPACITY > (FILWIDTH_SENSOR_MAX_DELAY_MM / FILWIDTH_SENSOR_QUEUE_BIN_MM) + 4,
                "FILWIDTH_SENSOR_QUEUE_CAPACITY must exceed the axis-2 in-flight sample count (sensor offset + spacing, binned) plus margin.");

  constexpr uint8_t sensor_axes              = 2;
  constexpr uint8_t sensor_digits_per_axis   = FILWIDTH_SENSOR_DIGITS;
  constexpr uint8_t sensor_data_bytes        = sensor_axes * sensor_digits_per_axis;
  constexpr uint8_t sensor_payload_bytes     = sensor_data_bytes + 2; // + version byte + CRC8
  constexpr uint8_t sensor_version_byte      = FILWIDTH_SENSOR_PROTOCOL_VERSION;
  constexpr uint8_t sensor_address           = FILWIDTH_SENSOR_I2C_ADDRESS;
  constexpr uint32_t sensor_i2c_timeout_ms   = FILWIDTH_SENSOR_TIMEOUT_MS;
  constexpr uint8_t sensor_ready_retries     = FILWIDTH_SENSOR_READY_RETRIES;
  constexpr millis_t sensor_poll_interval_ms = FILWIDTH_SENSOR_POLL_INTERVAL_MS;
  constexpr millis_t sensor_warn_interval_ms = FILWIDTH_SENSOR_WARN_INTERVAL_MS;
  constexpr millis_t sensor_rx_print_interval_ms = FILWIDTH_SENSOR_RX_PRINT_INTERVAL_MS;
  constexpr millis_t sensor_recovery_interval_ms = FILWIDTH_SENSOR_RECOVERY_INTERVAL_MS;
  constexpr uint8_t sensor_max_consecutive_failures = FILWIDTH_SENSOR_MAX_CONSECUTIVE_FAILURES;
  constexpr float sensor_max_ratio_percent   = FILWIDTH_SENSOR_MAX_RATIO_PERCENT;
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
   * @brief Decodes the digit payload of the I2C sensor frame.
   *
   * Slices the data bytes into per-axis sequences and delegates parsing to
   * `decode_axis_digits`. Version byte and CRC are validated by the caller.
   *
   * @param buffer Raw data received from the sensor.
   * @param length Number of data bytes available (must cover all axes).
   * @param axes_x10000 Output array for the parsed values per axis.
   * @return true if all axes were successfully decoded.
   */
  bool decode_sensor_payload(const uint8_t *buffer, const uint8_t length, uint32_t (&axes_x10000)[sensor_axes]) {
    if (!buffer || length < sensor_data_bytes) return false;
    for (uint8_t axis = 0; axis < sensor_axes; ++axis) {
      const uint8_t *axis_ptr = buffer + (axis * sensor_digits_per_axis);
      if (!decode_axis_digits(axis_ptr, sensor_digits_per_axis, axes_x10000[axis]))
        return false;
    }
    return true;
  }

  /**
   * @brief CRC-8 (polynomial 0x07, init 0x00) over the frame bytes including the version byte.
   *
   * Must match the implementation in the sensor module firmware.
   *
   * @param buffer Frame bytes.
   * @param length Number of bytes covered by the checksum.
   * @return The CRC byte value.
   */
  uint8_t crc8_frame(const uint8_t *buffer, const uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; ++i) {
      crc ^= buffer[i];
      for (uint8_t bit = 0; bit < 8; ++bit)
        crc = (crc & 0x80U) ? uint8_t(uint8_t(crc << 1) ^ 0x07U) : uint8_t(crc << 1);
    }
    return crc;
  }

  /**
   * @brief Caps the applied correction so a bad measurement can never produce extreme extrusion.
   */
  float clamp_ratio_percent(const float ratio) {
    return std::min(std::max(ratio, -sensor_max_ratio_percent), sensor_max_ratio_percent);
  }

  std::atomic<bool> filament_update_pending { false };
  millis_t next_poll_ms = 0;
  millis_t next_warn_ms = 0;
  millis_t next_rx_log_ms = 0;
  uint8_t consecutive_comm_failures = 0;
  bool watchdog_tripped = false;
  bool watchdog_held_correction = false; // Correction was active when the watchdog tripped

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
   * Counts consecutive failures; once the count reaches
   * FILWIDTH_SENSOR_MAX_CONSECUTIVE_FAILURES the comm watchdog trips and disables
   * correction until the sensor answers again. While tripped, retries continue at the
   * slower FILWIDTH_SENSOR_RECOVERY_INTERVAL_MS cadence so a dead device cannot keep
   * stalling the main loop at the full poll rate. Diagnostic logging is rate-limited.
   *
   * @param now Current system time in milliseconds.
   * @param msg The diagnostic error message.
   * @param code An associated numerical error code.
   * @return unconditionally returns false to cascade the failure state.
   */
  bool fail_and_reschedule(const millis_t now, const char *msg, const int code) {
    ++consecutive_comm_failures;
    next_poll_ms = now + (watchdog_tripped ? sensor_recovery_interval_ms : sensor_poll_interval_ms);
    if (consecutive_comm_failures >= sensor_max_consecutive_failures)
      FilamentWidthSensor::trip_watchdog();
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
  retraction_debt_mm = 0.0f;
  filament_position_mm = 0.0f;
  next_enqueue_mm = 0.0f;

  for (uint8_t axis = 0; axis < sensor_count; ++axis) {
    latest_axes_mm[axis] = nominal_mm;
  }

  clear_alignment_queues();
  refresh_sensor_offsets();

  reset_poll_state();
}

void FilamentWidthSensor::set_delay_cm(uint8_t cm) {
  meas_delay_cm = _MIN(cm, MAX_MEASUREMENT_DELAY);
  refresh_sensor_offsets();
  clear_alignment_queues();
  next_enqueue_mm = filament_position_mm;
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
  consecutive_comm_failures = 0;
  watchdog_tripped = false;
  watchdog_held_correction = false;
}

/**
 * @brief Comm-failure watchdog action: falls back to nominal extrusion after sustained I2C failures.
 *
 * Disables the correction (measuring continues), drops stale queued samples and restores
 * the plain M200-based volumetric multiplier, mirroring what M406 does. Remembers whether
 * correction was active so recovery can restore the previous state without overriding an
 * explicit M406.
 */
void FilamentWidthSensor::trip_watchdog() {
  if (watchdog_tripped) return;
  watchdog_tripped = true;
  watchdog_held_correction = correction_enabled;

  correction_enabled = false;
  clear_alignment_queues();          // Stale spatial samples are meaningless after an outage
  next_enqueue_mm = filament_position_mm;
  planner.calculate_volumetric_multipliers(); // Restore the M200-based multiplier, like M406

  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" FilWidth I2C: comm watchdog tripped - correction disabled until the sensor recovers");
}

/**
 * @brief Restores correction after the sensor answers again (only if the watchdog took it away).
 */
void FilamentWidthSensor::handle_watchdog_recovery() {
  watchdog_tripped = false;
  if (watchdog_held_correction && enabled) correction_enabled = true;

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM(" FilWidth I2C: sensor recovered - correction ");
  if (correction_enabled) SERIAL_ECHOPGM("re-enabled"); else SERIAL_ECHOPGM("stays off");
  SERIAL_EOL();
}

float FilamentWidthSensor::sample_to_size_ratio() {
  const float axis_a = std::max(latest_axes_mm[0], 0.01f);
  const float axis_b = std::max(latest_axes_mm[1], 0.01f);
  const float area = get_area_mm2(axis_a, axis_b);
  if (nominal_area <= 0.0f) return 0.0f;
  return clamp_ratio_percent((nominal_area / area) * 100.0f - 100.0f);
}

void FilamentWidthSensor::schedule_update() {
  filament_update_pending.store(true, std::memory_order_release);
}

void FilamentWidthSensor::service_update() {
  // Stream buffered log rows out in small, fixed-size batches so a dump never
  // blocks the planner or floods the small (512 B) USB-CDC TX buffer. Runs on
  // every temperature-loop pass, independent of the I2C poll gate below.
  if (logging && log_count) {
    flush_log(FILWIDTH_SENSOR_LOG_FLUSH_BATCH);
  }

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

  // Frame integrity: version byte + CRC8 over the data and version bytes.
  if (buffer[sensor_data_bytes] != sensor_version_byte)
    return fail_and_reschedule(now, "bad_version", buffer[sensor_data_bytes]);

  if (buffer[sensor_data_bytes + 1] != crc8_frame(buffer, sensor_data_bytes + 1))
    return fail_and_reschedule(now, "crc_failed", -1);

  uint32_t axes_x10000[sensor_axes] = { 0, 0 };
  if (!decode_sensor_payload(buffer, sensor_data_bytes, axes_x10000))
    return fail_and_reschedule(now, "decode_failed", -1);

  // A frame that passes CRC and decodes proves the sensor is alive: the comm watchdog
  // can stand down (correction is restored only if it disabled it).
  if (consecutive_comm_failures) consecutive_comm_failures = 0;
  if (watchdog_tripped) handle_watchdog_recovery();

  const float axis_a_mm = axes_x10000[0] * sensor_fixed_scale_inv;
  const float axis_b_mm = axes_x10000[1] * sensor_fixed_scale_inv;

  // Plausibility gate (FILWIDTH_ERROR_MARGIN): an axis outside the margin is rejected and
  // the previous measurement is held across this stretch instead of enqueueing garbage.
  const float margin = float(FILWIDTH_ERROR_MARGIN);
  const bool axis_a_plausible = WITHIN(axis_a_mm, nominal_mm - margin, nominal_mm + margin);
  const bool axis_b_plausible = WITHIN(axis_b_mm, nominal_mm - margin, nominal_mm + margin);
  if (!axis_a_plausible || !axis_b_plausible) {
    if (should_warn(now)) {
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM(" FilWidth I2C: sample out of margin a=");
      SERIAL_ECHO(axis_a_mm);
      SERIAL_ECHOPGM(" b=");
      SERIAL_ECHO(axis_b_mm);
      SERIAL_ECHOPGM(" nominal=");
      SERIAL_ECHO(nominal_mm);
      SERIAL_EOL();
    }
  }

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
    if (axis_a_plausible) enqueue_sensor_sample(0, axis_a_mm);
    if (axis_b_plausible) enqueue_sensor_sample(1, axis_b_mm);
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
 * @brief Verifies if the extruder has advanced enough to pop a sample for the specified axis.
 * 
 * A sample is only "ready" when the absolute `filament_position_mm` tracked by the 
 * stepper motors equals or exceeds the spatial target recorded during enqueueing.
 * 
 * @param axis_index The physical axis index (0 or 1).
 * @param axis_mm Output parameter for the axis diameter.
 * @return true if a valid sample was dequeued.
 */
bool FilamentWidthSensor::try_pop_axis_sample(uint8_t axis_index, float &axis_mm) {
  if (axis_index >= sensor_count || !sensor_size[axis_index]) return false;

  const uint16_t head = sensor_head[axis_index];

  if (filament_position_mm < sensor_targets_mm[axis_index][head]) return false;

  axis_mm = sensor_queue[axis_index][head];

  sensor_head[axis_index] = (head + 1) % FILWIDTH_SENSOR_QUEUE_CAPACITY;
  --sensor_size[axis_index];
  return true;
}

/**
 * @brief Dequeues all ready samples and computes the final effective diameter.
 * 
 * Loops over `try_pop_axis_sample` until the queue is either empty or the next 
 * sample hasn't reached the nozzle yet. Updates the global `measured_mm`.
 */
void FilamentWidthSensor::process_ready_samples() {
  bool updated = false;
  float axis_mm;
  
  while (try_pop_axis_sample(0, axis_mm)) {
    latest_axes_mm[0] = axis_mm;
    updated = true;
  }
  
  while (try_pop_axis_sample(1, axis_mm)) {
    latest_axes_mm[1] = axis_mm;
    updated = true;
  }
  
  if (updated) {
    measured_mm = compute_equivalent_diameter(latest_axes_mm[0], latest_axes_mm[1]);
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
  // Track retractions to prevent popping samples for filament that was pulled back
  if (e_move < 0.0f) {
    retraction_debt_mm += -e_move;
  }
  else if (e_move > 0.0f) {
    // Pay off retraction debt before advancing the absolute extrusion tracker
    const float debt_paid = _MIN(retraction_debt_mm, e_move);
    retraction_debt_mm -= debt_paid;
    const float forward_e_move = e_move - debt_paid;

    // Only advance the global filament tracker and process new samples if we moved forward
    if (forward_e_move > 0.0f) {
      filament_position_mm += forward_e_move;
      process_ready_samples();
    }
  }
}

float FilamentWidthSensor::get_averaged_size_ratio(float extrude_mm) {
  if (!enabled || nominal_area <= 0.0f) return 0.0f;

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
    bool can_pop_a = (sim_size_a > 0);
    bool can_pop_b = (sim_size_b > 0);
    float target_a = can_pop_a ? sensor_targets_mm[0][sim_head_a] : end_position_mm + 1.0f;
    float target_b = can_pop_b ? sensor_targets_mm[1][sim_head_b] : end_position_mm + 1.0f;

    // Determine the next simulation boundary (either a queued measurement or the target end position)
    float next_event_mm = end_position_mm;
    next_event_mm = std::min(next_event_mm, target_a);
    next_event_mm = std::min(next_event_mm, target_b);

    const float current_area = get_area_mm2(std::max(sim_latest_a, 0.01f), std::max(sim_latest_b, 0.01f));
    float step_length = next_event_mm - simulated_position_mm;
    
    // Prevent numerical instability leading to an infinite layout loop
    if (step_length <= 0.0f) step_length = 0.0001f;

    total_area_times_length += current_area * step_length;
    simulated_position_mm += step_length;

    // Advance simulated read pointers independently if we crossed a queued sample target
    if (can_pop_a && simulated_position_mm >= target_a) {
      sim_latest_a = sensor_queue[0][sim_head_a];
      sim_head_a = (sim_head_a + 1) % FILWIDTH_SENSOR_QUEUE_CAPACITY;
      sim_size_a--;
    }
    if (can_pop_b && simulated_position_mm >= target_b) {
      sim_latest_b = sensor_queue[1][sim_head_b];
      sim_head_b = (sim_head_b + 1) % FILWIDTH_SENSOR_QUEUE_CAPACITY;
      sim_size_b--;
    }
  }

  // Aggregate area / extrude_mm returns the average area, from which the size ratio is derived.
  const float average_area = total_area_times_length / extrude_mm;
  if (average_area <= 0.0f) return 0.0f;
  last_average_area = average_area;
  return clamp_ratio_percent((nominal_area / average_area) * 100.0f - 100.0f);
}

#endif // FILAMENT_WIDTH_SENSOR
