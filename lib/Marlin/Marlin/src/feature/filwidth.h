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
#pragma once

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"

// The external I2C sensor module is the only supported filament width sensor.
#ifndef FILWIDTH_SENSOR_QUEUE_BIN_MM
  #define FILWIDTH_SENSOR_QUEUE_BIN_MM 1
#endif
#ifndef FILWIDTH_SENSOR_MAX_DELAY_MM
  #define FILWIDTH_SENSOR_MAX_DELAY_MM ((MAX_MEASUREMENT_DELAY * 10) + int(FILWIDTH_SENSOR_SPACING_MM + 0.5f))
#endif
#ifndef FILWIDTH_SENSOR_QUEUE_CAPACITY
  #define FILWIDTH_SENSOR_QUEUE_CAPACITY ((FILWIDTH_SENSOR_MAX_DELAY_MM / FILWIDTH_SENSOR_QUEUE_BIN_MM) + 8)
#endif
#ifndef FILWIDTH_SENSOR_READY_RETRIES
  #define FILWIDTH_SENSOR_READY_RETRIES 3
#endif
#ifndef FILWIDTH_SENSOR_POLL_INTERVAL_MS
  #define FILWIDTH_SENSOR_POLL_INTERVAL_MS 250UL
#endif
#ifndef FILWIDTH_SENSOR_WARN_INTERVAL_MS
  #define FILWIDTH_SENSOR_WARN_INTERVAL_MS 3000UL
#endif
#ifndef FILWIDTH_SENSOR_RX_PRINT_INTERVAL_MS
  #define FILWIDTH_SENSOR_RX_PRINT_INTERVAL_MS 5000UL
#endif
#ifndef FILWIDTH_SENSOR1_OFFSET_MM
  #define FILWIDTH_SENSOR1_OFFSET_MM (MEASUREMENT_DELAY_CM * 10.0f)
#endif
#ifndef FILWIDTH_SENSOR_SPACING_MM
  #define FILWIDTH_SENSOR_SPACING_MM 1.0f
#endif
#ifndef FILWIDTH_SENSOR2_OFFSET_MM
  #define FILWIDTH_SENSOR2_OFFSET_MM (FILWIDTH_SENSOR1_OFFSET_MM + FILWIDTH_SENSOR_SPACING_MM)
#endif
#ifndef FILWIDTH_SENSOR_PROTOCOL_VERSION
  #define FILWIDTH_SENSOR_PROTOCOL_VERSION 0x01
#endif
#ifndef FILWIDTH_SENSOR_MAX_RATIO_PERCENT
  #define FILWIDTH_SENSOR_MAX_RATIO_PERCENT 20.0f
#endif
#ifndef FILWIDTH_SENSOR_MAX_CONSECUTIVE_FAILURES
  #define FILWIDTH_SENSOR_MAX_CONSECUTIVE_FAILURES 8
#endif
  #ifndef FILWIDTH_SENSOR_RECOVERY_INTERVAL_MS
    #define FILWIDTH_SENSOR_RECOVERY_INTERVAL_MS 2000UL
  #endif
  #ifndef FILWIDTH_SENSOR_LOG_FLUSH_BATCH
    #define FILWIDTH_SENSOR_LOG_FLUSH_BATCH 16
  #endif

struct FilLogEntry {
  uint32_t time_ms;      ///< millis() when the planner block was populated
  float x;               ///< target X of the block (mm)
  float y;               ///< target Y of the block (mm)
  float e_factor;        ///< applied E factor (flow * volumetric multiplier)
  int32_t msteps_raw;    ///< uncorrected E msteps requested by the block (de)
  uint32_t msteps_corr;  ///< corrected E msteps as executed (de * e_factor, rounded)
  float axis_a_mm;       ///< axis 1 diameter at the nozzle when the block was planned
  float axis_b_mm;       ///< axis 2 diameter at the nozzle when the block was planned
  float ellipse_area;    ///< length-weighted average cross-section area over the block (mm^2)
};

class FilamentWidthSensor {
public:
  static bool enabled;              // (M405-M406) Filament Width Sensor ON/OFF.
  static bool correction_enabled;   // (M406) Apply the sensor correction to e_factor. Sensor keeps measuring when false.
  static float nominal_mm;          // Nominal filament width
  static float measured_mm;         // Measured filament diameter (equivalent diameter for ellipse)
  static float nominal_area;        // Reference ellipse area for nominal filament
  static uint8_t meas_delay_cm;     // Distance delay setting (sensor 1 offset from the nozzle)

  static constexpr uint8_t sensor_count = 2;

  /**
   * @brief Spatial alignment queues for the multi-axis sensor data.
   * 
   * These ring buffers synchronize the physical distance between the measurement point 
   * and the extruder nozzle. Each axis has its independent queue and target tracking.
   */
  static float sensor_queue[sensor_count][FILWIDTH_SENSOR_QUEUE_CAPACITY];
  static float sensor_targets_mm[sensor_count][FILWIDTH_SENSOR_QUEUE_CAPACITY];
  static uint16_t sensor_head[sensor_count];
  static uint16_t sensor_size[sensor_count];
  static float sensor_offsets_mm[sensor_count];
  
  static float filament_position_mm;   ///< Absolute filament extrusion tracker
  static float retraction_debt_mm;     ///< Tracks retracted distance to pause sensor enqueueing
  static float next_enqueue_mm;        ///< Distance threshold for the next sensor read
  static float latest_axes_mm[sensor_count]; ///< Most recently popped aligned samples
  static float last_average_area;            ///< Most recently computed average integral area

  // RAM budget: entry = 36 bytes. 650 entries = ~23.4 KB, matching the budget the
  // mk4 release+boot build tolerated with the old 1000x24B layout (larger values
  // caused heap exhaustion mid-print). Entries are streamed out in small batches
  // by flush_log() from the main loop (never from the planner), so the buffer is
  // a shock absorber, not long-term storage.
  static constexpr uint16_t log_capacity = 650;
  static FilLogEntry log_array[log_capacity];
  static uint16_t log_count;
  static uint16_t log_dropped;   // entries discarded because the buffer was full
  static bool logging;

  /**
   * @brief Streams up to max_entries buffered log rows out over serial (FWLOG lines).
   * @return Number of entries actually flushed.
   * Must be called from the main loop (marlin task), never from the planner.
   */
  static uint16_t flush_log(uint16_t max_entries);

  /**
   * @brief Pushes a raw sensor measurement into the spatial alignment queue.
   * @param sensor_index The physical axis index (0 or 1).
   * @param diameter_mm The measured diameter on this axis.
   */
  static void enqueue_sensor_sample(uint8_t sensor_index, float diameter_mm);

  /**
   * @brief Attempts to pop a single sample from the specified axis that has reached the nozzle.
   * @param axis_index The physical axis index (0 or 1).
   * @param axis_mm Output parameter for the axis diameter.
   * @return true if a sample was popped, false if still buffered or empty.
   */
  static bool try_pop_axis_sample(uint8_t axis_index, float &axis_mm);

  /**
   * @brief Processes all samples that have reached the extruder nozzle and updates the effective diameter.
   */
  static void process_ready_samples();

  /**
   * @brief Computes the nominal circular equivalent diameter of the elliptical cross-section.
   * @param axis_a_mm Major/minor axis A
   * @param axis_b_mm Major/minor axis B
   * @return The equivalent circular diameter
   */
  static float compute_equivalent_diameter(float axis_a_mm, float axis_b_mm);

  FilamentWidthSensor() { init(); }

  static void init();
  static inline void enable(const bool ena) {
    if (ena && !enabled) reset_poll_state();
    enabled = ena;
    if (ena) correction_enabled = true;  // re-enabling the sensor always restores correction
  }
  static void set_delay_cm(uint8_t cm);
  static void set_nominal_mm(float mm);

  /**
   * @brief Calculates the percentage size ratio based on current nominal and measured values.
   */
  static float sample_to_size_ratio();

  /**
   * @brief Flags a pending I2C transaction from the timer ISR.
   */
  static void schedule_update();

  /**
   * @brief Main loop service routine to process pending I2C transactions asynchronously.
   */
  static void service_update();

  /**
   * @brief Executes the I2C read and decodes the sensor payload into the alignment queues.
   * @return true on successful read and parse.
   */
  static bool update_from_sensor();

  /**
   * @brief Performs a lookahead simulation to calculate the length-weighted volumetric error.
   * @param extrude_mm The physical length of the upcoming extrusion.
   * @return The average area ratio adjustment (multiplier offset).
   */
  static float get_averaged_size_ratio(float extrude_mm);

  /**
   * @brief Comm-watchdog action: falls back to nominal extrusion after sustained I2C failures.
   */
  static void trip_watchdog();

  /**
   * @brief Restores correction after the sensor answers again (only if the watchdog took it away).
   */
  static void handle_watchdog_recovery();

  /**
   * @brief Tracks the physical movement of the filament, triggering queue pops and integration.
   * @param e_move The physical distance the extruder motor moved.
   */
  static void advance_e(const float &e_move);

private:
  static inline float get_area_mm2(const float major_mm, const float minor_mm) {
    return 0.25f * (float)PI * major_mm * minor_mm;
  }
  static inline void refresh_nominal_area() { nominal_area = get_area_mm2(nominal_mm, nominal_mm); }
  static void clear_alignment_queues();
  static void refresh_sensor_offsets();
  static void reset_poll_state();
};

extern FilamentWidthSensor filwidth;
