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

#if ENABLED(FILAMENT_WIDTH_SENSOR) && DISABLED(FILWIDTH_SENSOR_USE_I2C)
  #error "FILAMENT_WIDTH_SENSOR now requires FILWIDTH_SENSOR_USE_I2C"
#endif

#if ENABLED(FILWIDTH_SENSOR_USE_I2C)
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
#endif

class FilamentWidthSensor {
public:
  static constexpr int MMD_CM = MAX_MEASUREMENT_DELAY + 1;
  static constexpr int MMD_MM = MMD_CM * 10;

  static bool enabled;              // (M405-M406) Filament Width Sensor ON/OFF.
  static float nominal_mm;          // (M104) Nominal filament width
  static float measured_mm;         // Measured filament diameter (equivalent diameter for ellipse)
  static float nominal_area;        // Reference ellipse area for nominal filament
  static float e_count;             // Extruder movement accumulator
  static float delay_dist;          // Running distance through the delay buffer
  static uint8_t meas_delay_cm;     // Distance delay setting
  static float ratios[MMD_CM];      // Ring buffer to delay measurement. (Extruder factor minus 100)
  static int8_t index_r;            // Index into ring buffer for reading
  static int8_t index_w;            // Index into ring buffer for writing

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

  /**
   * @brief Pushes a raw sensor measurement into the spatial alignment queue.
   * @param sensor_index The physical axis index (0 or 1).
   * @param diameter_mm The measured diameter on this axis.
   */
  static void enqueue_sensor_sample(uint8_t sensor_index, float diameter_mm);

  /**
   * @brief Attempts to pop a synchronized sample pair that has reached the nozzle.
   * @param axis_a_mm Output parameter for the first axis.
   * @param axis_b_mm Output parameter for the second axis.
   * @return true if a synchronized pair was popped, false if still buffered.
   */
  static bool try_pop_aligned_sample(float &axis_a_mm, float &axis_b_mm);

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
   * @brief Tracks the physical movement of the filament, triggering queue pops and integration.
   * @param e_move The physical distance the extruder motor moved.
   */
  static void advance_e(const float &e_move);
#if DISABLED(FILWIDTH_SENSOR_USE_I2C)
  static inline void update_volumetric() {
    if (enabled) {
      int8_t read_index = index_r;
      read_index -= meas_delay_cm;
      if (read_index < 0) read_index += MMD_CM; // Loop around buffer if needed
      LIMIT(read_index, 0, MAX_MEASUREMENT_DELAY);
      planner.apply_filament_width_sensor(ratios[read_index]);
    }
  }
#endif

private:
  static inline float get_area_mm2(const float major_mm, const float minor_mm) {
    return 0.25f * PI * major_mm * minor_mm;
  }
  static inline void refresh_nominal_area() { nominal_area = get_area_mm2(nominal_mm, nominal_mm); }
  static void clear_alignment_queues();
  static void refresh_sensor_offsets();
  static void reset_poll_state();
};

extern FilamentWidthSensor filwidth;
