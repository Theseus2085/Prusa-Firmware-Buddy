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
  #ifndef FILWIDTH_SENSOR_QUEUE_CAPACITY
    #define FILWIDTH_SENSOR_QUEUE_CAPACITY (MAX_MEASUREMENT_DELAY + 1)
  #endif
  #ifndef FILWIDTH_SENSOR_VALUE_SCALE
    #define FILWIDTH_SENSOR_VALUE_SCALE 0.01f
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

  // Two-dimensional circular queues keep each axis aligned to the E position where it reaches the nozzle.
  static float sensor_queue[sensor_count][FILWIDTH_SENSOR_QUEUE_CAPACITY];
  static float sensor_targets_mm[sensor_count][FILWIDTH_SENSOR_QUEUE_CAPACITY];
  static uint8_t sensor_head[sensor_count];
  static uint8_t sensor_size[sensor_count];
  static float sensor_offsets_mm[sensor_count];
  static float filament_position_mm;
  static float latest_axes_mm[sensor_count];

  static void enqueue_sensor_sample(uint8_t sensor_index, float diameter_mm);
  static bool try_pop_aligned_sample(float &axis_a_mm, float &axis_b_mm);
  static void process_ready_samples();
  static float compute_equivalent_diameter(float axis_a_mm, float axis_b_mm);

  FilamentWidthSensor() { init(); }

  static void init();
  static inline void enable(const bool ena) {
    if (ena && !enabled) reset_poll_state();
    enabled = ena;
  }
  static inline void set_delay_cm(const uint8_t cm) { meas_delay_cm = _MIN(cm, MAX_MEASUREMENT_DELAY); }

  static float sample_to_size_ratio();
  static void schedule_update();
  static void service_update();
  static bool update_from_sensor();

  static void advance_e(const float &e_move);
  static inline void update_volumetric() {
    if (enabled) {
      int8_t read_index = index_r - meas_delay_cm;
      if (read_index < 0) read_index += MMD_CM; // Loop around buffer if needed
      LIMIT(read_index, 0, MAX_MEASUREMENT_DELAY);
      planner.apply_filament_width_sensor(ratios[read_index]);
    }
  }

private:
  static inline float get_area_mm2(const float major_mm, const float minor_mm) {
    return 0.25f * PI * major_mm * minor_mm;
  }
  static inline void refresh_nominal_area() { nominal_area = get_area_mm2(nominal_mm, nominal_mm); }
  static void reset_poll_state();
};

extern FilamentWidthSensor filwidth;
