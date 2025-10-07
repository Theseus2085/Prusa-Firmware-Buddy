#ifndef MAX_MEASUREMENT_DELAY
#define MAX_MEASUREMENT_DELAY 12
#endif
#ifndef FILWIDTH_ERROR_MARGIN
#define FILWIDTH_ERROR_MARGIN 0.02f
#endif
#ifndef DEFAULT_NOMINAL_FILAMENT_DIA
#define DEFAULT_NOMINAL_FILAMENT_DIA 1.75f
#endif
#ifndef DEFAULT_MEASURED_FILAMENT_DIA
#define DEFAULT_MEASURED_FILAMENT_DIA 1.75f
#endif
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

class FilamentWidthSensor {
public:
  static constexpr int MMD_CM = MAX_MEASUREMENT_DELAY + 1, MMD_MM = MMD_CM * 10;
  static bool enabled;              // (M405-M406) Filament Width Sensor ON/OFF.
  // Removed analog accumulator and raw ADC value
  static float nominal_mm,          // (M104) Nominal filament width
               measured_mm,         // Measured filament diameter
               e_count, delay_dist;
  static uint8_t meas_delay_cm;     // Distance delay setting
  static int8_t ratios[MMD_CM],     // Ring buffer to delay measurement. (Extruder factor minus 100)
                index_r, index_w;   // Indexes into ring buffer

  FilamentWidthSensor() { init(); }
  static void init();

  static inline void enable(const bool ena) { enabled = ena; }
  static inline void set_delay_cm(const uint8_t cm) {
    meas_delay_cm = _MIN(cm, MAX_MEASUREMENT_DELAY);
  }

  /**
   * Convert Filament Width (mm) to an extrusion ratio
   * and reduce to an 8 bit value.
   *
   * A nominal width of 1.75 and measured width of 1.73
   * gives (100 * 1.75 / 1.73) for a ratio of 101 and
   * a return value of 1.
   */
  // Compute extrusion ratio from I2C sensor value
  static int8_t sample_to_size_ratio() {
    return (measured_mm > 0.0f && ABS(nominal_mm - measured_mm) <= FILWIDTH_ERROR_MARGIN)
           ? int(100.0f * nominal_mm / measured_mm) - 100 : 0;
  }

  // Update measured_mm from I2C sensor
  static void update_measured_mm();

  // Update ring buffer used to delay filament measurements
  static inline void advance_e(const float &e_move) {
    // Increment counters with the E distance
    e_count += e_move;
    delay_dist += e_move;
    // Only get new measurements on forward E movement
    if (!UNEAR_ZERO(e_count)) {
      while (delay_dist >= MMD_MM) delay_dist -= MMD_MM;
      index_r = int8_t(delay_dist * 0.1f);
      if (index_r != index_w) {
        e_count = 0;
        update_measured_mm(); // Read from I2C sensor
        const int8_t meas_sample = sample_to_size_ratio();
        do {
          if (++index_w >= MMD_CM) index_w = 0;
          ratios[index_w] = meas_sample;
        } while (index_r != index_w);
      }
    }
  }

  // Dynamically set the volumetric multiplier based on the delayed width measurement.
  static inline void update_volumetric() {
    if (enabled) {
      int8_t read_index = index_r - meas_delay_cm;
      if (read_index < 0) read_index += MMD_CM; // Loop around buffer if needed
      LIMIT(read_index, 0, MAX_MEASUREMENT_DELAY);
  ::planner.apply_filament_width_sensor(ratios[read_index]);
    }
  }

};

extern FilamentWidthSensor filwidth;
