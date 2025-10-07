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
  bool enabled = false;              // (M405-M406) Filament Width Sensor ON/OFF.
  float nominal_mm = DEFAULT_NOMINAL_FILAMENT_DIA;
  float measured_mm = DEFAULT_MEASURED_FILAMENT_DIA;
  float e_count = 0, delay_dist = 0;
  uint8_t meas_delay_cm = 12;
  int8_t ratios[MMD_CM] = {0};
  int8_t index_r = 0, index_w = 0;

  FilamentWidthSensor() { init(); }
  void init();

  inline void enable(const bool ena) { enabled = ena; }
  inline void set_delay_cm(const uint8_t cm) {
    meas_delay_cm = _MIN(cm, MAX_MEASUREMENT_DELAY);
  }

  int8_t sample_to_size_ratio() {
    return (measured_mm > 0.0f && ABS(nominal_mm - measured_mm) <= FILWIDTH_ERROR_MARGIN)
           ? int(100.0f * nominal_mm / measured_mm) - 100 : 0;
  }

  void update_measured_mm();

  float read_external_sensor();

  void update_volumetric() {
    if (enabled) {
      int8_t read_index = index_r - meas_delay_cm;
      if (read_index < 0) read_index += MMD_CM; // Loop around buffer if needed
      LIMIT(read_index, 0, MAX_MEASUREMENT_DELAY);
      // planner.apply_filament_width_sensor(ratios[read_index]);
    }
  }

  void advance_e(const float &e_move) {
    e_count += e_move;
    delay_dist += e_move;
    if (!UNEAR_ZERO(e_count)) {
      while (delay_dist >= MMD_MM) delay_dist -= MMD_MM;
      index_r = int8_t(delay_dist * 0.1f);
      if (index_r != index_w) {
        e_count = 0;
        update_measured_mm();
        const int8_t meas_sample = sample_to_size_ratio();
        do {
          if (++index_w >= MMD_CM) index_w = 0;
          ratios[index_w] = meas_sample;
        } while (index_r != index_w);
      }
    }
  }
};

extern FilamentWidthSensor filwidth;
