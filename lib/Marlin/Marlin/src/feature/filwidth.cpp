#include "../inc/MarlinConfig.h"
#include "filwidth.h"
#include "i2c.hpp"
#include <stdint.h>
#include "../planner.h"

// All members are now instance-based
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



FilamentWidthSensor filwidth;

void FilamentWidthSensor::init() {
  for (uint8_t i = 0; i < MMD_CM; ++i) ratios[i] = 0;
  index_r = index_w = 0;
}

// Helper for I2C sensor read
float FilamentWidthSensor::read_external_sensor() {
  float width = DEFAULT_NOMINAL_FILAMENT_DIA;
  // TODO: Implement actual I2C read here
  // uint8_t data[5] = {0};
  // ...
  // width = ...;
  return width;
}

void FilamentWidthSensor::update_measured_mm() {
  measured_mm = FilamentWidthSensor::read_external_sensor();
}


// update_volumetric is implemented inline in the header

