// ...existing code...
#include <stdint.h>
#include "filwidth.h"

uint8_t FilamentWidthSensor::meas_delay_cm = 12; // Default 120mm from sensor to melt zone

void FilamentWidthSensor::set_delay_cm(const uint8_t cm) {
  meas_delay_cm = cm;
}
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
#include "i2c.hpp"

FilamentWidthSensor filwidth;

bool FilamentWidthSensor::enabled;
uint32_t FilamentWidthSensor::accum;
uint16_t FilamentWidthSensor::raw;
float FilamentWidthSensor::nominal_mm = DEFAULT_NOMINAL_FILAMENT_DIA,
      FilamentWidthSensor::measured_mm = DEFAULT_MEASURED_FILAMENT_DIA,
      FilamentWidthSensor::e_count = 0,
      FilamentWidthSensor::delay_dist = 0;
uint8_t FilamentWidthSensor::meas_delay_cm = 12; // 120mm from sensor to melt zone
int8_t FilamentWidthSensor::ratios[MAX_MEASUREMENT_DELAY + 1],
       FilamentWidthSensor::index_r,
       FilamentWidthSensor::index_w;

void FilamentWidthSensor::init() {
  // No ring buffer needed, but keep for compatibility
  for (uint8_t i = 0; i < COUNT(ratios); ++i) ratios[i] = 0;
  index_r = index_w = 0;
}

float FilamentWidthSensor::read_external_sensor() {
  float width = DEFAULT_NOMINAL_FILAMENT_DIA;
  uint8_t data[5] = {0};
  constexpr uint8_t EXTERNAL_SENSOR_I2C_ADDRESS = 0x42; // Set to your ESP32's address
  constexpr uint32_t I2C_TIMEOUT_MS = 100;
  constexpr i2c::Port I2C_PORT = i2c::Port::port0; // Use the port previously used by expander
  i2c::Result res = i2c::Receive(I2C_PORT, (EXTERNAL_SENSOR_I2C_ADDRESS << 1) | 0x01, data, sizeof(data), I2C_TIMEOUT_MS);
  if (res == i2c::Result::ok) {
    char str[8] = {0};
    memcpy(str, data, 5);
    str[1] = '.'; // Insert decimal after first digit
    width = atof(str);
  }
  return width;
}

void FilamentWidthSensor::update_measured_mm() {
  measured_mm = read_external_sensor();
}

#endif // FILAMENT_WIDTH_SENSOR
