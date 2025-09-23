#pragma once

// Initialize the filament width sensor
void filament_width_sensor_init();

// Read the current filament width
float filament_width_sensor_read();

// Update the filament width measurements (call this periodically)
void filament_width_sensor_update();
