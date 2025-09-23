#include "filament_width_sensor.h"
#include "hwio_pindef.h"
#include "marlin_server.hpp"
#include "../Marlin/src/feature/filwidth.h"
#include "../Marlin/src/module/planner.h"
#include <Arduino.h>

// Initialize communication with your digital sensor
void filament_width_sensor_init() {
    // Set up the pin
    hwio_configure_pin(filamentWidthSensor);

    // Initialize your digital sensor's communication protocol
    // This depends on your specific sensor's requirements

    // Initialize the Marlin filament width system
    filwidth.init();

    // Set default state (usually disabled until M405 is sent)
    filwidth.enabled = false;
}

// Read the measurement from your digital sensor
float filament_width_sensor_read() {
    // Implement communication with your digital sensor here
    // This will depend on your sensor's protocol (I2C, SPI, UART, etc.)

    // For testing, return a static value - replace with actual sensor reading
    float measured_width = 1.75; // Default filament width in mm

    return measured_width;
}

// This function should be called periodically to update filament measurements
void filament_width_sensor_update() {
    static uint32_t last_update = 0;
    const uint32_t update_interval = 1000; // Update every 1 second

    // Only process if the sensor is enabled via M405
    if (!filwidth.enabled) {
        return;
    }

    uint32_t now = millis();
    if (now - last_update >= update_interval) {
        last_update = now;

        // Read the width from your sensor
        float width = filament_width_sensor_read();

        // Update Marlin's filament measurement
        filwidth.measured_mm = width;

        // Calculate ratio for volumetric extrusion
        filwidth.calculate_volumetric_multiplier();
    }
}
