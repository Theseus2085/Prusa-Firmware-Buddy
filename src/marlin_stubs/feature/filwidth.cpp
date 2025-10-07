#include "../../../lib/Marlin/Marlin/src/Marlin.h"
#include "../../../lib/Marlin/Marlin/src/feature/filwidth.h"
#include "../../../lib/Marlin/Marlin/src/module/planner.h"

FilamentWidthSensor filwidth;

void FilamentWidthSensor::init() {
    for (uint8_t i = 0; i < MMD_CM; ++i) {
        ratios[i] = 0;
    }
    index_r = index_w = 0;
    measured_mm = DEFAULT_MEASURED_FILAMENT_DIA;
    nominal_mm = DEFAULT_NOMINAL_FILAMENT_DIA;
    enabled = true;
}

float FilamentWidthSensor::read_external_sensor() {
    // TODO: Replace with actual ESP32/I2C transfer. Returning nominal diameter for now.
    return DEFAULT_MEASURED_FILAMENT_DIA;
}

void FilamentWidthSensor::update_measured_mm() {
    measured_mm = read_external_sensor();
    // Update volumetric multiplier if the planner is ready.
    planner.calculate_volumetric_multipliers();
}
