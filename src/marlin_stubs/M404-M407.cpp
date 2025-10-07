#include "../../lib/Marlin/Marlin/src/Marlin.h"
#include "../../lib/Marlin/Marlin/src/feature/filwidth.h"
#include "../../lib/Marlin/Marlin/src/module/planner.h"
#include "../../lib/Marlin/Marlin/src/gcode/gcode.h"

#if ENABLED(FILAMENT_WIDTH_SENSOR)

void GcodeSuite::M404() {
    if (parser.seenval('W')) {
        filwidth.nominal_mm = parser.value_linear_units();
        planner.volumetric_area_nominal = CIRCLE_AREA(filwidth.nominal_mm * 0.5f);
        planner.calculate_volumetric_multipliers();
    } else {
        SERIAL_ECHOLNPAIR("Filament dia (nominal mm):", filwidth.nominal_mm);
    }
}

void GcodeSuite::M405() {
    filwidth.enable(true);
    if (parser.seenval('D')) {
        filwidth.set_delay_cm(parser.value_byte());
    }
}

void GcodeSuite::M406() {
    filwidth.enable(false);
    planner.calculate_volumetric_multipliers();
}

void GcodeSuite::M407() {
    filwidth.update_measured_mm();
    SERIAL_ECHOLNPAIR("Filament dia (measured mm):", filwidth.measured_mm);
}

#endif // FILAMENT_WIDTH_SENSOR
