
#include <stdint.h>

class FilamentWidthSensor {
public:
  static void set_delay_cm(const uint8_t cm);
  static uint8_t meas_delay_cm;
  static void init();
  static float read_external_sensor();
  static void update_measured_mm();
  static float nominal_mm;
  static float measured_mm;
};

extern FilamentWidthSensor filwidth;
