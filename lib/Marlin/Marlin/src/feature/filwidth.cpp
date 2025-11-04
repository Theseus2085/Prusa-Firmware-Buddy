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

#if ENABLED(FILWIDTH_SENSOR_USE_I2C)
  #include <atomic>
  #include "i2c.hpp"

  namespace {

    // I2C address for the filament width sensor (currently set to 0x42 for testing with ESP32)
    constexpr uint8_t sensor_address = 0x42; // FILWIDTH_SENSOR_I2C_ADDRESS
    
    // Number of digits we expect from the sensor reading
    constexpr uint8_t sensor_digits = FILWIDTH_SENSOR_DIGITS;
    
    // Timeout in milliseconds for I2C operations - don't want to block forever if sensor doesn't respond
    constexpr uint32_t sensor_timeout_ms = FILWIDTH_SENSOR_TIMEOUT_MS;

    /**
     * Decode the sensor's ASCII digit response into a float value representing filament diameter in mm.
     * The sensor sends digits like "175" which we interpret as 1.75mm.
     * 
     * @param buffer Raw bytes received from the sensor
     * @param length How many bytes we got
     * @param out_mm Where to store the decoded diameter value
     * @return true if we successfully decoded a valid measurement, false otherwise
     */
    bool decode_sensor_digits(const uint8_t *buffer, const uint8_t length, float &out_mm) {
      if (!buffer) return false;

      uint8_t digits_collected = 0;
      uint8_t digits[sensor_digits] = { 0 };

      // Parse through the buffer, extracting digits and ignoring whitespace/punctuation
      for (uint8_t i = 0; i < length && digits_collected < sensor_digits; ++i) {
        const uint8_t value = buffer[i];

        // Skip newlines, carriage returns, and spaces - sensor might send formatting chars
        if (value == '\n' || value == '\r' || value == ' ') continue;
        
        // Skip decimal points - we'll handle the decimal placement ourselves
        if (value == '.') continue;

        uint8_t digit;
        // Handle raw binary digits (0-9)
        if (value <= 9) {
          digit = value;
        }
        // Handle ASCII digits ('0'-'9')
        else if (value >= '0' && value <= '9') {
          digit = value - '0';
        }
        else {
          // Got something that's not a digit - bail out
          return false;
        }

        digits[digits_collected++] = digit;
      }

      // Make sure we got exactly the right number of digits
      if (digits_collected != sensor_digits)
        return false;

      // Convert digits to a float. First digit is the ones place (e.g., "1" in 1.75)
      float result = digits[0];
      float scale = 0.1f;
      
      // Remaining digits are decimal places (e.g., "75" in 1.75)
      for (uint8_t i = 1; i < sensor_digits; ++i, scale *= 0.1f)
        result += digits[i] * scale;

      // Sanity check - filament diameter should be between 0.5mm and 3.5mm
      // Anything outside this range is probably garbage data
      if (!WITHIN(result, 0.5f, 3.5f))
        return false;

      out_mm = result;
      return true;
    }

  } // namespace

  namespace {
    // Thread-safe flag to track whether a sensor update has been requested
    // We use atomic because this might be set from an interrupt and read from main loop
    std::atomic<bool> filament_update_pending { false };
  }

#endif

FilamentWidthSensor filwidth;
bool FilamentWidthSensor::enabled = true; // (M405-M406) Filament Width Sensor ON/OFF.
float FilamentWidthSensor::nominal_mm = DEFAULT_NOMINAL_FILAMENT_DIA,   // (M104) Nominal filament width
      FilamentWidthSensor::measured_mm = DEFAULT_MEASURED_FILAMENT_DIA, // Measured filament diameter
      FilamentWidthSensor::e_count = 0,
      FilamentWidthSensor::delay_dist = 0;
uint8_t FilamentWidthSensor::meas_delay_cm = MEASUREMENT_DELAY_CM;      // Distance delay setting
int8_t FilamentWidthSensor::ratios[MAX_MEASUREMENT_DELAY + 1],          // Ring buffer to delay measurement. (Extruder factor minus 100)
       FilamentWidthSensor::index_r,                                    // Indexes into ring buffer
       FilamentWidthSensor::index_w;

#if DISABLED(FILWIDTH_SENSOR_USE_I2C)
uint32_t FilamentWidthSensor::accum; // = 0                             // ADC accumulator
uint16_t FilamentWidthSensor::raw; // = 0                               // Measured filament diameter - one extruder only
#endif

void FilamentWidthSensor::init() {
  const int8_t ratio = sample_to_size_ratio();
  for (uint8_t i = 0; i < COUNT(ratios); ++i) ratios[i] = ratio;
  index_r = index_w = 0;
}

#if ENABLED(FILWIDTH_SENSOR_USE_I2C)

/**
 * Request a sensor update from interrupt context.
 * This just sets a flag - the actual I2C communication happens later in service_update()
 * don't want to do slow I2C stuff in an interrupt.
 */
void FilamentWidthSensor::schedule_update() {
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth schedule_update");
  // Set the flag atomically so the main loop knows to read the sensor
  filament_update_pending.store(true, std::memory_order_relaxed);
}

/**
 * Service a pending sensor update in the main task context.
 * This is where the actual I2C communication happens - called from the main loop,
 * not from an interrupt, so it's safe to do blocking I2C operations here.
 */
void FilamentWidthSensor::service_update() {
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth service_update");
  
  // Check if an update was requested, and clear the flag atomically
  if (!filament_update_pending.exchange(false, std::memory_order_acq_rel)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(" filwidth service_update skip-no-pending");
    return; // No update pending, nothing to do
  }

  // Don't bother reading the sensor if the feature is disabled
  if (!enabled) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(" filwidth service_update skip-disabled");
    return;
  }

  // Do the actual sensor read
  update_from_sensor();
}

/**
 * Scan all three I2C buses and log any devices that respond.
 * Useful for debugging - helps you figure out which bus your sensor is on
 * and what address it's responding to.
 */
void FilamentWidthSensor::log_i2c_devices() {
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth i2c scan begin");

  // Lookup table for hex digit conversion (for pretty-printing addresses)
  constexpr char hex_digits[] = "0123456789ABCDEF";
  
  // Define which I2C buses to scan
  struct BusInfo {
    I2C_HandleTypeDef &handle;
    const char *name;
  };

  BusInfo buses[] = {
    { hi2c1, "hi2c1" },
    { hi2c2, "hi2c2" },
    { hi2c3, "hi2c3" }
  };

  // Scan each bus
  for (const auto &bus : buses) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR(" filwidth i2c bus=", bus.name);

    uint8_t found = 0;

    // Try every valid 7-bit I2C address (0x01 to 0x7F)
    // We skip 0x00 because that's reserved
    for (uint8_t address = 0x01; address < 0x80; ++address) {
      // Ping the device - if it responds, it's there
      const auto status = i2c::IsDeviceReady(bus.handle, address << 1, 1, sensor_timeout_ms);
      if (status == i2c::Result::ok) {
        ++found;
        // Print the address in hex (e.g., "0x42")
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM(" filwidth i2c addr 0x");
        SERIAL_CHAR(hex_digits[(address >> 4) & 0x0F]); // High nibble
        SERIAL_CHAR(hex_digits[address & 0x0F]);        // Low nibble
        SERIAL_ECHOLNPGM("");
      }
    }

    // Report how many devices where found on this bus
    SERIAL_ECHO_START();
    if (found == 0) {
      SERIAL_ECHOLNPGM(" filwidth i2c scan done - no devices");
    }
    else {
      SERIAL_ECHOLNPAIR(" filwidth i2c scan device_count=", int(found));
    }
  }
}

/**
 * Read the filament width sensor over I2C and update the measurement.
 * This is the main workhorse function that:
 * 1. Runs a full I2C bus scan (for debugging)
 * 2. Checks if the sensor is ready to talk
 * 3. Reads the raw data from the sensor
 * 4. Decodes it into a filament diameter measurement
 * 
 * @return true if successfully read and decoded a measurement, false on any error
 */
bool FilamentWidthSensor::update_from_sensor() {
  // First, do a full bus scan to see what's out there (helpful for debugging)
  log_i2c_devices();
  
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth update_from_sensor");
  
  // Buffer to hold the raw bytes from the sensor
  uint8_t buffer[sensor_digits] = { 0 };

  // Check if the sensor is ready to communicate
  // use hi2c2 (the IO expander bus) and left-shift address because I2C uses 8-bit addresses
  const auto ready = i2c::IsDeviceReady(hi2c2, sensor_address << 1, 1, sensor_timeout_ms);
  if (ready != i2c::Result::ok) {
    SERIAL_ECHO_START();
    // Log different error types for debugging
    switch (ready) {
      case i2c::Result::error:
        SERIAL_ECHOLNPAIR(" filwidth device_not_ready=errorinecho", int(ready));
        break;
      case i2c::Result::busy_after_retries:
        SERIAL_ECHOLNPGM(" filwidth device_not_ready=busy");
        break;
      case i2c::Result::timeout:
        SERIAL_ECHOLNPGM(" filwidth device_not_ready=timeout");
        break;
      default:
        SERIAL_ECHOLNPAIR(" filwidth device_not_ready=realerror", int(ready));
        break;
    }
    return false;
  }

  // Sensor is ready, so read the data
  // OR with 0x1 to set the read bit in the I2C address
  const auto result = i2c::Receive(I2C_HANDLE_FOR(io_expander2), (sensor_address << 1) | 0x1, buffer, sensor_digits, sensor_timeout_ms);
  if (result != i2c::Result::ok) {
    SERIAL_ECHO_START();
    // Different errors mean different things - log them all
    switch (result) {
      case i2c::Result::error:
        SERIAL_ECHOLNPGM(" filwidth i2c failure=errorrrrrr");
        break;
      case i2c::Result::busy_after_retries:
        SERIAL_ECHOLNPGM(" filwidth i2c failure=busy");
        break;
      case i2c::Result::timeout:
        SERIAL_ECHOLNPGM(" filwidth i2c failure=timeout");
        break;
      default:
        SERIAL_ECHOLNPAIR(" filwidth i2c failure=", int(result));
        break;
    }
    return false;
  }

  // We got data! Log it for debugging
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth i2c ok");

  // Print the raw bytes as decimal numbers
  SERIAL_ECHO_START();
  SERIAL_ECHOPGM(" filwidth raw:");
  for (uint8_t i = 0; i < sensor_digits; ++i) {
    SERIAL_ECHOPGM(" ");
    SERIAL_ECHO(int(buffer[i]));
  }
  
  // Also print as ASCII characters (if printable) - helps see if sensor is sending text
  SERIAL_ECHOPGM(" ascii:'");
  for (uint8_t i = 0; i < sensor_digits; ++i)
    SERIAL_CHAR((buffer[i] >= 32 && buffer[i] <= 126) ? buffer[i] : '.'); // Replace unprintable chars with '.'
  SERIAL_CHAR('\'');
  SERIAL_EOL();

  // Try to decode the raw bytes into a measurement
  float measured_value = measured_mm; // Start with current value as fallback
  if (!decode_sensor_digits(buffer, sensor_digits, measured_value)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(" filwidth decode failed");
    return false; // Couldn't make sense of what the sensor sent
  }

  // Success! We decoded a valid measurement
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(" filwidth decode ok");
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPAIR(" filwidth decoded=", measured_value);
  
  // Update our stored measurement
  measured_mm = measured_value;
  return true;
}

#endif

#endif // FILAMENT_WIDTH_SENSOR
