#include "PrusaGcodeSuite.hpp"
#include "../../lib/Marlin/Marlin/src/gcode/parser.h"

// Legacy IO expander commands now emit an informative warning instead of interacting with removed hardware.

#include "../../lib/Marlin/Marlin/src/Marlin.h"
#if HAS_I2C_EXPANDER()

// Legacy IO expander commands now emit an informative warning instead of interacting with removed hardware.

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M262: Configure IO Expander pin <a href="https://reprap.org/wiki/G-code#M262:_Configure_IO_Expander_pin">M262: Configure IO Expander pin</a>
 *
 * Only MK3.5/S, MK3.9/S, MK4/S
 *
 *#### Usage
 *
 *    M262 [ P | B ]
 *
 *#### Parameters
 *
 * - `P` - Single pin <0;7> to be configured.
 * - `B` - Pin mode
 *   - `0` - As output pin
 *   - `1` - As input pin
 *
 *#### Examples
 *
 *    M262 P0 B0 ; Set pin0 as Output pin (0)
 *
 */
void PrusaGcodeSuite::M262() {
    SERIAL_ECHOLNPGM("M262 disabled: IO expander not present.");
}

/**
 *### M263: Read IO Expander pin <a href="https://reprap.org/wiki/G-code#M263:_Read_IO_Expander_pin">M263: Read IO Expander pin</a>
 *
 * Only MK3.5/S, MK3.9/S, MK4/S
 *
 *#### Usage
 *
 *    M263 [ P ]
 *
 *#### Parameters
 *
 * - `P` - Single pin to read <0;7>.
 *
 *#### Examples
 *
 *    M263 P6 ; read only from pin6, received value will be HIGH (binary 0010 0000 => dec 32) or LOW (binary 0000 0000 => dec 0)
 *    M263    ; read whole Input register (byte)
 *
 */
void PrusaGcodeSuite::M263() {
    SERIAL_ECHOLNPGM("M263 disabled: IO expander not present.");
}

/**
 *### M264: Write IO Expander pin <a href="https://reprap.org/wiki/G-code#M264:_Write_IO_Expander_pin">M264: Write IO Expander pin</a>
 *
 * Only MK3.5/S, MK3.9/S, MK4/S
 *
 *#### Usage
 *
 *    M264 [ P | B ]
 *
 *
 *#### Parameters
 *
 * - `P` - Select single pin to write to <0;7>.
 * - `B` - Set pin
 *   - `1` - Set HIGH
 *   - `0` - Set LOW
 *
 *#### Examples
 *
 *    M264 P0 B1 ; Set output pin0 to HIGH (1)
 *    M264 P7 B0 ; Set output pin7 to LOW (0)
 *
 */
void PrusaGcodeSuite::M264() {
    SERIAL_ECHOLNPGM("M264 disabled: IO expander not present.");
}

/**
 *### M265: Toggle IO Expander output pin <a href="https://reprap.org/wiki/G-code#M265:_Toggle_IO_Expander_output_pin">M265: Toggle IO Expander output pin</a>
 *
 * Only MK3.5/S, MK3.9/S, MK4/S
 *
 *#### Usage
 *
 *    M265 [ P ]
 *
 *
 * This G-Code doesn't check if selected pin is configured as Output pin.
 *
 *#### Parameters
 *
 *  - `P` - Select single pin to flip <0;7>.
 *
 *#### Examples
 *
 *    M264 P0 ; Flip pin0
 *
 */
void PrusaGcodeSuite::M265() {
    SERIAL_ECHOLNPGM("M265 disabled: IO expander not present.");
}

/**
 *### M267: Write IO Expander register <a href="https://reprap.org/wiki/G-code#M267:_Write_IO_Expander_register">M267: Write IO Expander register</a>
 *
 * This overwrites whole byte. Configuration and Output registers are saved into persistent memory.
 *
 * Only MK3.5/S, MK3.9/S, MK4/S
 *
 *#### Usage
 *
 *    M [ R | B ]
 *
 *#### Parameters
 *
 *  - `R` - Register
 *      - Output = 1,
 *      - Polarity = 2,
 *      - Config = 3
 *
 *  - `B` - [value] to be set
 *
 *#### Examples
 *
 *    M267 R3 B255 ; Set up Config register to the value 255dec (1111 1111b)
 *
 */
void PrusaGcodeSuite::M267() {
    SERIAL_ECHOLNPGM("M267 disabled: IO expander not present.");
}

/**
 *### M268: Read IO expander register <a href="https://reprap.org/wiki/G-code#M268:_Read_IO_expander_register">M268: Read IO expander register</a>
 *
 * Only MK3.5/S, MK3.9/S, MK4/S
 *
 *#### Usage
 *
 *    M268 [ R ]
 *

 *#### Parameters
 *
 * - `R` - Register
 *   - `0` - Input register
 *   - `1` - Output register
 *   - `2` - Polarity register
 *   - `3` - Config register
 *
 *#### Examples
 *
 *    M268 R3 ; Read Config register
 *
 */
void PrusaGcodeSuite::M268() {
    SERIAL_ECHOLNPGM("M268 disabled: IO expander not present.");
}

#endif // HAS_I2C_EXPANDER()

/** @}*/
