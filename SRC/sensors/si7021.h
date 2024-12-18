/* File for the ambient sensor Si7021 function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef SI7021_H
#define SI7021_H

// Si7021 MACROS --------------------------------------------------------------------------------
#define SI7021_ADDR 0x40 << 1                               // Si7021 I2C Address: 7-bit I2C address SHIFTED BY 1 BIT
#define CMD_MEASURE_HUMIDITY 0xE5                           // Si7021 Command: Measure Relative Humidity, Hold Master Mode
#define CMD_MEASURE_TEMP 0xE3                               // Si7021 Command: Measure Temperature, Hold Master Mode

// ==============================================================================================
// Si7021 CLASS
// ==============================================================================================
class Si7021 {
public:
    // Constructor ------------------------------------------------------------------------------
    Si7021(I2C& i2c_bus);

    // Public functions -------------------------------------------------------------------------
    uint16_t read_register_si7021(char command);            // Function to read a 16-bit register

private:
    // Reference to the I2C bus -----------------------------------------------------------------
    I2C& _i2c;
};
// Si7021 CLASS END =============================================================================

#endif