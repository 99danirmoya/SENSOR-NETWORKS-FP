/* File for the ambient sensor Si7021 function definitions */

// LIBRARIES ---------------------------------------------------------------------------------------------------------------
#include "mbed.h"
#include "si7021.h"

// CONSTRUCTOR -------------------------------------------------------------------------------------------------------------
Si7021::Si7021(I2C& i2c_bus) : _i2c(i2c_bus) {}

// FUNCTION TO READ 16-BIT DATA FROM SENSOR Si7021 =========================================================================
uint16_t Si7021::read_register_si7021(char command) {
    char data[2];                                           // Data buffer of 16-bit size
    _i2c.write(SI7021_ADDR, &command, 1);                    // Send command to start measurement
    _i2c.read(SI7021_ADDR, data, 2);                         // Read the data after waiting

    return (data[0] << 8) | data[1];                        // Combine the two bytes (0 is the value if the reading is not successful)
}