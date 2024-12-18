/* File for the accelerometer MMA8451Q function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef MMA8451_H
#define MMA8451_H

// MMA8451 MACROS -------------------------------------------------------------------------------
#define MMA8451_I2C_ADDRESS (0x1D << 1)                           // MMA8451Q I2C Address: 7-bit address shifted because of mBed 8-bit format
#define CTRL_REG1 0x2A                                            // MMA8451Q Control Register 1 to allow writing                                          // Interrupt pin routing register
#define OUT_X_MSB 0x01                                            // Register for X-axis MSB
#define OUT_Y_MSB 0x03                                            // Register for Y-axis MSB
#define OUT_Z_MSB 0x05                                            // Register for Z-axis MSB

// ==============================================================================================
// MMA8451Q CLASS
// ==============================================================================================
class MMA8451Q {
public:
    // Constructor ------------------------------------------------------------------------------
    MMA8451Q(I2C& i2c_bus);

    // Public functions -------------------------------------------------------------------------
    void init_mma8451();                                          // Function to initialize the accelerometer
    int16_t read_axis(char msb_reg);                              // Function to read a 14-bit axis value (X, Y, Z)

private:
    // Private functions ------------------------------------------------------------------------
    void write_register_mma8451(char reg, char value);
    char read_register_mma8451(char reg);

    // Reference to the I2C bus -----------------------------------------------------------------
    I2C& _i2c;
};
// MMA8451Q CLASS END ===========================================================================

#endif