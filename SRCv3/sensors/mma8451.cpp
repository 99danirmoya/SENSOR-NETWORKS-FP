/* File for the accelerometer MMA8451Q function definitions */

// LIBRARIES ------------------------------------------------------------------------------------------------------------
#include "mbed.h"
#include "mma8451.h"

// CONSTRUCTORS ---------------------------------------------------------------------------------------------------------
MMA8451Q::MMA8451Q(I2C& i2c_bus) : _i2c(i2c_bus) {}                                                 // I2C communication

// FUNCTION TO WRITE TO REGISTER ========================================================================================
void MMA8451Q::write_register_mma8451(char reg, char value){         // WRITE function receives the Control Register 1 and the ax direction
    char data[2] = {reg, value};
    _i2c.write(MMA8451_I2C_ADDRESS, data, 2);                      // We write in the MMA8451 direction the command to get, for example, ax
}

// FUNCTION TO READ A REGISTER ==========================================================================================
char MMA8451Q::read_register_mma8451(char reg){                      // READ function does only get as a parameter the register to be read
    char data;
    _i2c.write(MMA8451_I2C_ADDRESS, &reg, 1, true);                // Send register address from, for example, ax
    _i2c.read(MMA8451_I2C_ADDRESS, &data, 1);                      // Read data returned from the register
    return data;
}

// FUNCTION TO READ 14-BIT AXIS VALUE (X, Y, Z) =========================================================================
int16_t MMA8451Q::read_axis(char msb_reg){                                  // This function receives as a parameter only the MSB acceleration register
    char msb = read_register_mma8451(msb_reg);                    // The MSB register for each component is read
    char lsb = read_register_mma8451(msb_reg + 1);                // The same with the LSB register which is actually 6 bit long, having bits 0 and 1, the most significant, empty
    return (int16_t)((msb << 8) | lsb) >> 2;                      // Combine MSB (8-bit) and LSB (6-bit), and shift by 2 for 14-bit value
}

// FUNCTION TO INITIALIZE THE ACCELEROMETER WITH FREEFALL DETECTION =====================================================
void MMA8451Q::init_mma8451() {
    char data = read_register_mma8451(CTRL_REG1);                 // Initialize the MMA8451Q accelerometer by setting it to active mode, read the state of Control Register 1 (0x2A)
    write_register_mma8451(CTRL_REG1, data | 0x01);
}