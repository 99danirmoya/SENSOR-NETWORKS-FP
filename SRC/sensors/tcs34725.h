/* File for the colour sensor TCS34725 function declarations and macros */

// LIBRARIES ------------------------------------------------------------------------------------
#include "mbed.h"

// LIBRARY GUARD --------------------------------------------------------------------------------
#ifndef TCS34725_H
#define TCS34735_H

// MACROS ---------------------------------------------------------------------------------------
#define LED_PIN PH_1                                                        // White LED connected to PA_5 (adjust if necessary)
#define TCS34725_ADDRESS (0x29 << 1)                                        // 7-bit I2C address shifted
#define TCS34725_COMMAND_BIT 0x80                                           // Indicate that the following byte will be a command
#define TCS34725_ENABLE 0x00                                                // Enables states and interrupts
#define TCS34725_ATIME 0x01                                                 // RGBC time 
#define TCS34725_ENABLE_AEN 0x02                                            // ADC enable
#define TCS34725_AGAIN 0x0F                                                 // Gain control
#define TCS34725_CDATAL 0x14                                                // Clear data low byte
#define TCS34725_RDATAL 0x16                                                // Red data low byte
#define TCS34725_GDATAL 0x18                                                // Green data low byte
#define TCS34725_BDATAL 0x1A                                                // Blue data low byte

// ==============================================================================================
// TCS34725 CLASS
// ==============================================================================================
class TCS34725 {
public:
    // Constructor ------------------------------------------------------------------------------
    TCS34725(I2C& i2c_bus);

    // Public functions -------------------------------------------------------------------------
    void tcs34725_init();                                          // Function to initialize the accelerometer
    uint16_t read_channel(uint8_t reg);                              // Function to read a 14-bit axis value (X, Y, Z)

private:
    // Private functions ------------------------------------------------------------------------
    void write_register(uint8_t reg, uint8_t value);

    // Reference to the I2C bus -----------------------------------------------------------------
    I2C& _i2c;
};
// TCS34725 CLASS END ===========================================================================


#endif