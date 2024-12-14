/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cstdint>
#include <cstdio>

#include "mbed_version.h"
#include "mbed.h"

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "trace_helper.h"
#include "lora_radio_helper.h"

// Self-crafted libraries
#include "gps_thread.h"
#include "mma8451.h"
#include "si7021.h"
#include "tcs34725.h"
#include "soilmoisture.h"
#include "phototrans.h"

#define RGB_RED_PIN        PH_0                                                  // Pin connected to the RGB red
#define RGB_GREEN_PIN      PA_14                                                 // Pin connected to the RGB green
#define RGB_BLUE_PIN       PA_13                                                 // Pin connected to the RGB blue

using namespace events;
using namespace std::chrono_literals;

/*
 * Thread for the measurements of the GPS
 */
static Thread gps_th(osPriorityNormal, 512);

extern uint8_t fix_status;
extern float latitude, longitude;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
constexpr size_t TX_BUFFER_SIZE = 30;
uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        20s

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Pin for sensors
 */
#define SDA_PIN PB_9
#define SCL_PIN PB_8
#define LED_PIN PH_1                                                        // White LED connected to PA_5 (adjust if necessary)
#define MOISTURE_PIN PA_0
#define PHTRANS_PIN PA_4

/**
 * Sensor class object
 */
I2C i2c(SDA_PIN, SCL_PIN);                                      // I2C communication
static BusOut myRGB(RGB_RED_PIN, RGB_GREEN_PIN, RGB_BLUE_PIN);  // BusOut to control the RGB LED with just an object
static DigitalOut whiteLED(LED_PIN);                            // DigitalOut for builtin white LED control
static AnalogIn moistureIn(MOISTURE_PIN);                       // Analog pin corresponding to Arduino's A0
static AnalogIn lightIn(PHTRANS_PIN);                           // Analog pin corresponding to Arduino's A2

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Default and configured device EUI, application EUI and application key
 */
static const uint8_t DEFAULT_DEV_EUI[] = {0x40, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t DEV_EUI[] = {0x86, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = {0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xac, 0x4a};
static uint8_t APP_KEY[] = {0x86, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94,
                            0x86, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};

/**
 * Entry point for application
 */
int main(void){
    // Start GPS thread
    gps_th.start(gps_th_routine); 

    // setup sensors
    init_mma8451();
    tcs34725_init();                                         // Initialize the TCS34725 sensor

    // start RGB OFF
    myRGB = 0b111;                                                               // Ensure RGB LED is OFF

    printf("\r\n*** Sensor Networks @ ETSIST, UPM ***\r\n"
           "   Mbed (v%d.%d.%d) LoRaWAN example\r\n",
           MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    printf("\r\n DEV_EUI: ");
    for (int i = 0; i < sizeof(DEV_EUI); ++i) printf("%02x", DEV_EUI[i]);
    printf("\r\n APP_EUI: ");
    for (int i = 0; i < sizeof(APP_EUI); ++i) printf("%02x", APP_EUI[i]);
    printf("\r\n APP_KEY: ");
    for (int i = 0; i < sizeof(APP_KEY); ++i) printf("%02x", APP_KEY[i]);
    printf("\r\n");

    if (!memcmp(DEV_EUI, DEFAULT_DEV_EUI, sizeof(DEV_EUI))) {
        printf("\r\n *** You are using the default device EUI value!!! *** \r\n");
        printf("Please, change it to ensure that the device EUI is unique \r\n");
        return -1;
    }

    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;

    retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/**
 * Sends a message to the Network Server
 */
static void send_message(){
    uint16_t packet_len;
    int16_t retcode;
    
    int16_t raw_ax, raw_ay, raw_az;
    uint16_t raw_clear, raw_red, raw_green, raw_blue, raw_temperature, raw_humidity, raw_soilMoist, raw_light;

    size_t pos = 0;

    // Accelometer MMA8451 raw measurements - 14 bit -----------------------------------------------------
    raw_ax = read_axis(OUT_X_MSB);                              // Remember, only the MSB is passed, but the function adds one to the register to get the LSB part
    raw_ay = read_axis(OUT_Y_MSB);
    raw_az = read_axis(OUT_Z_MSB);
    
    // Si7021 raw measurements - 16 bit
    raw_temperature = read_register_si7021(CMD_MEASURE_TEMP);
    raw_humidity = read_register_si7021(CMD_MEASURE_HUMIDITY);

    // Soil moisture measurements - 12 bit -----------------------------------------------------------
    raw_soilMoist = moistureIn.read_u16();                      // Read the analog value of soil moisture
    
    // Ambient light measurements - 12 bit -----------------------------------------------------------
    raw_light = lightIn.read_u16();                             // Read the analog value of ambient light
    
    // Colour sensor TCS34725 measurements --------------------------------------------------
    whiteLED = 1;                                               // Turn on the white LED before taking a measurement
    ThisThread::sleep_for(30ms);                                // Wait for the integration time (24ms) + small extra time for stable readings

    raw_clear = read_channel(TCS34725_CDATAL);
    raw_red   = read_channel(TCS34725_RDATAL);
    raw_green = read_channel(TCS34725_GDATAL);
    raw_blue  = read_channel(TCS34725_BDATAL);

    whiteLED = 0;                                               // Turn off the white LED after the measurement

    if(fix_status == 0){
        latitude = 43.563644;
        longitude = -5.937019;
    }

    uint32_t lat_u32 = *(uint32_t *) &latitude;
    uint32_t lon_u32 = *(uint32_t *) &longitude;

    tx_buffer[pos++] = raw_ax & 0xff;                           // In LUA -> payload, 1
    tx_buffer[pos++] = (raw_ax >> 8) & 0xff;                    // In LUA -> payload, 2
    tx_buffer[pos++] = raw_ay & 0xff;
    tx_buffer[pos++] = (raw_ay >> 8) & 0xff;
    tx_buffer[pos++] = raw_az & 0xff;
    tx_buffer[pos++] = (raw_az >> 8) & 0xff;

    tx_buffer[pos++] = raw_temperature & 0xff;
    tx_buffer[pos++] = (raw_temperature >> 8) & 0xff;
    tx_buffer[pos++] = raw_humidity & 0xff;
    tx_buffer[pos++] = (raw_humidity >> 8) & 0xff;
    
    tx_buffer[pos++] = raw_soilMoist & 0xff;
    tx_buffer[pos++] = (raw_soilMoist >> 8) & 0xff;
    tx_buffer[pos++] = raw_light & 0xff;
    tx_buffer[pos++] = (raw_light >> 8) & 0xff;

    tx_buffer[pos++] = raw_clear & 0xff;
    tx_buffer[pos++] = (raw_clear >> 8) & 0xff;
    tx_buffer[pos++] = raw_red & 0xff;
    tx_buffer[pos++] = (raw_red >> 8) & 0xff;
    tx_buffer[pos++] = raw_green & 0xff;
    tx_buffer[pos++] = (raw_green >> 8) & 0xff;
    tx_buffer[pos++] = raw_blue & 0xff;
    tx_buffer[pos++] = (raw_blue >> 8) & 0xff;

    tx_buffer[pos++] = lat_u32 & 0xff;
    tx_buffer[pos++] = (lat_u32 >> 8) & 0xff;
    tx_buffer[pos++] = (lat_u32 >> 16) & 0xff;
    tx_buffer[pos++] = (lat_u32 >> 24) & 0xff;

    tx_buffer[pos++] = lon_u32 & 0xff;
    tx_buffer[pos++] = (lon_u32 >> 8) & 0xff;
    tx_buffer[pos++] = (lon_u32 >> 16) & 0xff;
    tx_buffer[pos++] = (lon_u32 >> 24) & 0xff;

    printf("Ax: %d, Ay: %d, Az: %d\n\r", raw_ax, raw_ay, raw_az);
    printf("T: %d, RH: %d\n\r", raw_temperature, raw_humidity);
    printf("Moisture: %d, light = %d\n\r", raw_soilMoist, raw_light);
    printf("C: %d, R: %d, G: %d, B: %d\n\r", raw_clear, raw_red, raw_green, raw_blue);
    printf("FS: %d, Lat: %.6f, Lon: %.6f\n\r", fix_status, latitude, longitude);

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, pos,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3s, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");

    // Check for specific messages
    if (retcode == 3 && rx_buffer[0] == 'O' && rx_buffer[1] == 'F' && rx_buffer[2] == 'F') {
        myRGB = 0b111;
        printf("OFF received\r\n");
    } else if (retcode == 5 && rx_buffer[0] == 'G' && rx_buffer[1] == 'r' && rx_buffer[2] == 'e' && rx_buffer[3] == 'e' && rx_buffer[4] == 'n') {
        myRGB = 0b101;
        printf("Green received\r\n");
    } else if (retcode == 3 && rx_buffer[0] == 'R' && rx_buffer[1] == 'e' && rx_buffer[2] == 'd') {
        myRGB = 0b110;
        printf("Red received\r\n");
    }
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
