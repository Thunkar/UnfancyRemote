#pragma once

// Log to serial
#define DEBUG

// RF params

#define CH_BANDWIDTH_HZ 2500000
#define BASE_FREQUENCY 2400000000

// HW Pins

const int NSS = 7;                           //select on LoRa device
const int NRESET = 9;                        //reset on LoRa device
const int RFBUSY = 10;                        //RF busy on LoRa device 
const int DIO1 = -1;                          //DIO1 on LoRa device, used for RX and TX done
const int DIO2 = -1;                         //DIO2 on LoRa device, normally not used so set to -1
const int DIO3 = -1;                         //DIO3 on LoRa device, normally not used so set to -1
const int RX_EN = -1;                        //pin for RX enable, used on some SX1280 devices, set to -1 if not used
const int TX_EN = -1;                        //pin for TX enable, used on some SX1280 devices, set to -1 if not used

const int ON = 21;
const int BUTTON = 3;
const int THR1 = 0;
const int LED = 8;
const int MOTOR = 20;
const int VBAT = 2;
const int THR2 = 1;

// General constants

const unsigned long R2 = 6800;
const unsigned long R1 = 100000;

const unsigned int ENCODED_MAX = 4095;
const unsigned int ENCODED_HALF = 2048;

const unsigned int ADC_SAMPLES = 5;

// Time in us to hold the button to enter setup mode

const int SETUP_MODE_DELAY = 5000 * 1e3; 

// Scheduler

const int N_TASKS = 12;
struct TaskResult {
    bool success;
};

// Battery thresholds: Define what each LED of the remote means in terms of cell voltage, both for the remote and the board. First level is 4 LEDs ON, second one 3, etc. The last level will make the remote rumble and the last LED blink. WARNING: a maximum board voltage of 12S (50.4V) is measurable.
const int BATTERY_THRESHOLDS_LENGTH = 5;
const float REMOTE_BATTERY_CELL_V_THR[] = { 4.0, 3.9, 3.8, 3.7, 3.6 };
const float BOARD_BATTERY_CELL_V_THR[] = { 4.0, 3.8, 3.7, 3.5, 3.3 };
