#pragma once
//#define DUAL_THROTTLE <- Uncomment this line if you're using a dual trigger case with 2 sensors.

// In case a dual trigger remote is used, the following line adjusts the sensitivity of the brakes, or in other words, when does the brake lever overrules the throttle. It is NOT recommended to lower it, but it can be raised to prevent accidental brake activation

#ifdef DUAL_THROTTLE
const float BRAKE_SENSITIVITY = 5; 
#endif

#define CH_BANDWIDTH_HZ 2500000
#define BASE_FREQUENCY 2400000000

// Task periods in ms

const unsigned long periods[] = { 20, 1, 10, 100, 1000, 50, 10, 10, 2000, 20 };

// Pairing: Choose a channel (0-40) and a transmitter identity (0-254) to prevent collisions with nearby remotes or other LoRa devices

const unsigned int channel = 15;
const unsigned int TXIdentity = 224; 

const unsigned long frequency = channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;

// Time in ms to hold the button to enter setup mode

const int setupModeDelay = 5000; 

// Battery thresholds: Define what each LED of the remote means in terms of cell voltage, both for the remote and the board. First level is 4 LEDs ON, second one 3, etc. The last level will make the remote rumble and the last LED blink. WARNING: a maximum board voltage of 12S (50.4V) is measurable.
const int BATTERY_THRESHOLDS_LENGTH = 5;

const float REMOTE_BATTERY_CELL_V_THR[] = { 4.0, 3.9, 3.8, 3.7, 3.6 };

const float BOARD_BATTERY_CELL_V_THR[] = { 4.0, 3.8, 3.7, 3.5, 3.3 };
const unsigned int BOARD_CELL_S = 12;