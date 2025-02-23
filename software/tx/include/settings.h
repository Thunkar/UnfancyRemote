#pragma once

#define CH_BANDWIDTH_HZ 2500000
#define BASE_FREQUENCY 2400000000

// Task periods in ms

const unsigned long periods[] = { 10, 1, 20, 200, 1000, 100, 50, 20, 2000, 50 };

// Time in ms to hold the button to enter setup mode

const int setupModeDelay = 5000; 

// Battery thresholds: Define what each LED of the remote means in terms of cell voltage, both for the remote and the board. First level is 4 LEDs ON, second one 3, etc. The last level will make the remote rumble and the last LED blink. WARNING: a maximum board voltage of 12S (50.4V) is measurable.
const int BATTERY_THRESHOLDS_LENGTH = 5;

const float REMOTE_BATTERY_CELL_V_THR[] = { 4.0, 3.9, 3.8, 3.7, 3.6 };

const float BOARD_BATTERY_CELL_V_THR[] = { 4.0, 3.8, 3.7, 3.5, 3.3 };
