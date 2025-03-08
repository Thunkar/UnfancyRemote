#pragma once

#define CH_BANDWIDTH_HZ 2500000
#define BASE_FREQUENCY 2400000000

// Task periods in ms

const unsigned long periods[] = { 1, 20, 1, 1000, 2000 };

// Time in ms to hold the button to enter setup mode

const int setupModeDelay = 5000; 
