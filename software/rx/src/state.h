#pragma once

#include <Arduino.h>
#include "board.h"

struct State {
    // Throttle value
    unsigned int encodedThrottleValue;
    // Button state
    int lastButtonState;  
    // Mode
    bool setupMode;
    // Battery voltage
    float boardVoltage;
    // RF
    bool isConnected;
    long currentRSSI;
    int currentSNR;
    // Last time tasks were run in ms
    unsigned long lastRun[N_TASKS];
    // Active tasks
    bool activeTasks[N_TASKS];
    // Error
    bool error;
};

extern State state;