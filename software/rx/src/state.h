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
    // Next time tasks should be run in ns
    unsigned long nextRun[N_TASKS];
    // Active tasks
    bool activeTasks[N_TASKS];
};

extern State state;