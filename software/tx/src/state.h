#pragma once

#include <Arduino.h>
#include "board.h"

struct State {
    // Throttle value
    unsigned int encodedThrottleValue;
    unsigned int rawThrottle1Value;
    unsigned int rawThrottle2Value;
    // Button state
    int lastButtonState;  
    // Modes
    int currentDisplayMode; // -1 transition, 0 board voltage/connection status, 1 remote voltage
    int lastDisplayMode;
    unsigned long lastModeTransition;
    bool canChangeMode;
    bool setupMode;
    // Battery voltages
    float batteryVoltage;
    float boardVoltage;
    float boardCellVoltage;
    // RF
    bool isConnected;
    // Next time tasks should be run in us
    unsigned long nextRun[N_TASKS];
    // Active tasks
    bool activeTasks[N_TASKS];
    // Error
    bool error;
};

extern State state;