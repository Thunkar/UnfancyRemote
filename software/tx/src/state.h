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
    long currentRSSI;
    int currentSNR;
    unsigned long packets;
    unsigned long TMPackets;
    volatile int interruptCounter;
    // Errors 
    bool error;
    int errors;
    char errorReason[30];
};

extern State state;