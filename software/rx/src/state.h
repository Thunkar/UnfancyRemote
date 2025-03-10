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
    unsigned long packets;
    unsigned long TMPackets;
    unsigned long waitingForRF;
    unsigned long RFWaits;
    // Errors 
    bool error;
    int errors;
    char errorReason[30];
};

extern State state;