#pragma once

#include <Arduino.h>
#include "board.h"
#include "config.h"
#include "state.h"

struct Stats {
    unsigned long successes[N_TASKS];
    unsigned long failures[N_TASKS];
    unsigned long times[N_TASKS];
    unsigned long maxTimes[N_TASKS];
    unsigned long minTimes[N_TASKS];
    unsigned long loops;
    unsigned long packets;
    unsigned long packetTimes;
    unsigned long maxPacketTime;
    unsigned long minPacketTime;
    unsigned long TMPackets;
    unsigned long timeWaitingForTX;
    unsigned long timeWaitingForRX;
    unsigned long TXWaits;
    unsigned long RXWaits;
    long rxOffsets; 
    int errors;
    char errorReason[30];
};

extern Stats stats;

TaskResult printStats(unsigned long now);