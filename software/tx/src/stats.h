#pragma once

#include <Arduino.h>
#include "board.h"
#include "scheduler.h"
#include "config.h"
#include "state.h"

const int ERROR_TYPES = 5;

enum ERROR_CODE {
    IRQ_ERROR,
    INCORRECT_IDENTITY,
    TX_TIMEOUT,
    RX_TIMEOUT,
    DISCONNECTED
};

struct Stats {
    unsigned long successes[N_TASKS];
    unsigned long failures[N_TASKS];
    unsigned long times[N_TASKS];
    unsigned long maxTimes[N_TASKS];
    unsigned long minTimes[N_TASKS];
    unsigned long loops;
    unsigned long packets;
    unsigned long TMPackets;
    unsigned long timeWaitingForTX;
    unsigned long timeWaitingForRX;
    unsigned long TXWaits;
    unsigned long RXWaits;
    unsigned long errors[ERROR_TYPES];
};

struct ComputedStats {
    int packetsPerSecond;
    int TMPacketsPerSecond;
    float taskFrequencies[N_TASKS];
    float taskMeanTimes[N_TASKS];
    float taskRatios[N_TASKS];
    float loopFrequency;
    float RXWaitMean;
    float RXWaitsPerSecond;
    float TXWaitMean;
    float TXWaitsPerSecond;
    float meanRXOffsets;
    float errorsPerSecond[ERROR_TYPES];
};

extern Stats stats;

TaskResult printStats(unsigned long now);

void setError(ERROR_CODE code);
char* getReason(ERROR_CODE code);