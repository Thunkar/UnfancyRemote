#pragma once

#include <Arduino.h>
#include "board.h"
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
    unsigned long packetTimes;
    unsigned long maxPacketTime;
    unsigned long minPacketTime;
    unsigned long TMPackets;
    int maxSNR;
    int minSNR;
    int RSSI;
    int SNR;
    unsigned long errors[ERROR_TYPES];
};

struct ComputedStats {
    unsigned long meanPacketTime;
    int packetsPerSecond;
    int TMPacketsPerSecond;
    float taskFrequencies[N_TASKS];
    float taskMeanTimes[N_TASKS];
    float taskRatios[N_TASKS];
    float loopFrequency;
    float errorsPerSecond[ERROR_TYPES];
};

extern Stats stats;
extern ComputedStats computedStats;

TaskResult printStats(unsigned long now);

void setError(ERROR_CODE code);
char* getReason(ERROR_CODE code);