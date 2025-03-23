#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "board.h"

struct Config {
    unsigned int channel;
    unsigned int identity; 
    unsigned int cellN;
    unsigned int calBrake;
    unsigned int calAcc;
    unsigned int centerAcc;
    unsigned int centerBrake;
    unsigned int inverted;
    unsigned int isDual;
    unsigned long frequency;
};

extern Config config;

void readConfig();
void readCalibration();
void writeConfig();
void writeCalibration();