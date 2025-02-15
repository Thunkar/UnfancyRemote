#pragma once

#include <Arduino.h>
#include <EEPROM.h>

struct Config {
    unsigned int channel;
    unsigned int TXIdentity; 
    unsigned int nCells;
    unsigned int calBrake;
    unsigned int calAcc;
    unsigned int centerAcc;
    unsigned int centerBrake;
    unsigned int inverted;
    unsigned int isDual;
};

extern Config config;

void readConfig();
void writeConfig();