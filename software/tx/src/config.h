#pragma once

#include <Arduino.h>
#include <EEPROM.h>

struct Config {
    unsigned int calBrake;
    unsigned int calAcc;
    unsigned int centerAcc;
    unsigned int centerBrake;
    unsigned int inverted;
};

extern Config config;

void readConfig();
void writeConfig();