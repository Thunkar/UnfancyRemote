#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "board.h"

struct Config {
    unsigned int channel;
    unsigned int identity; 
    unsigned int cellN;
    unsigned long frequency;
};

extern Config config;

void readConfig();
void writeConfig();
void setupEEPROM();