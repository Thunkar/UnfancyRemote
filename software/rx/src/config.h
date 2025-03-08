#pragma once

#include <Arduino.h>
#include <EEPROM.h>

struct Config {
    unsigned int channel;
    unsigned int identity; 
    unsigned int cellN;
};

extern Config config;

void readConfig();
void writeConfig();