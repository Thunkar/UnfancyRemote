#pragma once

#include <Arduino.h>
#include "board.h"

unsigned int sampleAdc(const int pin);
unsigned int roundAndCastToInt(float var);