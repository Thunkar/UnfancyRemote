#pragma once

#include <Arduino.h>
#include "board.h"

void pulseMotor(int times, unsigned long period);
bool setMotor(unsigned long now);