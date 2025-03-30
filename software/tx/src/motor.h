#pragma once

#include <Arduino.h>
#include "board.h"
#include "scheduler.h"

void pulseMotor(int times, unsigned long period);

TaskResult setMotor(unsigned long now);