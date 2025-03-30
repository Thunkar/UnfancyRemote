#pragma once

#include <ESP32Servo.h> 
#include "board.h"
#include "state.h"

extern Servo PPM_OUTPUT;

TaskResult writePPMValue(unsigned long now);