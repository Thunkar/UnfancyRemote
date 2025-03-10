#pragma once

#include <ESP32Servo.h> 
#include "state.h"

extern Servo PPM_OUTPUT;

bool writePPMValue(unsigned long now);