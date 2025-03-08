#pragma once

#include <ESP32Servo.h> 
#include "state.h"

extern Servo PPM;

bool writePPMValue(unsigned long now);