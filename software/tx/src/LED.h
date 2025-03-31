#pragma once

#include "FastLED.h"
#include "scheduler.h"

#define LEDS_LENGTH 4
#define SEQUENCE_SPEED 3

void changeLEDColor(int LEDn, CRGB color);

void setLEDOn(int LEDn);

void setLEDOff(int LEDn);

void flashLED(int LEDn, int times, unsigned long period, CRGB resetStatus);

TaskResult setLEDs(unsigned long now);
void setupLEDs();

void sequence();

extern CRGB LEDColor[];
extern CRGB rainbow[];