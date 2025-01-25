#include "FastLED.h"

void changeLEDColor(int LEDn, CRGB color);

void setLEDOn(int LEDn);

void setLEDOff(int LEDn);

void flashLED(int LEDn, int times, unsigned long period, CRGB resetStatus);

bool setLEDs(unsigned long now);

void sequence();


extern CRGB LEDColor[];
extern const int LEDS_LENGTH;