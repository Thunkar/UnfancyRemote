#include "LED.h"

const int LEDS_LENGTH = 4;
CRGB rainbow[] = { CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green };

CRGB LEDColor[] = { CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black };
CRGB storedLEDColor[] = { CRGB::White, CRGB::White, CRGB::White, CRGB::White };
long LEDPeriods[] = { -1, -1, -1, -1 };
int LEDResetCounters[] = { -1, -1, -1, -1 };
unsigned long lastLEDToggled[] = { 0, 0, 0, 0 };
int sequenceSpeed = 3;
bool inSequence = false;

void changeLEDColor(int LEDn, CRGB color) {
  storedLEDColor[LEDn] = color;
}

void setLEDOn(int LEDn) {
  LEDPeriods[LEDn] = 0;
  LEDResetCounters[LEDn] = -1;
}

void setLEDOff(int LEDn) {
  LEDPeriods[LEDn] = -1;
  LEDResetCounters[LEDn] = -1;
}

void flashLED(int LEDn, int times, unsigned long period, CRGB resetStatus) {
  LEDPeriods[LEDn] = period;
  LEDResetCounters[LEDn] = times;
  storedLEDColor[LEDn] = resetStatus;
}

void sequence() {
  if(LEDPeriods[0] == -2) {
    return;
  }
  for(int i = 0; i < LEDS_LENGTH; i++) {
    LEDPeriods[i] = -2;
    LEDResetCounters[i] = 0;
  }
  LEDResetCounters[0] = sequenceSpeed;
}

bool setLEDs(unsigned long now) {
  for(int i = 0; i < LEDS_LENGTH; i++) {
    if (LEDPeriods[i] == -1) {
      LEDColor[i] = CRGB::Black;
    } else if(LEDPeriods[i] == 0) {
      LEDColor[i] = storedLEDColor[i];
    } else if (LEDPeriods[i] == -2) {
      if(LEDResetCounters[i] > 0) {
        LEDResetCounters[i]--;
        LEDColor[i] = storedLEDColor[i];
      } else if (LEDColor[i] != CRGB::Black) {
        int nextIndex = i+1 >= LEDS_LENGTH ? 0 : i+1;
        LEDResetCounters[nextIndex] = sequenceSpeed;
        LEDColor[i] = CRGB::Black;
      }
    } else if(now - lastLEDToggled[i] >= LEDPeriods[i]) {
      int isBlinking = LEDResetCounters[i] != 0;
      if(isBlinking) {
        LEDColor[i] = LEDColor[i] == CRGB::Black ? storedLEDColor[i] : CRGB::Black;
        if(LEDColor[i] != CRGB::Black && LEDResetCounters[i] > 0) {
          LEDResetCounters[i]--;
        }
        lastLEDToggled[i] = now;
      } else {
        setLEDOn(i);
      }
    } else {
      storedLEDColor[i] != CRGB::Black ? setLEDOn(i) : setLEDOff(i);
    }
  }
  FastLED.show();
  return true;
}