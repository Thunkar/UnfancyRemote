#include "utils.h"

unsigned int sampleAdc(const int pin) {
  unsigned int scaledmVolts = 0;

  for (int i = 0; i < ADC_SAMPLES; i++) {
    scaledmVolts+=analogReadMilliVolts(pin);
  }

  return (unsigned int)((float)scaledmVolts/(float)ADC_SAMPLES);
}

void scheduleImmediate(const int task) {
  lastRun[task] = 0;
}