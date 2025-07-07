#include "battery.h"

TaskResult checkBattery(unsigned long now) {
  unsigned int scaledBatmVolts = sampleAdc(VBAT);
  
  state.boardVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  return { true };
}