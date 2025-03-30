#include "battery.h"

TaskResult checkBattery(unsigned long now) {
  if(state.lastButtonState) {
    return { false, 0 };
  }

  int scaledBatmVolts = sampleAdc(VBAT);
  
  state.boardVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  return { true, 0 };
}