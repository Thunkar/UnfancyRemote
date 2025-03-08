#include "battery.h"

bool checkBattery(unsigned long now) {
  if(state.lastButtonState) {
    return false;
  }

  int scaledBatmVolts = sampleAdc(VBAT);
  
  state.boardVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  return true;
}