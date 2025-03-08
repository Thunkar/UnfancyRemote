#include "battery.h"

bool checkBattery(unsigned long now) {
  if(state.lastButtonState) {
    return false;
  }

  int scaledBatmVolts = analogReadMilliVolts(VBAT);
  
  float newBatteryVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  state.boardVoltage = state.boardVoltage != -1 ? (newBatteryVoltage + state.boardVoltage) / 2 : newBatteryVoltage;
  return true;
}