#include "battery.h"

bool checkBattery(unsigned long now) {
  if(state.lastButtonState) {
    return false;
  }

  unsigned int scaledBatmVolts = sampleAdc(VBAT);
  
  state.batteryVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  if(state.currentDisplayMode != 1 && state.batteryVoltage <= REMOTE_BATTERY_CELL_V_THR[BATTERY_THRESHOLDS_LENGTH-1]) {
     changeMode(1);
  }
  return true;
}