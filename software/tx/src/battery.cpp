#include "battery.h"

bool checkBattery(unsigned long now) {
  if(state.lastButtonState) {
    return false;
  }

  int scaledBatmVolts = analogReadMilliVolts(VBAT);
  
  float newBatteryVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  state.batteryVoltage = state.batteryVoltage != -1 ? (newBatteryVoltage + state.batteryVoltage) / 2 : newBatteryVoltage;
  if(state.currentDisplayMode != 1 && state.batteryVoltage <= REMOTE_BATTERY_CELL_V_THR[BATTERY_THRESHOLDS_LENGTH-1]) {
     changeMode(1);
  }
  return true;
}