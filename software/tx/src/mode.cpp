#include "mode.h"


int nextDisplayMode = 0;

void changeMode(int mode) {
  state.currentDisplayMode = -1;
  nextDisplayMode = mode;
  for(int i = 0; i < LEDS_LENGTH; i++) {
    setLEDOff(i);
  }
  pulseMotor(nextDisplayMode+1, 100 * 1e3);
}

TaskResult displayMode(unsigned long now) {
  if(nextDisplayMode != state.lastDisplayMode) {
    if(now - state.lastModeTransition > TRANSITION_DELAY_US) {
      state.currentDisplayMode = nextDisplayMode;
      state.lastDisplayMode = nextDisplayMode;
    }
  }
  switch(state.currentDisplayMode) {
    case 0: {
      if(state.isConnected && !state.setupMode) {
        for(int i = 0; i < BATTERY_THRESHOLDS_LENGTH-1; i++) {
          if(state.boardCellVoltage > BOARD_BATTERY_CELL_V_THR[i]) {
            changeLEDColor(i, rainbow[i]);
            setLEDOn(i);
          } else {
            setLEDOff(i);
          }
        }
        if(state.boardCellVoltage <= BOARD_BATTERY_CELL_V_THR[BATTERY_THRESHOLDS_LENGTH-1] && state.boardCellVoltage > 0) {
          pulseMotor(-1, 500 * 1e3);
          flashLED(3, -1, 200, CRGB::Black);
        } else {
          pulseMotor(-1, -1);
        }
      } else {
        for(int i = 0; i < LEDS_LENGTH; i++) {
          changeLEDColor(i, state.setupMode ? CRGB::Green : CRGB::Blue);
          pulseMotor(-1, -1);
          sequence();
        }
      }
      break;
    }
    case 1: {
      for(int i = 0; i < BATTERY_THRESHOLDS_LENGTH-1; i++) {
        if(state.batteryVoltage > REMOTE_BATTERY_CELL_V_THR[i]) {
          changeLEDColor(i, rainbow[i]);
          setLEDOn(i);
        } else {
          setLEDOff(i);
        }
      }
      if(state.batteryVoltage <= REMOTE_BATTERY_CELL_V_THR[BATTERY_THRESHOLDS_LENGTH-1] && state.batteryVoltage > 0) {
          pulseMotor(-1, 100 * 1e3);
      } else {
          pulseMotor(-1, -1);
      }
      break;
    }
  }
  return { true };
}

