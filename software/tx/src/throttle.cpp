#include "throttle.h"

const unsigned int BRAKE_SENSITIVITY = 5;

bool readThrottle(unsigned long now) {
  if(state.lastButtonState) {
    state.encodedThrottleValue = ENCODED_HALF;
    return true;
  }
  if(config.isDual) {
    unsigned int newRawThrottle1Voltage = analogReadMilliVolts(PPM_THR1);
    state.rawThrottle1Value = state.rawThrottle1Value != -1 ? (newRawThrottle1Voltage + state.rawThrottle1Value) / 2 : newRawThrottle1Voltage;
    unsigned int newRawThrottle2Voltage = analogReadMilliVolts(THR2);
    state.rawThrottle2Value = state.rawThrottle2Value != -1 ? (newRawThrottle2Voltage + state.rawThrottle2Value) / 2 : newRawThrottle2Voltage;
    unsigned int throttle1Value = constrain(state.rawThrottle1Value, min(config.centerAcc, config.calAcc) + 1, max(config.centerAcc, config.calAcc) - 1);
    unsigned int throttle2Value = constrain(state.rawThrottle2Value, min(config.centerBrake, config.calBrake) + 1, max(config.centerBrake, config.calBrake) - 1);

    bool isBraking = abs((int)throttle2Value-(int)config.centerBrake) > BRAKE_SENSITIVITY;
    
    if(isBraking) {
      state.encodedThrottleValue = throttle2Value > config.centerBrake ? 
                                        ENCODED_HALF - map(throttle2Value, config.centerBrake, config.calBrake, 0, ENCODED_HALF):
                                        map(throttle2Value, config.calBrake, config.centerBrake, 0, ENCODED_HALF); 
    } else {
      state.encodedThrottleValue = throttle1Value > config.centerAcc ? 
                                        map(throttle1Value, config.centerAcc, config.calAcc, ENCODED_HALF, ENCODED_MAX):
                                        ENCODED_HALF - map(throttle1Value, config.calAcc, config.centerAcc, ENCODED_HALF+1, ENCODED_MAX); 
  }
  } else {
    unsigned int newRawThrottle1Voltage = analogReadMilliVolts(PPM_THR1);
    state.rawThrottle1Value = state.rawThrottle1Value != -1 ? (newRawThrottle1Voltage + state.rawThrottle1Value) / 2 : newRawThrottle1Voltage;
    unsigned int throttle1Value = constrain(state.rawThrottle1Value, min(config.calBrake, config.calAcc) + 1, max(config.calBrake, config.calAcc) - 1);
    unsigned int scaledValue = throttle1Value > config.centerAcc ? 
                                  map(throttle1Value, config.centerAcc, max(config.calBrake, config.calAcc), ENCODED_HALF, ENCODED_MAX) : 
                                  map(throttle1Value, min(config.calBrake, config.calAcc), config.centerAcc, 0, ENCODED_HALF); 
    state.encodedThrottleValue = config.inverted ? ENCODED_MAX - scaledValue : scaledValue;
  }
  #ifdef CALIBRATION
  Serial.print(throttle1Value);
  Serial.print(F(" | "));
  #ifdef DUAL_THROTTLE
  Serial.print(throttle2Value);
  Serial.print(F(" | "));
  Serial.print(isBraking);
  Serial.print(F(" "));
  #endif
  Serial.println(encodedThrottleValue);
  #endif
  return true;
}