#include "throttle.h"

bool readThrottle(unsigned long now) {
  if(state.lastButtonState) {
    state.encodedThrottleValue = ENCODED_HALF;
    return true;
  }
  #ifdef DUAL_THROTTLE
  unsigned int throttle1Value = analogRead(PPM_THR1);
  unsigned int throttle2Value = analogRead(THR2);
  throttle1Value = constrain(throttle1Value, min(config.centerAcc, config.calAcc), max(config.centerAcc, config.calAcc));
  throttle2Value = constrain(throttle2Value, min(config.centerBrake, config.calBrake), max(config.centerBrake, config.calBrake));

  bool isBraking = abs((int)throttle2Value-(int)config.centerBrake) > BRAKE_SENSITIVITY;
  
  if(isBraking) {
     encodedThrottleValue = throttle2Value > config.centerBrake ? 
                                      ENCODED_HALF - map(throttle2Value, config.centerBrake, config.calBrake, 0, ENCODED_HALF):
                                      map(throttle2Value, config.calBrake, config.centerBrake, 0, ENCODED_HALF); 
  } else {
    encodedThrottleValue = throttle1Value > centerAcc ? 
                                      map(throttle1Value, config.centerAcc, config.calAcc, ENCODED_HALF, ENCODED_MAX):
                                      ENCODED_HALF - map(throttle1Value, config.calAcc, config.centerAcc, ENCODED_HALF+1, ENCODED_MAX); 
  }
  #else
  state.rawThrottleValue = analogReadMilliVolts(PPM_THR1);
  unsigned int throttle1Value = constrain(throttle1Value, min(config.calBrake, config.calAcc), max(config.calBrake, config.calAcc));
  unsigned int scaledValue = throttle1Value > config.centerAcc ? 
                                map(throttle1Value, config.centerAcc, max(config.calBrake, config.calAcc), ENCODED_HALF, ENCODED_MAX) : 
                                map(throttle1Value, min(config.calBrake, config.calAcc), config.centerAcc, 0, ENCODED_HALF); 
  state.encodedThrottleValue = config.inverted ? ENCODED_MAX - scaledValue : scaledValue;
  #endif
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