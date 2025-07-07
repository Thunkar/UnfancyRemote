#include "throttle.h"

TaskResult readThrottle(unsigned long now) {
  if(config.isDual) {
    state.rawThrottle1Value = sampleAdc(THR1);
    state.rawThrottle2Value = sampleAdc(THR2);
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
    state.rawThrottle1Value = sampleAdc(THR1);
    unsigned int throttle1Value = constrain(state.rawThrottle1Value, min(config.calBrake, config.calAcc) + 1, max(config.calBrake, config.calAcc) - 1);
    unsigned int scaledValue = throttle1Value > config.centerAcc ? 
                                  map(throttle1Value, config.centerAcc, max(config.calBrake, config.calAcc), ENCODED_HALF, ENCODED_MAX) : 
                                  map(throttle1Value, min(config.calBrake, config.calAcc), config.centerAcc, 0, ENCODED_HALF); 
    state.encodedThrottleValue = config.inverted ? ENCODED_MAX - scaledValue : scaledValue;
  }
  return { true };
}