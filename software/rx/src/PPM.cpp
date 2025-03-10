
#include "PPM.h"

Servo PPM_OUTPUT;

bool writePPMValue(unsigned long now) {
  unsigned int throttlePulse = map(state.encodedThrottleValue, 0, ENCODED_MAX, 1000, 2000);
  PPM_OUTPUT.writeMicroseconds(throttlePulse);
  return true;
}
