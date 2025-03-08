
#include "PPM.h"

Servo PPM;

bool writePPMValue(unsigned long now) {
  unsigned int throttlePulse = map(state.encodedThrottleValue, 0, ENCODED_MAX, 1000, 2000);
  PPM.writeMicroseconds(throttlePulse);
  return true;
}
