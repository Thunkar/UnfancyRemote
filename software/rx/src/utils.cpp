#include "utils.h"

// qsort requires you to create a sort function
int sort_desc(const void *cmp1, const void *cmp2) {
  // Need to cast the void * to int *
  unsigned int a = *((int *)cmp1);
  unsigned int b = *((int *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
}

unsigned int sampleAdc(const int pin) {
  unsigned int scaledmVolts[ADC_SAMPLES];

  for (int i = 0; i < ADC_SAMPLES; i++) {
    scaledmVolts[i] = analogReadMilliVolts(pin);
  }

  qsort(scaledmVolts, ADC_SAMPLES, sizeof(scaledmVolts[0]), sort_desc);

  unsigned int sum = 0;

  // Skip the first and last sample to avoid outliers
  for (int i = 1; i < ADC_SAMPLES - 1; i++) {
    sum += scaledmVolts[i];
  }

  return (unsigned int)((float)sum/(float)(ADC_SAMPLES - 2));
}

unsigned int roundAndCastToInt(float var) {
    return (int)(var * 100 + .5);
}