#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
}

void loop() {
  Serial.print("THR1:");
  Serial.print(analogReadMilliVolts(0));
  Serial.print(",");
  Serial.print("THR2:");
  Serial.print(analogReadMilliVolts(1));
  Serial.println("");
  delay(10);
}