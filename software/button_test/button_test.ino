#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  pinMode(3, INPUT_PULLDOWN);
}

void loop() {
  Serial.print("BUTTON: ");
  Serial.print(digitalRead(3));
  Serial.println("");
  delay(10);
}