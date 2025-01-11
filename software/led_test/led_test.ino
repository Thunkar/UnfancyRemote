#include <Arduino.h>
#include <FastLED.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const int LEDS_LENGTH = 4;
const int LED = 8;
  CRGB leds[4];

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector



  pinMode(LED, OUTPUT);

  FastLED.addLeds<WS2812B, LED, GRB>(leds, LEDS_LENGTH);
  FastLED.setBrightness(255);
  FastLED.show();  
  Serial.begin(115200);
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
}

void loop() {
  Serial.println("pixel");
    leds[0] = CRGB::Green;
  leds[1] = CRGB::Red;
  leds[2] = CRGB::Blue;
  leds[3] = CRGB::White;
  FastLED.show();
  delay(500);
}