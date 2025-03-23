#include <Arduino.h>
#include <SPI.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "board.h"
#include "utils.h"
#include "config.h"
#include "state.h"
#include "RF.h"
#include "mode.h"
#include "button.h"
#include "throttle.h"
#include "LED.h"
#include "motor.h"
#include "battery.h"
#include "wifi_setup.h"
#include "stats.h"
#include "error_handling.h"	


void ONSequence() {
    for(int i = LEDS_LENGTH-1; i > -1; i--) {
      LEDColor[i] = rainbow[i];
      FastLED.show();
      delay(100);
    }
    digitalWrite(ON, HIGH);
    delay(100);
    digitalWrite(MOTOR, HIGH);
    delay(100);
    unsigned long now = millis();
    unsigned long lastCheck = now;
    while(digitalRead(BUTTON)){
      state.setupMode = (lastCheck - now) > SETUP_MODE_DELAY;
      lastCheck = millis();
      if(state.setupMode) {
        break;
      }
    };
    digitalWrite(MOTOR, LOW);
    for(int i = 0; i < LEDS_LENGTH; i++) {
      LEDColor[i] = CRGB::Black;
      FastLED.show();
      delay(50);
    }
}

const unsigned long periods[N_TASKS] = { 20, 10, 200, 1000, 100, 50, 20, 50, 2000 };

typedef bool (*task)(unsigned long);

task tasks[] = { sendThrottlePacket, readThrottle, checkButton, checkBattery, displayMode, setLEDs, setMotor, doServerWork, printStats };

void loop() {
  for(int i = 0; i < N_TASKS; i++) {
    unsigned long start = micros();
    unsigned long startMillis = start/1000;
    if(state.activeTasks[i] && ((startMillis - state.lastRun[i]) >= periods[i])) {
      if(tasks[i](startMillis)) {
        stats.successes[i]++;
        state.lastRun[i] = startMillis;
      } else {
        stats.failures[i]++;
      }
      unsigned long end = micros();
      unsigned long ellapsed = end - start;
      stats.times[i]+=ellapsed;
      if(stats.maxTimes[i] < ellapsed) {
        stats.maxTimes[i] = ellapsed;
      } 
      if (stats.minTimes[i] > ellapsed) {
        stats.minTimes[i] = ellapsed;
      }
    }
  }
  stats.loops++;
}


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  EEPROM.begin(18);
  readConfig();
  readCalibration();
  FastLED.addLeds<WS2812B, LED, GRB>(LEDColor, LEDS_LENGTH);
  FastLED.setBrightness(128);
  FastLED.show();
  pinMode(THR1, INPUT);
  pinMode(ON, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(RFBUSY, INPUT);
  pinMode(NRESET, OUTPUT);


  ONSequence();

  if(state.setupMode) {
    state.activeTasks[7] = true;
  }
  
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  if(state.setupMode) {
    if(!SPIFFS.begin(true)){
      Serial.println(F("An Error has occurred while mounting SPIFFS"));
      return;
    }
    WiFi.softAP("Unfancy Remote TX");
    setupServer();
    #ifdef DEBUG
    Serial.println(F("Setup mode"));
    #endif
  } 

  SPI.begin();

  if (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    #ifdef DEBUG
    Serial.println(F("Device error"));
    #endif
  }

  LT.setupLoRa(config.frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  LT.setPeriodBase(PERIODBASE_15_US);

  #ifdef DEBUG
  Serial.println(F("Remote ready"));
  #endif

 
}
