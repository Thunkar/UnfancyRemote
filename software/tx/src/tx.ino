#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "board.h"
#include "scheduler.h"
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
    unsigned long now = micros();
    unsigned long lastCheck = now;
    while(digitalRead(BUTTON)){
      state.setupMode = (lastCheck - now) > SETUP_MODE_DELAY;
      lastCheck = micros();
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


task tasks[N_TASKS] = { sendThrottlePacket, readThrottle, checkButton, checkBattery, displayMode, setLEDs, setMotor, doServerWork, printStats };

void loop() {
  schedule(tasks);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  setupEEPROM();
  readConfig();
  readCalibration();
  setupLEDs();
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

  setupLoRa();

  #ifdef DEBUG
  Serial.println(F("Remote ready"));
  #endif

 
}
