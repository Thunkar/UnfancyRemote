#include <Arduino.h>
#include "board.h"
#include "scheduler.h"
#include "config.h"
#include "state.h"
#include "RF.h"
#include "PPM.h"
#include "battery.h"
#include "wifi_setup.h"
#include "stats.h"

#define DEBUG

void ONSequence() {
  unsigned long now = micros();
  unsigned long lastCheck = now;
  digitalWrite(LED, HIGH);
  while(digitalRead(BUTTON)){
    state.setupMode = (lastCheck - now) > SETUP_MODE_DELAY;
    lastCheck = micros();
    if(state.setupMode) {
      digitalWrite(LED, LOW);
      break;
    }
  };
}

task tasks[N_TASKS] = { receiveThrottlePacket, checkRFStatus, handleThrottlePacket, sendTMPacket, writePPMValue, checkBattery, doServerWork, printStats };

void loop() {
  schedule(tasks);
}

void setup() {
  setupEEPROM();
  readConfig();
  pinMode(PPM, OUTPUT);
  pinMode(BUTTON, INPUT_PULLDOWN);
  pinMode(LED, OUTPUT);
  pinMode(VBAT, INPUT_PULLDOWN);
  pinMode(RFBUSY, INPUT);
  pinMode(NRESET, OUTPUT);

  ONSequence();

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  if(state.setupMode) {
    state.activeTasks[6] = true;
    if(!SPIFFS.begin(true)){
      Serial.println(F("An Error has occurred while mounting SPIFFS"));
      return;
    }
    setupServer();
    #ifdef DEBUG
    Serial.println(F("Setup mode"));
    #endif
  } 

  PPM_OUTPUT.attach(PPM);
  setupLoRa();

  #ifdef DEBUG
  Serial.println(F("Receiver ready"));
  #endif
}
