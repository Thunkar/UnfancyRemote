#include <Arduino.h>
#include <SPI.h>
#include "board.h"
#include "scheduler.h"
#include "config.h"
#include "state.h"
#include "RF.h"
#include "PPM.h"
#include "battery.h"
#include "wifi_setup.h"
#include "error_handling.h"	
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

task tasks[N_TASKS] = { receiveThrottlePacket, writePPMValue, checkBattery, doServerWork, printStats };

void loop() {
  schedule(tasks);
}

void setup() {
  EEPROM.begin(6);
  readConfig();
  pinMode(PPM, OUTPUT);
  pinMode(BUTTON, INPUT_PULLDOWN);
  pinMode(LED, OUTPUT);
  pinMode(VBAT, INPUT_PULLDOWN);
  pinMode(RFBUSY, INPUT);
  pinMode(NRESET, OUTPUT);

  ONSequence();

  if(state.setupMode) {
    state.activeTasks[3] = true;
  }

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  if(state.setupMode) {
    if(!SPIFFS.begin(true)){
      Serial.println(F("An Error has occurred while mounting SPIFFS"));
      return;
    }
    WiFi.softAP("Unfancy Remote RX");
    setupServer();
    #ifdef DEBUG
    Serial.println(F("Setup mode"));
    #endif
  } 

  PPM_OUTPUT.attach(PPM);

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
  Serial.println(F("Receiver ready"));
  #endif
}
