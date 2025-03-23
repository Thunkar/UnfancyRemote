#include <Arduino.h>
#include <SPI.h>
#include "board.h"
#include "config.h"
#include "state.h"
#include "RF.h"
#include "PPM.h"
#include "battery.h"
#include "wifi_setup.h"
#include "stats.h"
#include "error_handling.h"	

#define DEBUG

void ONSequence() {
  unsigned long now = millis();
  unsigned long lastCheck = now;
  digitalWrite(LED, HIGH);
  while(digitalRead(BUTTON)){
    state.setupMode = (lastCheck - now) > SETUP_MODE_DELAY;
    lastCheck = millis();
    if(state.setupMode) {
      digitalWrite(LED, LOW);
      break;
    }
  };
}

const unsigned long periods[] = { 20, 20, 1000, 50, 2000 };
unsigned long lastRun[] = { 0, 0, 0, 0, 0 };
unsigned long successes[] = { 0, 0, 0, 0, 0 };
unsigned long failures[] = { 0, 0, 0, 0, 0 };
unsigned long times[] = { 0, 0, 0, 0, 0 };
unsigned long maxTimes[] = { 0, 0, 0, 0, 0 };
unsigned long minTimes[] = { 10000000, 10000000, 10000000, 10000000, 10000000 };
unsigned long loops = 0;

typedef bool (*task)(unsigned long);

task tasks[] = { receiveThrottlePacket, writePPMValue, checkBattery, doServerWork, printStats };

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

  #ifdef DEBUG
  Serial.println(F("Receiver ready"));
  #endif
}
