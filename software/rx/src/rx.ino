#include <Arduino.h>
#include <SPI.h>
#include "board.h"
#include "config.h"
#include "state.h"
#include "RF.h"
#include "PPM.h"
#include "battery.h"
#include "wifi_setup.h"
#include "error_handling.h"


int LAST_TASK;
int FIRST_TASK;

char *taskNames[] = { "receiveThrottlePacket", "writePPMValue", "sendTMPacket", "checkBattery", "printStats", "doServerWork" };
long lastRun[] = { 0, 0, 0, 0, 0, 0 };
long executions[] = { 0, 0, 0, 0, 0, 0 };


#define DEBUG

void ONSequence() {
  unsigned long now = millis();
  unsigned long lastCheck = now;
  digitalWrite(LED, HIGH);
  while(digitalRead(BUTTON)){
    state.setupMode = (lastCheck - now) > setupModeDelay;
    lastCheck = millis();
    if(state.setupMode) {
      digitalWrite(LED, LOW);
      break;
    }
  };
}


bool printStats(unsigned long now) {
  #ifdef DEBUG
  if(state.errors > 0) {
    Serial.println(F("////////ERROR//////////"));
    Serial.println(state.errorReason);
    Serial.println(F("//////////////////////"));
  }
  Serial.print(F("Frequency: "));
  Serial.print(frequency);
  Serial.println(F("Hz"));
  float ellapsed = (now - lastRun[4])/1000;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
  Serial.print(state.boardVoltage);
  Serial.print(F("V | SNR: "));
  Serial.print(state.currentSNR);
  Serial.print(F("dB | RSSI: "));
  Serial.print(state.currentRSSI);
  Serial.println(F("dBm"));
  Serial.println(F("-------------- TASKS --------------"));
  for(int i = FIRST_TASK; i <= LAST_TASK; i++) {
    char prBuffer[45];
    int frequency = round(executions[i] / ellapsed);
    sprintf(prBuffer, "%-23s | %5dHz", taskNames[i], frequency);
    Serial.print(prBuffer);
    Serial.println("");
    executions[i] = 0;
  }
  Serial.println(F("-----------------------------------"));
  int packetsPerSecond = round(state.packets / ellapsed);
  Serial.print(F("Packets/s: "));
  Serial.println(packetsPerSecond);
  int TMPacketsPerSecond = round(state.TMPackets / ellapsed);
  Serial.print(F("TM packets/s: "));
  Serial.println(TMPacketsPerSecond);
  Serial.print(F("Errors: "));
  Serial.println(state.errors);
  state.errors = 0;
  state.packets = 0;
  state.TMPackets = 0;
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { receiveThrottlePacket, writePPMValue, sendTMPacket, checkBattery, printStats, doServerWork };

void loop() {
  LAST_TASK = state.setupMode ? 5 : 4;
  FIRST_TASK = 0; 
  for(int i = FIRST_TASK; i <= LAST_TASK; i++) {
    unsigned long now = millis();
    if(now - lastRun[i] >= periods[i]) {
      if(tasks[i](now)) {
        executions[i]++;
      }
      lastRun[i] = millis();
    }
  }
}


void setup() {
  EEPROM.begin(6);
  readConfig();
  pinMode(PPM, OUTPUT);
  pinMode(BUTTON, INPUT_PULLDOWN);
  pinMode(LED, OUTPUT);
  pinMode(VBAT, INPUT_PULLDOWN);
  ONSequence();

  attachInterrupt(RFBUSY, processRFInterrupt, CHANGE);

  PPM_OUTPUT.attach(PPM);

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

  SPI.begin();

  if (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    #ifdef DEBUG
    Serial.println(F("Device error"));
    #endif
  }

  LT.setupLoRa(frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  LT.clearIrqStatus(IRQ_RADIO_ALL);

  #ifdef DEBUG
  Serial.println(F("Receiver ready"));
  #endif
}
