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

int LAST_TASK;
int FIRST_TASK;
unsigned long lastRun[] = { 0, 0, 0, 0, 0 };
unsigned long executions[] = { 0, 0, 0, 0, 0 };

char *taskNames[] = { "receiveThrottlePacket", "writePPMValue", "checkBattery", "printStats", "doServerWork" };

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
  float ellapsed = (now - lastRun[3])/1000;
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
    float frequency = executions[i] / ellapsed;
    sprintf(prBuffer, "%-23s | %.2fHz", taskNames[i], frequency);
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
  Serial.println(F("RF waits: "));
  float RFWaitMeanUs = state.waitingForRF / state.RFWaits;
  char meanTimeWaitingBuffer[50];
  sprintf(meanTimeWaitingBuffer, "%-40s %.2fus", "- Mean time waiting:", RFWaitMeanUs); 
  Serial.print(meanTimeWaitingBuffer);
  Serial.println("");
  float RFWaitsPerSecond = state.RFWaits / ellapsed;
  char RFWaitsPerSecondBuffer[50];
  sprintf(RFWaitsPerSecondBuffer, "%-40s %.2f", "- RF waits/s: ", RFWaitsPerSecond);
  Serial.print(RFWaitsPerSecondBuffer);
  Serial.println("");
  Serial.print(F("Errors: "));

  Serial.println(state.errors);
  state.errors = 0;
  state.packets = 0;
  state.TMPackets = 0;
  state.RFWaits = 0;
  state.waitingForRF = 0;
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { receiveThrottlePacket, writePPMValue, checkBattery, printStats, doServerWork };

void loop() {
  LAST_TASK = state.setupMode ? 4 : 3;
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
  pinMode(RFBUSY, INPUT);

  ONSequence();

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

  LT.setupLoRa(frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  LT.clearIrqStatus(IRQ_RADIO_ALL);

  #ifdef DEBUG
  Serial.println(F("Receiver ready"));
  #endif
}
