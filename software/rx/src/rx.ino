#include <Arduino.h>
#include <SPI.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "board.h"
#include "settings.h"
#include "config.h"
#include "state.h"
#include "battery.h"
#include "RF.h"
#include "PPM.h"
#include "wifi_setup.h"
#include "error_handling.h"


const int TASKS_LENGTH = 5;

char *taskNames[] = { "receiveThrottlePacket", "writePPMValue", "sendTMPacket", "checkBattery", "printStats" };
long lastRun[] = { 0, 0, 0, 0, 0 };
long executions[] = { 0, 0, 0, 0, 0 };


#define DEBUG


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
  float ellapsed = (now - lastRun[TASKS_LENGTH-1])/1000;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
  Serial.print(state.boardVoltage/1000.0);
  Serial.print(F("V | SNR: "));
  Serial.print(state.currentSNR);
  Serial.print(F("dB | RSSI: "));
  Serial.print(state.currentRSSI);
  Serial.println(F("dBm"));
  Serial.println(F("-------------- TASKS --------------"));
  for(int i = 0; i < TASKS_LENGTH - 1; i++) {
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

task tasks[] = { receiveThrottlePacket, writePPMValue, sendTMPacket, checkBattery, printStats };

void loop()
{
  for(int i = 0; i < TASKS_LENGTH; i++) {
    unsigned long now = millis();
    if(now - lastRun[i] >= periods[i]) {
      if(tasks[i](now)) {
        executions[i]++;
      }
      lastRun[i] = millis();
    }
  }
}


void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  EEPROM.begin(18);
  readConfig();
  pinMode(PPM_THR1, OUTPUT);

  attachInterrupt(RFBUSY, processRFInterrupt, CHANGE);
  PPM.attach(PPM_THR1);

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
  } else {
    attachInterrupt(RFBUSY, processRFInterrupt, CHANGE);

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
}
