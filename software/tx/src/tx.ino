#include <Arduino.h>
#include <SPI.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "board.h"
#include "settings.h"
#include "state.h"
#include "RF.h"
#include "mode.h"
#include "button.h"
#include "throttle.h"
#include "LED.h"
#include "motor.h"
#include "battery.h"
#include "wifi_setup.h"
#include "error_handling.h"


int LAST_TASK;
int FIRST_TASK;

char *taskNames[] = { "sendThrottlePacket", "receiveTMPacket", "readThrottle", "checkButton", "checkBattery", "displayMode", "setLEDs", "setMotor", "printStats", "doServerWork" };
unsigned long lastRun[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long executions[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


#define DEBUG
//#define CALIBRATION


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
      state.setupMode = (lastCheck - now) > setupModeDelay;
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

void calibrate() {
  LEDColor[3] = CRGB::White;
  FastLED.show();
  while(!digitalRead(BUTTON)) {
    config.centerAcc = analogRead(PPM_THR1);
    if(config.isDual) {
      config.centerBrake = analogRead(THR2);
    } else {
      Serial.println();
    }
  }
  digitalWrite(MOTOR, HIGH);
  delay(500);
  digitalWrite(MOTOR, LOW);
  LEDColor[2] = CRGB::White;
  FastLED.show();
  int diff = 0;
  while(!digitalRead(BUTTON)) {
    unsigned int current = analogRead(PPM_THR1);
    int newDiff = abs((int)config.centerAcc-(int)current);
    if(newDiff > diff) {
      config.calAcc = current;
      diff = newDiff;
    }
  }
  digitalWrite(MOTOR, HIGH);
  delay(500);
  digitalWrite(MOTOR, LOW);
  LEDColor[1] = CRGB::White;
  FastLED.show();
  diff = 0;
  while(!digitalRead(BUTTON)) {
    int current = 0;
    int newDiff = 0;
    if(config.isDual) {
      current = analogRead(THR2);
      newDiff = abs((int)config.centerBrake-(int)current);
    } else {
      current = analogRead(PPM_THR1);
      newDiff = abs((int)config.centerAcc-(int)current);
    }
    if(newDiff > diff) {
      config.calBrake = current;
      diff = newDiff;
    }
  }
  config.inverted = config.calAcc < config.calBrake;
  digitalWrite(MOTOR, HIGH);
  delay(500);
  digitalWrite(MOTOR, LOW);
  for(int i = 0; i < LEDS_LENGTH; i++) {
    LEDColor[i] = CRGB::Black;
    FastLED.show();
  }
}

bool printStats(unsigned long now) {
  #ifdef DEBUG
  if(state.errors > 0) {
    Serial.println(F("////////ERROR//////////"));
    Serial.println(state.errorReason);
    Serial.println(F("//////////////////////"));
  }
  float ellapsed = (now - lastRun[8])/1000;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
  Serial.print(state.batteryVoltage);
  Serial.print(F("V | Mode: "));
  Serial.println(state.currentDisplayMode);
  Serial.print(F("Frequency: "));
  Serial.print(frequency);
  Serial.println(F("Hz"));
  Serial.print(F("SNR: "));
  Serial.print(state.currentSNR);
  Serial.print(F("dB | RSSI: "));
  Serial.print(state.currentRSSI);
  Serial.print(F("dBm | Board V: "));
  Serial.print(state.boardVoltage);
  Serial.print(F(" ("));
  Serial.print(state.boardCellVoltage);
  Serial.println(F(")"));
  Serial.print(F("Calibration: "));
  Serial.print(config.calBrake);
  Serial.print(F(" | "));
  Serial.print(config.centerAcc);
  Serial.print(F(" | "));
  Serial.print(config.calAcc);
  Serial.print(F(" | Inverted: "));
  Serial.println(config.inverted ? "y" : "n");
  Serial.println(F("-------------- TASKS --------------"));
  for(int i = FIRST_TASK; i <= LAST_TASK; i++) {
    char prBuffer[45];
    float frequency = executions[i] / ellapsed;
    sprintf(prBuffer, "%-20s | %.2fHz", taskNames[i], frequency);
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
  int interruptsPerSecond = round(state.interruptCounter / ellapsed);
  Serial.print(F("Interrupts/s: "));
  Serial.println(interruptsPerSecond);
  Serial.print(F("Errors: "));
  Serial.println(state.errors);
  state.errors = 0; 
  state.packets = 0;
  state.TMPackets = 0;
  state.interruptCounter = 0;
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { sendThrottlePacket, receiveTMPacket, readThrottle, checkButton, checkBattery, displayMode, setLEDs, setMotor, printStats, doServerWork };

void loop() {
  LAST_TASK = state.setupMode ? 9 : 8;
  FIRST_TASK = state.setupMode ? 2 : 0; 
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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  EEPROM.begin(16);
  readConfig();
  readCalibration();
  FastLED.addLeds<WS2812B, LED, GRB>(LEDColor, LEDS_LENGTH);
  FastLED.setBrightness(128);
  FastLED.show();
  pinMode(PPM_THR1, INPUT);
  pinMode(ON, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(BUTTON, INPUT);
  ONSequence();
  
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  if(state.setupMode) {
    if(!SPIFFS.begin(true)){
      Serial.println(F("An Error has occurred while mounting SPIFFS"));
      return;
    }
    WiFi.softAP("Unfancy Remote");
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
    Serial.println(F("Remote ready"));
    #endif
  }

 
}
