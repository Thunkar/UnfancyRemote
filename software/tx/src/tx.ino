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
#include "error_handling.h"

#define DEBUG

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

int LAST_TASK;
const unsigned long periods[] = { 20, 10, 200, 1000, 100, 50, 20, 2000, 50 };
unsigned long lastRun[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long successes[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long failures[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long times[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long loops = 0;

const char *taskNames[] = { "sendThrottlePacket", "readThrottle", "checkButton", "checkBattery", "displayMode", "setLEDs", "setMotor", "printStats", "doServerWork" };

bool printStats(unsigned long now) {
  #ifdef DEBUG
  if(state.errors > 0) {
    Serial.println(F("////////ERROR//////////"));
    Serial.println(state.errorReason);
    Serial.println(F("//////////////////////"));
  }
  float ellapsed = (now - lastRun[7])/1000;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
  Serial.print(state.batteryVoltage);
  Serial.print(F("V | Mode: "));
  Serial.println(state.currentDisplayMode);
  Serial.print(F("Frequency: "));
  Serial.print(frequency);
  Serial.println(F("Hz"));
  Serial.print(F("Board V: "));
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
  Serial.println(F("---------------------- TASKS ----------------------"));
  for(int i = 0; i <= LAST_TASK; i++) {
    char prBuffer[100];
    long executions = successes[i] + failures[i];
    float frequency = executions / ellapsed;
    float mean = times[i] / (float)executions;
    sprintf(prBuffer, "%-20s | %6.2fHz | ~%7.2fus | %.2f", taskNames[i], frequency, mean, successes[i]/(float)executions);
    Serial.print(prBuffer);
    Serial.println("");
    successes[i] = 0;
    failures[i] = 0;
    times[i] = 0;
  }
  Serial.println(F("---------------------------------------------------"));
  float loopFrequency = loops / ellapsed;
  Serial.print(F("Loop frequency: "));
  Serial.print(loopFrequency);
  Serial.println(F("Hz"));
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
  loops = 0;
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { sendThrottlePacket, readThrottle, checkButton, checkBattery, displayMode, setLEDs, setMotor, printStats, doServerWork };

void loop() {
  for(int i = 0; i <= LAST_TASK; i++) {
    unsigned long start = micros();
    unsigned long startMillis = start/1000;
    if((startMillis - lastRun[i]) >= periods[i]) {
      if(tasks[i](startMillis)) {
        successes[i]++;
      } else {
        failures[i]++;
      }
      unsigned long end = micros();
      times[i]+=(end - start);
      lastRun[i] = end/1000;
    }
  }
  loops++;
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

  ONSequence();

  LAST_TASK = state.setupMode ? 8 : 7;
  
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

  LT.setupLoRa(frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  LT.clearIrqStatus(IRQ_RADIO_ALL);

  #ifdef DEBUG
  Serial.println(F("Remote ready"));
  #endif

 
}
