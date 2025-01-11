
#include <Arduino.h>
#include <SPI.h>
#include <SX128XLT.h>
#include <EEPROM.h>
#include "board.h"
#include "settings.h"
#include <ProgramLT_Definitions.h>
#include <FastLED.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

SX128XLT LT;

const int TASKS_LENGTH = 9;

char *taskNames[] = { "readThrottle", "sendThrottlePacket", "receiveTMPacket", "checkButton", "checkBattery", "displayMode", "setLEDs", "setMotor", "printStats" };
unsigned long periods[] = { 10, 20, 1, 100, 1000, 50, 10, 10, 2000 };
unsigned long lastRun[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long executions[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

unsigned int encodedThrottleValue;
const unsigned int ENCODED_MAX = 65535;
const unsigned int ENCODED_HALF = 32768;

const int LEDS_LENGTH = 4;

CRGB LEDStatus[] = { CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black };
CRGB storedLEDStatus[] = { CRGB::Black, CRGB::Black, CRGB::Black, CRGB::Black };
unsigned long LEDPeriods[] = { -1, -1, -1, -1 };
int LEDResetCounters[] = { -1, -1, -1, -1 };
unsigned long lastLEDToggled[] = { 0, 0, 0, 0 };

int motorStatus = LOW;
unsigned long motorPeriod = -1;
unsigned long lastMotorToggled = 0;
int motorResetCounter = -1;
 
int lastButtonState = LOW;  

unsigned long lastPressedTime = 0;
unsigned long debounceDelay = 100;
unsigned long offDelay = 1500;    
unsigned long changeModeDelay = 500;

float batteryVoltage = -1.0;

int lastDisplayMode = 0;
int currentDisplayMode = 0; // -1 transition, 0 board voltage, 1 remote voltage
int nextDisplayMode = 0;
unsigned long transitionDelay = 250;
unsigned long lastTransition = 0;
bool canChangeMode = true;

const int BATTERY_THRESHOLDS_LENGTH = 5;

volatile int RFAvailable = 1;
volatile int interruptCounter = 1;
bool forceTX = true;

bool error = false;
int errors = 0;
char errorReason[30];
long currentRSSI = -100;
int currentSNR = -100;
unsigned long frequency;

unsigned long packets = 0;
unsigned long TMPackets = 0;
float boardVoltage = 0.0;
float boardCellVoltage = 0.0;

unsigned long TMPeriod = 500;
unsigned long lastTMPacketReceived = 0;

unsigned int requestTM = 0;
bool waitingForRX = false;
unsigned int maxWaitForTM = 40;
unsigned int currentTMCycles = 0;
unsigned int currentTransmitCycles = 0;
unsigned int maxWaitForTransmit = 20;
int resetTMCounter = 0;

int VCC = 0;

unsigned int calBrake;
unsigned int calAcc;
unsigned int centerAcc;
unsigned int centerBrake;
unsigned int inverted;

bool forceCalibration = false;
int forceCalibrationDelay = 5000;

#define DEBUG
//#define DEBUG_FLAGS
//#define CALIBRATION

void resetTM() {
  currentSNR = -100;
  currentRSSI = -100;
  boardVoltage = 0.0;
  boardCellVoltage = 0.0;
}

void printFlags(char title[]) {
  Serial.print(title);
  Serial.print(F(" | RFAvailable: "));
  Serial.print(RFAvailable);
  Serial.print(F(" | RequestTM: "));
  Serial.println(requestTM);
}

int readVcc(void) {
  int result = 0;
  return result; // Vcc in millivolts
}

void clearError() {
  error = false;
}

void setError(char reason[]) {
  errors++;
  error = true;
  strcpy(errorReason, reason);
}

void writeUInt(int address, unsigned int number){ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

unsigned int readUInt(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void readSettings() {
  centerAcc = readUInt(0);
  calAcc = readUInt(2);
  calBrake = readUInt(4);
  inverted = readUInt(6);
  centerBrake = readUInt(8);
}

void writeSettings() {
  writeUInt(0, centerAcc);
  writeUInt(2, calAcc);
  writeUInt(4, calBrake);
  writeUInt(6, inverted);
  writeUInt(8, centerBrake);
}

void ONSequence() {
    for(int i = LEDS_LENGTH-1; i > -1; i--) {
      LEDStatus[i] = CRGB::Red;
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
      forceCalibration = (lastCheck - now) > forceCalibrationDelay;
      if(forceCalibration) {
        digitalWrite(MOTOR, LOW);
      }
      lastCheck = millis();
    };
    digitalWrite(MOTOR, LOW);
    for(int i = 0; i < LEDS_LENGTH; i++) {
      LEDStatus[i] = CRGB::Black;
      FastLED.show();
      delay(50);
    }
}

void calibrate() {
  LEDStatus[3] = CRGB::White;
  FastLED.show();
  while(!digitalRead(BUTTON)) {
    centerAcc = analogRead(PPM_THR1);
    #ifdef DUAL_THROTTLE
    centerBrake = analogRead(THR2);
    #else
    Serial.println();
    #endif
  }
  digitalWrite(MOTOR, HIGH);
  delay(500);
  digitalWrite(MOTOR, LOW);
  LEDStatus[2] = CRGB::White;
  FastLED.show();
  int diff = 0;
  while(!digitalRead(BUTTON)) {
    unsigned int current = analogRead(PPM_THR1);
    int newDiff = abs((int)centerAcc-(int)current);
    if(newDiff > diff) {
      calAcc = current;
      diff = newDiff;
    }
  }
  digitalWrite(MOTOR, HIGH);
  delay(500);
  digitalWrite(MOTOR, LOW);
  LEDStatus[1] = CRGB::White;
  FastLED.show();
  diff = 0;
  while(!digitalRead(BUTTON)) {
    #ifdef DUAL_THROTTLE
    int current = analogRead(THR2);
    int newDiff = abs((int)centerBrake-(int)current);
    #else
    int current = analogRead(PPM_THR1);
    int newDiff = abs((int)centerAcc-(int)current);
    #endif
    if(newDiff > diff) {
      calBrake = current;
      diff = newDiff;
    }
  }
  inverted = calAcc < calBrake;
  digitalWrite(MOTOR, HIGH);
  delay(500);
  digitalWrite(MOTOR, LOW);
  for(int i = 0; i < LEDS_LENGTH; i++) {
    LEDStatus[i] = CRGB::Black;
    FastLED.show();
  }
}

void setLEDOn(int LEDn) {
  LEDPeriods[LEDn] = 0;
  LEDResetCounters[LEDn] = -1;
}

void setLEDOff(int LEDn) {
  LEDPeriods[LEDn] = -1;
  LEDResetCounters[LEDn] = -1;
}

void flashLED(int LEDn, int times, unsigned long period, int resetStatus) {
  LEDPeriods[LEDn] = period;
  LEDResetCounters[LEDn] = times;
  storedLEDStatus[LEDn] = resetStatus;
}

void pulseMotor(int times, unsigned long period) {
  motorPeriod = period;
  motorResetCounter = times;
}

void changeMode(int mode) {
  currentDisplayMode = -1;
  nextDisplayMode = mode;
  for(int i = 0; i < LEDS_LENGTH; i++) {
    setLEDOff(i);
  }
  pulseMotor(nextDisplayMode+1, 100);
}

bool displayMode(unsigned long now) {
  if(nextDisplayMode != lastDisplayMode) {
    if(now - lastTransition > transitionDelay) {
      currentDisplayMode = nextDisplayMode;
      lastDisplayMode = nextDisplayMode;
    }
  }
  switch(currentDisplayMode) {
    case 0: {
       for(int i = 0; i < BATTERY_THRESHOLDS_LENGTH-1; i++) {
        if(boardCellVoltage > BOARD_BATTERY_CELL_V_THR[i]) {
          setLEDOn(i);
        } else {
          setLEDOff(i);
        }
      }
      if(boardCellVoltage <= BOARD_BATTERY_CELL_V_THR[BATTERY_THRESHOLDS_LENGTH-1] && boardCellVoltage > 0) {
        pulseMotor(-1, 500);
        flashLED(3, -1, 200, 0);
      } else {
        pulseMotor(-1, -1);
      }
      break;
    }
    case 1: {
      for(int i = 0; i < BATTERY_THRESHOLDS_LENGTH-1; i++) {
        if(batteryVoltage > REMOTE_BATTERY_CELL_V_THR[i]) {
          setLEDOn(i);
        } else {
          setLEDOff(i);
        }
      }
      if(batteryVoltage <= REMOTE_BATTERY_CELL_V_THR[BATTERY_THRESHOLDS_LENGTH-1] && batteryVoltage > 0) {
          pulseMotor(-1, 100);
      } else {
          pulseMotor(-1, -1);
      }
      break;
    }
  }
  return true;
}

bool checkButton(unsigned long now) {
  int buttonState;
  int reading = digitalRead(BUTTON);
  if (reading != lastButtonState) {
    lastPressedTime = now;
  }

  if(now - lastPressedTime > debounceDelay) {
      buttonState = reading;
  }

  if (now - lastPressedTime > offDelay) {
    if (buttonState) {
      digitalWrite(MOTOR, HIGH);
      digitalWrite(ON, LOW);
      digitalWrite(4, LOW);
      pinMode(BUTTON, INPUT_PULLDOWN);
      delay(999999999999);
    }
  }

  if(now - lastPressedTime > changeModeDelay) {
    if(buttonState && currentDisplayMode != -1 && canChangeMode) {
      lastTransition = now;
      canChangeMode = false;
      changeMode(!currentDisplayMode);
    }
  }
  
  if(!reading && lastButtonState) {
    canChangeMode = true;
  }
  lastButtonState = reading;
  return true;
}

bool checkBattery(unsigned long now) {
  if(lastButtonState) {
    return false;
  }
  int batValue = analogRead(VBAT);
  int scaledBatmVolts = map(batValue, 0, 1023, 0, VCC);
  
  float newBatteryVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  batteryVoltage = batteryVoltage != -1 ? (newBatteryVoltage + batteryVoltage) / 2 : newBatteryVoltage;
  // if(currentDisplayMode != 1 && batteryVoltage <= REMOTE_BATTERY_CELL_V_THR[BATTERY_THRESHOLDS_LENGTH-1]) {
  //   changeMode(1);
  // }
  return true;
}

bool readThrottle(unsigned long now) {
  if(lastButtonState) {
    encodedThrottleValue = ENCODED_HALF;
    return true;
  }
  #ifdef DUAL_THROTTLE
  unsigned int throttle1Value = analogRead(PPM_THR1);
  unsigned int throttle2Value = analogRead(THR2);
  throttle1Value = constrain(throttle1Value, min(centerAcc, calAcc), max(centerAcc, calAcc));
  throttle2Value = constrain(throttle2Value, min(centerBrake, calBrake), max(centerBrake, calBrake));

  bool isBraking = abs((int)throttle2Value-(int)centerBrake) > BRAKE_SENSITIVITY;
  
  if(isBraking) {
     encodedThrottleValue = throttle2Value > centerBrake ? 
                                      ENCODED_HALF - map(throttle2Value, centerBrake, calBrake, 0, ENCODED_HALF):
                                      map(throttle2Value, calBrake, centerBrake, 0, ENCODED_HALF); 
  } else {
    encodedThrottleValue = throttle1Value > centerAcc ? 
                                      map(throttle1Value, centerAcc, calAcc, ENCODED_HALF, ENCODED_MAX):
                                      ENCODED_HALF - map(throttle1Value, calAcc, centerAcc, ENCODED_HALF+1, ENCODED_MAX); 
  }
  #else
  unsigned int throttle1Value = analogRead(PPM_THR1);
  throttle1Value = constrain(throttle1Value, min(calBrake, calAcc), max(calBrake, calAcc));
  unsigned int scaledValue = throttle1Value > centerAcc ? 
                                map(throttle1Value, centerAcc, max(calBrake, calAcc), ENCODED_HALF, ENCODED_MAX) : 
                                map(throttle1Value, min(calBrake, calAcc), centerAcc, 0, ENCODED_HALF); 
  encodedThrottleValue = inverted ? ENCODED_MAX - scaledValue : scaledValue;
  #endif
  #ifdef CALIBRATION
  Serial.print(throttle1Value);
  Serial.print(F(" | "));
  #ifdef DUAL_THROTTLE
  Serial.print(throttle2Value);
  Serial.print(F(" | "));
  Serial.print(isBraking);
  Serial.print(F(" "));
  #endif
  Serial.println(encodedThrottleValue);
  #endif
  return true;
}

void IRAM_ATTR processRFInterrupt() {
  RFAvailable = !digitalRead(RFBUSY);
  interruptCounter++;
}

void processTMPacket() {    
  unsigned int RXIdentity = -1;
  unsigned int receivedValue = 0;
  unsigned int measuredRXPacketLength = LT.readRXPacketL();
  int measuredSNR = 0;
  long measuredRSSI = 0;
  if(measuredRXPacketLength == TMPacketLength){
    LT.startReadSXBuffer(0);                
    RXIdentity = LT.readUint8();         
    receivedValue = LT.readUint16();       
    LT.endReadSXBuffer(); 
    measuredRSSI = LT.readPacketRSSI();      
    measuredSNR = LT.readPacketSNR();
    
    if(TXIdentity != RXIdentity) {
      char reason[50];
      sprintf(reason, "Incorrect identity %3d", TXIdentity);
      setError(reason);
    }
  } else {
    char reason[50];
    sprintf(reason, "Incorrect packet length %3d", measuredRXPacketLength);
    setError(reason);
  }
  
  if(!error) {
    TMPackets++;
    resetTMCounter = 0;
    currentSNR = measuredSNR;
    currentRSSI = measuredRSSI;
    boardVoltage = receivedValue/1000.0;
    boardCellVoltage = boardVoltage/float(BOARD_CELL_S);
  } 
}

bool receiveTMPacket(unsigned long now) {
  clearError();
  if(!checkRXIRQError()) {
    setError("IRQ Error");
    return false;
  }   
  if(!requestTM) {
    currentTMCycles = 0;
    return false;
  }
  // We cannot wait for TM forever and stop sending throttle packages. This shortcuts the TM reception routine and gets on transmitting again
  if(currentTMCycles >= maxWaitForTM/periods[2]) { 
    currentTMCycles = 0;
    requestTM = 0; 
    resetTMCounter++;
    if(resetTMCounter >= 5) {
      resetTM();
    }
    waitingForRX = false;
    forceTX = true;
    lastTMPacketReceived = now;
    setError("TM timeout");
    LT.setMode(MODE_STDBY_RC);  
    LT.config();
    return false;
  }
  if(!checkTXRXDone() || !RFAvailable) {
    currentTMCycles++;
    return false;
  }
  if(!waitingForRX) {
    waitingForRX = true; 
    #ifdef DEBUG_FLAGS       
    printFlags("Receive");
    #endif
    LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
    return false;    
  } else {    
    #ifdef DEBUG_FLAGS
    printFlags("Process TM");
    #endif
    processTMPacket();
    waitingForRX = false;
    lastTMPacketReceived = now;
    requestTM = 0;
    currentTMCycles = 0;
    return true;  
  }
}

bool checkTXRXDone() {
  uint16_t IRQStatus = LT.readIrqStatus();
  bool done = (IRQStatus & 0x4022 ) || (IRQStatus & 0x4001);   //IRQs going active
  return done;
}

bool checkRXIRQError() {
  uint16_t IRQStatus = LT.readIrqStatus();
  return !(IRQStatus & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT + IRQ_SYNCWORD_ERROR));
}

bool sendThrottlePacket(unsigned long now)
{
  if(requestTM) {
    currentTransmitCycles = 0;
    return false;
  }
  // We have been waiting for more than 60ms to send a throttle packet, so we stop everything and try again for safety.
  if(currentTransmitCycles >= maxWaitForTransmit/periods[1]) { 
    currentTransmitCycles = 0;
    forceTX = true;
    requestTM = 0;
    setError("Transmit timeout");
    LT.setMode(MODE_STDBY_RC);  
    LT.config();
    return false;
  }

  if((!RFAvailable || !checkTXRXDone()) && !forceTX) {
    currentTransmitCycles++;
    return false;
  }

  if(now - lastTMPacketReceived > TMPeriod) {
    requestTM = 1;
  }
  
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(TXIdentity);                     
  LT.writeUint16(encodedThrottleValue);  
  LT.writeUint8(requestTM);                      
  LT.endWriteSXBuffer();         
  forceTX = false;
  #ifdef DEBUG_FLAGS
  printFlags("Transmit");
  #endif
  currentTransmitCycles = 0;
  LT.transmitSXBufferIRQ(0, throttlePacketLength, 0, TXpower, NO_WAIT);  
  packets++;

  return true;                  
}

bool setLEDs(unsigned long now) {
  for(int i = 0; i < LEDS_LENGTH; i++) {
    int currentStatus = LEDPeriods[i];
    if(LEDPeriods[i] == -1) {
      LEDStatus[i] = CRGB::Black;
    } else if(LEDPeriods[i] == 0) {
      LEDStatus[i] = CRGB::White;
    } else if(now - lastLEDToggled[i] >= LEDPeriods[i]) {
      int isBlinking = LEDResetCounters[i] != 0;
      if(isBlinking) {
        LEDStatus[i] = LEDStatus[i] == CRGB::Black ? CRGB::White : CRGB::Black;
        if(LEDStatus[i] && LEDResetCounters[i] > 0) {
          LEDResetCounters[i]--;
        }
        lastLEDToggled[i] = now;
      } else {
        storedLEDStatus[i] ? setLEDOn(i) : setLEDOff(i);
      }
    }
  }
  FastLED.show();
  return true;
}

bool setMotor(unsigned long now) {
  int currentStatus = motorStatus;
  if(motorPeriod == -1) {
    motorStatus = LOW;  
  } else if (motorPeriod == 0) {
    motorStatus = HIGH;
  } else if (now - lastMotorToggled >= motorPeriod) {
    int isPulsing = motorResetCounter != 0;
    if(isPulsing) {
      motorStatus = !motorStatus;
      if(motorStatus && motorResetCounter > 0){
        motorResetCounter--;
      }
      lastMotorToggled = now;
    } else {
      motorResetCounter = -1;
      motorPeriod = -1;
    }
  }
  if(currentStatus != motorStatus) {
    digitalWrite(MOTOR, motorStatus);
  }
  return true;
}

bool printStats(unsigned long now) {
  #ifdef DEBUG
  if(errors > 0) {
    Serial.println(F("////////ERROR//////////"));
    Serial.println(errorReason);
    Serial.println(F("//////////////////////"));
  }
  float ellapsed = (now - lastRun[TASKS_LENGTH-1])/1000;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
  Serial.print(batteryVoltage);
  Serial.print(F("V | Mode: "));
  Serial.println(currentDisplayMode);
  Serial.print(F("Frequency: "));
  Serial.print(frequency);
  Serial.println(F("Hz"));
  Serial.print(F("SNR: "));
  Serial.print(currentSNR);
  Serial.print(F("dB | RSSI: "));
  Serial.print(currentRSSI);
  Serial.print(F("dBm | Board V: "));
  Serial.print(boardVoltage);
  Serial.print(F(" ("));
  Serial.print(boardCellVoltage);
  Serial.println(F(")"));
  Serial.print(F("Calibration: "));
  Serial.print(calBrake);
  Serial.print(F(" | "));
  Serial.print(centerAcc);
  Serial.print(F(" | "));
  Serial.print(calAcc);
  Serial.print(F(" | Inverted: "));
  Serial.println(inverted ? "y" : "n");
  Serial.println(F("-------------- TASKS --------------"));
  for(int i = 0; i < TASKS_LENGTH - 1; i++) {
    char prBuffer[45];
    int frequency = round(executions[i] / ellapsed);
    sprintf(prBuffer, "%-20s | %5dHz", taskNames[i], frequency);
    Serial.print(prBuffer);
    Serial.println("");
    executions[i] = 0;
  }
  Serial.println(F("-----------------------------------"));
  int packetsPerSecond = round(packets / ellapsed);
  Serial.print(F("Packets/s: "));
  Serial.println(packetsPerSecond);
  int TMPacketsPerSecond = round(TMPackets / ellapsed);
  Serial.print(F("TM packets/s: "));
  Serial.println(TMPacketsPerSecond);
  int interruptsPerSecond = round(interruptCounter / ellapsed);
  Serial.print(F("Interrupts/s: "));
  Serial.println(interruptsPerSecond);
  Serial.print(F("Errors: "));
  Serial.println(errors);
  errors = 0; 
  packets = 0;
  TMPackets = 0;
  interruptCounter = 0;
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { readThrottle, sendThrottlePacket, receiveTMPacket, checkButton, checkBattery, displayMode, setLEDs, setMotor, printStats };

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

  FastLED.addLeds<WS2812B, LED, GRB>(LEDStatus, LEDS_LENGTH);
  FastLED.setBrightness(255);
  FastLED.show();
  pinMode(PPM_THR1, INPUT);
  pinMode(ON, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  pinMode(BUTTON, INPUT);
  ONSequence();

  attachInterrupt(RFBUSY, processRFInterrupt, CHANGE);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  // readSettings();
  // if((centerAcc == calAcc && centerAcc == calBrake) || forceCalibration) {
  //   calibrate();
  //   writeSettings();
  // }

  SPI.begin();

  if (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    #ifdef DEBUG
    Serial.println(F("Device error"));
    #endif
  }

  frequency = channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;
  LT.setupLoRa(frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  LT.clearIrqStatus(IRQ_RADIO_ALL);

  #ifdef DEBUG
  Serial.println(F("Remote ready"));
  #endif
  
}
