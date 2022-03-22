#include <SPI.h>
#include <SX128XLT.h>
#include "board.h"
#include "settings.h"
#include <ProgramLT_Definitions.h>

SX128XLT LT;

const int TASKS_LENGTH = 8;

char *taskNames[] = { "readThrottle", "sendThrottlePacket", "checkButton", "checkBattery", "displayMode", "setLEDs", "setMotor", "printStats" };
unsigned long periods[] = { 5, 5, 100, 1000, 50, 10, 10, 1000 };
unsigned long lastRun[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long executions[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

byte encodedThrottle1Value;

const int LEDS_LENGTH = 4;

int LEDPins[] = { PPM_L1, L2, L3, L4 };
int LEDStatus[] = { LOW, LOW, LOW, LOW };
int storedLEDStatus[] = { LOW, LOW, LOW, LOW };
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

volatile int txDone = 1;
bool firstRound = true;

#define DEBUG

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void ONSequence() {
    for(int i = LEDS_LENGTH-1; i > -1; i--) {
      digitalWrite(LEDPins[i], HIGH);
      delay(50);
    }
    digitalWrite(ON, HIGH);
    delay(100);
    digitalWrite(MOTOR, HIGH);
    delay(300);
    digitalWrite(MOTOR, LOW);
    for(int i = 0; i < LEDS_LENGTH; i++) {
      digitalWrite(LEDPins[i], LOW);
      delay(50);
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
  pulseMotor(nextDisplayMode+1, 50);
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
          pulseMotor(-1, 200);
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
  int reading = analogRead(BUTTON) > 512;
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
      delay(500);
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
    return;
  }
  int batValue = analogRead(VBAT);
  int scaledBatmVolts = map(batValue, 0, 1023, 0, 3300);
  
  batteryVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  return true;
}

bool readThrottle(unsigned long now) {
  uint16_t throttle1Value = lastButtonState ? center : analogRead(THROTTLE1);
  int scaledValue = throttle1Value > center ? map(throttle1Value, center, calMax, 127, 254) : map(throttle1Value, calMin, center, 0, 126); 
  scaledValue = constrain(scaledValue, 0, 254);
  encodedThrottle1Value = inverted ? 254 - (byte)scaledValue : (byte)scaledValue;
  return true;
}

ISR (PCINT2_vect) {
  txDone = digitalRead(DIO1);
}  

bool sendThrottlePacket(unsigned long now)
{
  if(!txDone && !firstRound) {
    return false;
  }
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(TXIdentity);                     
  LT.writeUint8(encodedThrottle1Value);                        
  LT.endWriteSXBuffer();                         


  firstRound = false;
  LT.transmitSXBuffer(0, PacketLength, 0, TXpower, NO_WAIT);    
  return true;                  
}

bool setLEDs(unsigned long now) {
  for(int i = 0; i < LEDS_LENGTH; i++) {
    int currentStatus = LEDPeriods[i];
    if(LEDPeriods[i] == -1) {
      LEDStatus[i] = LOW;
    } else if(LEDPeriods[i] == 0) {
      LEDStatus[i] = HIGH;
    } else if(now - lastLEDToggled[i] >= LEDPeriods[i]) {
      int isBlinking = LEDResetCounters[i] != 0;
      if(isBlinking) {
        LEDStatus[i] = !LEDStatus[i];
        if(LEDStatus[i] && LEDResetCounters[i] > 0) {
          LEDResetCounters[i]--;
        }
        lastLEDToggled[i] = now;
      } else {
        storedLEDStatus[i] ? setLEDOn(i) : setLEDOff(i);
      }
    }
    if(currentStatus != LEDStatus[i]) {
      digitalWrite(LEDPins[i], LEDStatus[i]);
    }
  }
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
  float ellapsed = (now - lastRun[TASKS_LENGTH-1])/1000;
  Serial.print("Ellapsed: ");
  Serial.print(ellapsed);
  Serial.print("s | VBat: ");
  Serial.print(batteryVoltage);
  Serial.print("V | Mode: ");
  Serial.println(currentDisplayMode);
  Serial.println("-------------- TASKS --------------");
  for(int i = 0; i < TASKS_LENGTH - 1; i++) {
    char prBuffer[45];
    int frequency = round(executions[i] / ellapsed);
    sprintf(prBuffer, "%-20s | %5dHz", taskNames[i], frequency);
    Serial.print(prBuffer);
    Serial.println("");
    executions[i] = 0;
  }
  Serial.println("-----------------------------------");
  Serial.println("");
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { readThrottle, sendThrottlePacket, checkButton, checkBattery, displayMode, setLEDs, setMotor, printStats };

void loop()
{
  for(int i = 0; i < TASKS_LENGTH; i++) {
    unsigned long now = millis();
    if(now - lastRun[i] >= periods[i]) {
      if(tasks[i](now)) {
        executions[i]++;
        lastRun[i] = millis();
      }
    }
  }
}


void setup()
{
  pinMode(PPM_L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
  pinMode(ON, OUTPUT);
  pinMode(MOTOR, OUTPUT);
  ONSequence();
  pciSetup(DIO1);

  #ifdef DEBUG
  Serial.begin(19200);
  #endif

  SPI.begin();
  

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    #ifdef DEBUG
    Serial.println(F("LoRa ready"));
    #endif
  }
  else
  {
    #ifdef DEBUG
    Serial.println(F("Device error"));
    #endif
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  
  #ifdef DEBUG
  Serial.println(F("Remote ready"));
  #endif
  
}
