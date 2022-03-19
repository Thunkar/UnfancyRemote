#include <SPI.h>
#include <SX128XLT.h>
#include "board.h"
#include "settings.h"
#include <ProgramLT_Definitions.h>

SX128XLT LT;

const int TASKS_LENGTH = 8;

char *taskNames[] = { "readThrottle", "sendThrottlePacket", "checkButton", "checkBattery", "displayMode", "setLEDs", "setMotor", "printStats" };
long periods[] = { 20, 20, 100, 1000, 50, 5, 5, 5000 };
unsigned long lastRun[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned long executions[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

byte encodedThrottle1Value;

const int LEDS_LENGTH = 4;

int LEDPins[] = { PPM_L1, L2, L3, L4 };
int LEDStatus[] = { LOW, LOW, LOW, LOW };
int storedLEDStatus[] = { LOW, LOW, LOW, LOW };
int LEDPeriods[] = { 0, 0, 0, 0 };
int LEDResetCounters[] = { -1, -1, -1, -1 };
int lastLEDToggled[] = { 0, 0, 0, 0 };

int motorStatus = LOW;
int motorPeriod = -1;
int lastMotorToggled = 0;
int motorResetCounter = 0;
 
int lastButtonState = LOW;  

unsigned long lastPressedTime = 0;
unsigned long debounceDelay = 100;
unsigned long offDelay = 1500;    
unsigned long changeModeDelay = 500;

float batteryVoltage = 0.0;

int lastDisplayMode = 0;
int currentDisplayMode = 0; // -1 transition, 0 board voltage, 1 remote voltage
int nextDisplayMode = 0;
unsigned long transitionDelay = 250;
unsigned long lastTransitionCheck = 0;

#define DEBUG

void ONSequence() {
    digitalWrite(L4, HIGH);
    delay(100);
    digitalWrite(L3, HIGH);
    delay(100);
    digitalWrite(L2, HIGH);
    delay(100);
    digitalWrite(L3, HIGH);
    delay(100);
    digitalWrite(PPM_L1, HIGH);
    digitalWrite(ON, HIGH);
    delay(100);
    digitalWrite(MOTOR, HIGH);
    delay(300);
    digitalWrite(MOTOR, LOW);
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
  for(int i = 0; i < LEDS_LENGTH; i++) {
    setLEDOff(i);
  }
  pulseMotor(2, 50);
  nextDisplayMode = mode;
}

void displayMode(unsigned long now) {
  if(nextDisplayMode != lastDisplayMode) {
    if(now - lastTransitionCheck > transitionDelay) {
      currentDisplayMode = nextDisplayMode;
      lastDisplayMode = nextDisplayMode;
    } else {
      currentDisplayMode = -1;
      lastTransitionCheck = now;
    }
  }
  switch(currentDisplayMode) {
    case 0: {
  
    }
    case 1: {
    
    }
  }
}

void checkButton(unsigned long now) {
  int buttonState;
  int reading = analogRead(BUTTON) > 512;
  if (reading != lastButtonState) {
    lastPressedTime = now;
  }

  if((now - lastPressedTime > debounceDelay) && reading != buttonState) {
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
    if(buttonState && currentDisplayMode != -1) {
      changeMode(!currentDisplayMode);
    }
  }

  lastButtonState = reading;
}

void checkBattery(unsigned long now) {
  int batValue = analogRead(VBAT);
  int scaledBatmVolts = map(batValue, 0, 1023, 0, 3300);
  
  batteryVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
}

void readThrottle(unsigned long now) {

  uint16_t throttle1Value = analogRead(THROTTLE1);
  int scaledValue = throttle1Value > center ? map(throttle1Value, center, calMax, 127, 254) : map(throttle1Value, calMin, center, 0, 126); 
  scaledValue = constrain(scaledValue, 0, 254);
  encodedThrottle1Value = inverted ? 254 - (byte)scaledValue : (byte)scaledValue;
/*
#ifdef DEBUG
  Serial.print(F("Raw,"));
  Serial.print(throttle1Value);
  Serial.print(F(",Scaled,"));
  Serial.print(scaledValue);
  Serial.print(F(",Encoded,"));
  Serial.print(encodedThrottle1Value);
#endif
*/
}

void sendThrottlePacket(unsigned long now)
{
  LT.startWriteSXBuffer(0);                      
  LT.writeUint8(RControl1);                     
  LT.writeUint8(TXIdentity);                     
  LT.writeUint8(encodedThrottle1Value);                        
  LT.endWriteSXBuffer();                         

  uint8_t TXPacketL = LT.transmitSXBuffer(0, PacketLength, 10000, TXpower, WAIT_TX);

  if (!TXPacketL)
  {
    Serial.print(F("Send Error - IRQreg,"));
    Serial.print(LT.readIrqStatus(), HEX);
  }                           
}

void setLEDs(unsigned long now) {
  for(int i = 0; i < LEDS_LENGTH; i++) {
    if(LEDPeriods[i] == -1) {
      LEDStatus[i] = LOW;
    } else if(LEDPeriods[i] == 0) {
      LEDStatus[i] = HIGH;
    } else if(now - lastLEDToggled[i] > LEDPeriods[i]) {
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
    digitalWrite(LEDPins[i], LEDStatus[i]);
  }
}

void setMotor(unsigned long now) {
  if(motorPeriod == -1) {
    motorStatus = LOW;  
  } else if (motorPeriod == 0) {
    motorStatus = HIGH;
  } else if (now - lastMotorToggled > motorPeriod) {
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
  digitalWrite(MOTOR, motorStatus);
}

void printStats(unsigned long now) {
  int ellapsed = now - lastRun[TASKS_LENGTH-1];
  Serial.print("----- VBat: ");
  Serial.print(batteryVoltage);
  Serial.print("V | Mode: ");
  Serial.print(currentDisplayMode);
  Serial.println(" -----");
  Serial.println("-------------- TASKS --------------");
  for(int i = 0; i < TASKS_LENGTH - 1; i++) {
    char prBuffer[45];
    int frequency = round(executions[i]*1000 / float(ellapsed));
    sprintf(prBuffer, "%-20s | %5dHz", taskNames[i], frequency);
    Serial.print(prBuffer);
    Serial.println("");
    executions[i] = 0;
  }
  Serial.println("-----------------------------------");
  Serial.println("");
}

typedef void (*task)(unsigned long);

task tasks[] = { readThrottle, sendThrottlePacket, checkButton, checkBattery, displayMode, setLEDs, setMotor, printStats };

void loop()
{
  for(int i = 0; i < TASKS_LENGTH; i++) {
    unsigned long now = millis();
    if(now - lastRun[i] > periods[i]) {
      tasks[i](now);
      executions[i]++;
      lastRun[i] = now;
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

  Serial.begin(19200);

  SPI.begin();
  

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("LoRa ready"));
  }
  else
  {
    Serial.println(F("Device error"));
  }

  LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  Serial.println(F("Remote ready"));
  
}
