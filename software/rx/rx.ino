#include <SPI.h>
#include <SX128XLT.h>
#include "board.h"
#include "settings.h"
#include <ProgramLT_Definitions.h>

SX128XLT LT;

#include <Servo.h>
Servo PPM; 

const int TASKS_LENGTH = 5;

char *taskNames[] = { "receiveThrottlePacket", "writePPMValue", "checkBattery", "setLEDs", "printStats" };
unsigned long periods[] = { 10, 5, 1000, 10, 1000 };
unsigned long lastRun[] = { 0, 0, 0, 0, 0 };
unsigned long executions[] = { 0, 0, 0, 0, 0 };

const int LEDS_LENGTH = 1;

int LEDPins[] = { L2, L3, L4 };
int LEDStatus[] = { LOW, LOW, LOW };
int storedLEDStatus[] = { LOW, LOW, LOW };
unsigned long LEDPeriods[] = { -1, -1, -1 };
int LEDResetCounters[] = { -1, -1, -1 };
unsigned long lastLEDToggled[] = { 0, 0, 0 };

float batteryVoltage = -1.0;
unsigned int throttleValue = 127;

volatile int rxDone = 1;
bool firstRound = true;

bool error = false;
int errors = 0;
char errorReason[50];
float currentRSSI = -100.0;
float currentSNR = -100.0;

unsigned long packets = 0;
unsigned long TMPackets = 0;
#define DEBUG

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void clearError() {
  error = false;
}

void setError(char reason[]) {
  errors++;
  error = true;
  strcpy(errorReason, reason);
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

bool checkBattery(unsigned long now) {
  int batValue = analogRead(VBAT);
  int scaledBatmVolts = map(batValue, 0, 1023, 0, 3300);
  
  batteryVoltage = (scaledBatmVolts/1000.0)*(R1+R2)/R2;
  return true;
}

ISR (PCINT2_vect) {
  rxDone = digitalRead(DIO1);
} 

void processReceivedPacket() {
  clearError();
  unsigned int TXIdentity = -1;
  unsigned int receivedValue = 127;
  unsigned int measuredRXPacketLength = LT.readRXPacketL();
  unsigned int TMRequest = 0;
  
  if(measuredRXPacketLength == throttlePacketLength){
    LT.startReadSXBuffer(0);                
    TXIdentity = LT.readUint8();         
    receivedValue = LT.readUint8();       
    TMRequest = LT.readUint8();
    LT.endReadSXBuffer(); 
    currentRSSI = LT.readPacketRSSI();      
    currentSNR = LT.readPacketSNR(); 
       
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
    packets++;
    throttleValue = receivedValue;
  }

  if(TMRequest) {
    sendTMPacket();
  }
}

bool sendTMPacket() {
  unsigned int encodedBatteryVoltage = map(round(batteryVoltage*1000), 0, 50400, 0, 254);
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(RXIdentity);                    
  LT.writeUint8(encodedBatteryVoltage);                        
  LT.endWriteSXBuffer();                    
  LT.transmitSXBuffer(0, TMPacketLength, 100, TXpower, WAIT_TX);     
  TMPackets++;
}

bool receiveThrottlePacket(unsigned long now) {
  if(!rxDone && !firstRound) {
    return false;
  }
  processReceivedPacket();
  
  firstRound = false;
  LT.receiveSXBuffer(0, 0, NO_WAIT);
  return true;
}

bool writePPMValue(unsigned long now) {
  int throttlePulse = map(throttleValue, 0, 254, 1000, 2000);
  PPM.writeMicroseconds(throttlePulse);
  return true;
}

bool setLEDs(unsigned long now) {
  for(int i = 0; i < LEDS_LENGTH; i++) {
    int currentStatus = LEDStatus[i];
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


bool printStats(unsigned long now) {
  #ifdef DEBUG
  if(errors > 0) {
    Serial.println("////////ERROR//////////");
    Serial.print(">>>> ");
    Serial.print(errorReason);
    Serial.println(" <<<<");
    Serial.println("//////////////////////");
  }
  float ellapsed = (now - lastRun[TASKS_LENGTH-1])/1000;
  Serial.print("Ellapsed: ");
  Serial.print(ellapsed);
  Serial.print("s | VBat: ");
  Serial.print(batteryVoltage);
  Serial.print("V | SNR: ");
  Serial.print(currentSNR);
  Serial.print("dB | RSSI: ");
  Serial.print(currentRSSI);
  Serial.println("dBm");
  Serial.println("-------------- TASKS --------------");
  for(int i = 0; i < TASKS_LENGTH - 1; i++) {
    char prBuffer[45];
    int frequency = round(executions[i] / ellapsed);
    sprintf(prBuffer, "%-23s | %5dHz", taskNames[i], frequency);
    Serial.print(prBuffer);
    Serial.println("");
    executions[i] = 0;
  }
  Serial.println("-----------------------------------");
  int packetsPerSecond = round(packets / ellapsed);
  Serial.print("Packets per second: ");
  Serial.println(packetsPerSecond);
  int TMPacketsPerSecond = round(TMPackets / ellapsed);
  Serial.print("TM packets per second: ");
  Serial.println(TMPacketsPerSecond);
  Serial.print("Errors: ");
  Serial.println(errors);
  errors = 0;
  packets = 0;
  TMPackets = 0;
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { receiveThrottlePacket, writePPMValue, checkBattery, setLEDs, printStats };

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
  
  PPM.attach(PPM_L1);

  #ifdef DEBUG
  Serial.println("Receiver ready");
  #endif
}
