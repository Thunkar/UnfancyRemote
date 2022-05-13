#include <SPI.h>
#include <SX128XLT.h>
#include "board.h"
#include "settings.h"
#include <ProgramLT_Definitions.h>

SX128XLT LT;

#include <Servo.h>
Servo PPM; 

const int TASKS_LENGTH = 6;

char *taskNames[] = { "receiveThrottlePacket", "writePPMValue", "sendTMPacket", "checkBattery", "setLEDs", "printStats" };
unsigned long periods[] = { 5, 20, 5, 1000, 100, 2000 };
unsigned long lastRun[] = { 0, 0, 0, 0, 0, 0 };
unsigned long executions[] = { 0, 0, 0, 0, 0, 0 };

const int LEDS_LENGTH = 3;

int LEDPins[] = { L2, L3, L4 };
int LEDStatus[] = { LOW, LOW, LOW };
int storedLEDStatus[] = { LOW, LOW, LOW };
unsigned long LEDPeriods[] = { -1, -1, -1 };
int LEDResetCounters[] = { -1, -1, -1 };
unsigned long lastLEDToggled[] = { 0, 0, 0 };

const unsigned int ENCODED_MAX = 65535;
const unsigned int ENCODED_HALF = 32768;
unsigned int batteryVoltage = 0;
unsigned int throttleValue = ENCODED_HALF;

volatile int RFAvailable = 1;
bool forceRX = true;

bool error = false;
int errors = 0;
char errorReason[30];
long currentRSSI = -100;
int currentSNR = -100;
unsigned long frequency;

unsigned long packets = 0;
unsigned long TMPackets = 0;
unsigned int TMRequest = 0;
bool waitingForRX = false;
unsigned int maxWaitForReceive = 250;
unsigned int currentReceiveCycles = 0;

int VCC = 0;

#define DEBUG
//#define DEBUG_FLAGS

void printFlags(char title[]) {
  Serial.print(title);
  Serial.print(F(" | RFAvailable: "));
  Serial.print(RFAvailable);
  Serial.print(F(" | TMRequest: "));
  Serial.println(TMRequest);
}

int readVcc(void) {
   int result;
   ADCSRA = (1<<ADEN);  //enable and
   ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);  // set prescaler to 128
  // set the reference to Vcc and the measurement to the internal 1.1V reference
   ADMUX = (1<<REFS0) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1);
   delay(1); // Wait for ADC and Vref to settle
   ADCSRA |= (1<<ADSC); // Start conversion
   while (bit_is_set(ADCSRA,ADSC)); // wait until done
   result = ADC;
   // second time is a charm
   ADCSRA |= (1<<ADSC); // Start conversion
   while (bit_is_set(ADCSRA,ADSC)); // wait until done
   result = ADC;
   // must be individually calibrated for EACH BOARD
   result = VREF / (unsigned long)result; //1126400 = 1.1*1024*1000
   return result; // Vcc in millivolts
}


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

void ONSequence() {
    for(int i = LEDS_LENGTH-1; i > -1; i--) {
      digitalWrite(LEDPins[i], HIGH);
      delay(100);
    }
    delay(100);
    for(int i = 0; i < LEDS_LENGTH; i++) {
      digitalWrite(LEDPins[i], LOW);
      delay(50);
    }
    VCC = readVcc();
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
  int scaledBatmVolts = map(batValue, 0, 1023, 0, VCC);
  batteryVoltage = scaledBatmVolts*(R1+R2)/R2;
  return true;
}

ISR (PCINT2_vect) {
  RFAvailable = digitalRead(DIO1);
} 

void processReceivedPacket() {
  clearError();
  unsigned int TXIdentity = -1;
  unsigned int receivedValue = ENCODED_HALF;
  unsigned int receivedTMRequest = 0;
  unsigned int measuredRXPacketLength = LT.readRXPacketL();
  int measuredSNR = 0;
  long measuredRSSI = 0;
  
  if(measuredRXPacketLength == throttlePacketLength){
    LT.startReadSXBuffer(0);                
    TXIdentity = LT.readUint8();         
    receivedValue = LT.readUint16();       
    receivedTMRequest = LT.readUint8();
    LT.endReadSXBuffer(); 
    measuredRSSI = LT.readPacketRSSI();      
    measuredSNR = LT.readPacketSNR(); 
       
    if(TXIdentity != RXIdentity) {
      char reason[50];
      sprintf(reason, "Incorrect identity %3d", TXIdentity);
      setError(reason);
    }
  } else {
    char reason[30];
    sprintf(reason, "Incorrect packet length %3d", measuredRXPacketLength);
    setError(reason);
  }
  
  if(!error) {
    packets++;
    currentSNR = measuredSNR;
    currentRSSI = measuredRSSI;
    throttleValue = receivedValue;
    TMRequest = receivedTMRequest;
  }
}

bool sendTMPacket(unsigned long now) {
  if(!TMRequest || !RFAvailable) {
    return false;
  }
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(RXIdentity);                    
  LT.writeUint16(batteryVoltage);                        
  LT.endWriteSXBuffer();   
  #ifdef DEBUG_FLAGS                 
  printFlags("Transmit");
  #endif
  LT.transmitSXBuffer(0, TMPacketLength, 0, TXpower, NO_WAIT);  
  TMPackets++;
  TMRequest = 0;
  return true;
}

bool receiveThrottlePacket(unsigned long now) {
  if(TMRequest) {
    currentReceiveCycles = 0;
    return false;
  }
  if(currentReceiveCycles >= maxWaitForReceive/periods[0]) { // Excluding tm receives, we have been waiting for more than 40ms for a throttle packet
    currentReceiveCycles = 0;
    waitingForRX = false;
    currentSNR = -100;
    currentRSSI = -100;
    throttleValue = ENCODED_HALF;
    setError("Receive timeout");
    return false;
  }
  if(!RFAvailable && !forceRX) {
    currentReceiveCycles++;
    return false;
  }
  forceRX = false;
  if(!waitingForRX) {
    waitingForRX = true;
    #ifdef DEBUG_FLAGS  
    printFlags("Receive");
    #endif
    LT.receiveSXBuffer(0, 0, NO_WAIT);
    return false;
  } else {
    #ifdef DEBUG_FLAGS   
    printFlags("Process throttle");
    #endif
    processReceivedPacket();
    currentReceiveCycles = 0;
    waitingForRX = false;
    return true;
  }
}

bool writePPMValue(unsigned long now) {
  unsigned int throttlePulse = map(throttleValue, 0, ENCODED_MAX, 1000, 2000);
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
    Serial.println(F("////////ERROR//////////"));
    Serial.println(errorReason);
    Serial.println(F("//////////////////////"));
  }
  Serial.print(F("Frequency: "));
  Serial.print(frequency);
  Serial.println(F("Hz"));
  float ellapsed = (now - lastRun[TASKS_LENGTH-1])/1000;
  Serial.print(F("Ellapsed: "));
  Serial.print(ellapsed);
  Serial.print(F("s | VBat: "));
  Serial.print(batteryVoltage/1000.0);
  Serial.print(F("V | SNR: "));
  Serial.print(currentSNR);
  Serial.print(F("dB | RSSI: "));
  Serial.print(currentRSSI);
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
  int packetsPerSecond = round(packets / ellapsed);
  Serial.print(F("Packets per second: "));
  Serial.println(packetsPerSecond);
  int TMPacketsPerSecond = round(TMPackets / ellapsed);
  Serial.print(F("TM packets per second: "));
  Serial.println(TMPacketsPerSecond);
  Serial.print(F("Errors: "));
  Serial.println(errors);
  errors = 0;
  packets = 0;
  TMPackets = 0;
  #endif
  return true;
}

typedef bool (*task)(unsigned long);

task tasks[] = { receiveThrottlePacket, writePPMValue, sendTMPacket, checkBattery, setLEDs, printStats };

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
  pinMode(PPM_L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(L3, OUTPUT);
  pinMode(L4, OUTPUT);
  ONSequence();
  pciSetup(DIO1);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  SPI.begin();

  if (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    #ifdef DEBUG
    Serial.println(F("Device error"));
    #endif
  }

  frequency = channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;
  LT.setupLoRa(frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  
  PPM.attach(PPM_L1);

  #ifdef DEBUG
  Serial.println(F("Receiver ready"));
  #endif
}
