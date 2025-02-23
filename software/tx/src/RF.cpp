#include "RF.h"

SX128XLT LT;

volatile int RFAvailable = 1;
bool forceTX = true;

unsigned long TMPeriod = 500;
unsigned long lastTMPacketReceived = 0;
unsigned int requestTM = 0;
bool waitingForRX = false;
unsigned int maxWaitForTM = 40;
unsigned int currentTMCycles = 0;
unsigned int currentTransmitCycles = 0;
unsigned int maxWaitForTransmit = 20;
int resetTMCounter = 0;

unsigned long frequency = config.channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;

void resetTM() {
  state.currentSNR = -100;
  state.currentRSSI = -100;
  state.boardVoltage = 0.0;
  state.boardCellVoltage = 0.0;
}


void IRAM_ATTR processRFInterrupt() {
  RFAvailable = !digitalRead(RFBUSY);
  state.interruptCounter++;
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
    
    if(config.TXIdentity != RXIdentity) {
      char reason[50];
      sprintf(reason, "Incorrect identity %3d", config.TXIdentity);
      setError(reason);
    }
  } else {
    char reason[50];
    sprintf(reason, "Incorrect packet length %3d", measuredRXPacketLength);
    setError(reason);
  }
  
  if(!state.error) {
    state.TMPackets++;
    resetTMCounter = 0;
    state.currentSNR = measuredSNR;
    state.currentRSSI = measuredRSSI;
    state.boardVoltage = receivedValue/1000.0;
    state.boardCellVoltage = state.boardVoltage/float(config.cellN);
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
  if(currentTMCycles >= maxWaitForTM/periods[1]) { 
    currentTMCycles = 0;
    requestTM = 0; 
    resetTMCounter++;
    if(resetTMCounter >= 5) {
      resetTM();
    }
    waitingForRX = false;
    forceTX = true;
    lastTMPacketReceived = now;
    state.isConnected = false;
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
    LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
    return false;    
  } else {    
    processTMPacket();
    waitingForRX = false;
    lastTMPacketReceived = now;
    state.isConnected = true;
    requestTM = 0;
    currentTMCycles = 0;
    return true;  
  }
}

bool sendThrottlePacket(unsigned long now) {
  if(requestTM) {
    currentTransmitCycles = 0;
    return false;
  }
  // We have been waiting for more than 20ms to send a throttle packet, so we stop everything and try again for safety.
  if(currentTransmitCycles >= maxWaitForTransmit/periods[0]) { 
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
  LT.writeUint8(config.TXIdentity);                     
  LT.writeUint16(state.encodedThrottleValue);  
  LT.writeUint8(requestTM);                      
  LT.endWriteSXBuffer();         
  forceTX = false;
  currentTransmitCycles = 0;
  LT.transmitSXBufferIRQ(0, throttlePacketLength, 0, TXpower, NO_WAIT);  
  state.packets++;

  return true;                  
}