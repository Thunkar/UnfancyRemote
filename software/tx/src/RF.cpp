#include "RF.h"

SX128XLT LT;

unsigned long TMPeriod = 500;
unsigned long lastTMPacketAttempt = 0;
unsigned long lastTMPacketReceived = 0;

unsigned int resetTMCounter = 0;

unsigned int RXIdentityMask = 0xFF;
unsigned int batteryVoltageMask = 0xFF00;

#define RX_WAIT 0
#define TX_WAIT 1

#define RX_IRQ_MASK IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
#define TX_IRQ_MASK IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT

const unsigned int RX_TIMEOUT = 5000;

void checkRXTimeout() {
  if((micros() - lastTMPacketReceived) > RX_TIMEOUT*1000) {
    state.boardVoltage = 0.0;
    state.boardCellVoltage = 0.0;
    state.isConnected = false;
  }
}

bool checkRFBusy() {
  return !digitalRead(RFBUSY);
}

bool checkRFDone(uint16_t IRQMask) {
  uint16_t IRQStatus = LT.readIrqStatus();
  return IRQStatus & IRQMask;
}

bool waitForRFReady(long timeoutMs, int waitFor) {
  long timeout = timeoutMs*1000;
  long ellapsed = 0;
  bool RFAvailable = false;
  unsigned long start = micros();
  uint16_t IRQMask = waitFor == RX_WAIT ? RX_IRQ_MASK : TX_IRQ_MASK;
  bool notBusy = false;
  while (!RFAvailable && (timeout-ellapsed) > 0) {
    notBusy = notBusy || checkRFBusy();
    RFAvailable = notBusy && checkRFDone(IRQMask);
    ellapsed = micros() - start;
  }
  LT.setMode(MODE_STDBY_RC);
  if(waitFor == RX_WAIT) {
    stats.timeWaitingForRX+=ellapsed;
    stats.RXWaits++;
  } else {
    stats.timeWaitingForTX+=ellapsed;
    stats.TXWaits++;
  }
  return RFAvailable;
}

bool checkRXIRQError() {
  uint16_t IRQStatus = LT.readIrqStatus();
  return !(IRQStatus & (IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT + IRQ_SYNCWORD_ERROR));
}

void processTMPacket() {    
  unsigned int RXIdentity = -1;
  unsigned int receivedData = 0;

  if(!checkRXIRQError()) {
    setError("IRQ Error");
  } else {
    LT.startReadSXBuffer(0);                
    receivedData = LT.readUint16();
    RXIdentity = receivedData & RXIdentityMask;
    LT.endReadSXBuffer(); 
    
    if(config.identity != RXIdentity) {
      char reason[50];
      sprintf(reason, "Incorrect identity %3d", config.identity);
      setError(reason);
    }
  } 
  
  if(!state.error) {
    stats.TMPackets++;
    unsigned int decodedBatteryVoltage = (receivedData & batteryVoltageMask) >> 8;
    state.boardVoltage = map(decodedBatteryVoltage, 0, 255, 0, config.cellN * 420)/100.0;
    state.boardCellVoltage = state.boardVoltage/float(config.cellN);
    state.isConnected = true;
    resetTMCounter = 0;
    lastTMPacketReceived = micros();
  }
}

void receiveTMPacket() {
  clearError();
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, TM_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  if(!waitForRFReady(3, RX_WAIT)) {
    setError("RX timeout");
  }
  processTMPacket();
}

bool sendThrottlePacket(unsigned long now) {
  checkRXTimeout();
  clearError();
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(config.identity); 
  bool requestTM = (now - lastTMPacketAttempt) > TMPeriod;
  unsigned int encodedData = (requestTM << 12) + state.encodedThrottleValue;                   
  LT.writeUint16(encodedData);          
  LT.endWriteSXBuffer();     
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, THROTTLE_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);  
  LT.transmitSXBufferIRQ(0, THROTTLE_PACKET_LENGTH, 0, TX_POWER, NO_WAIT);  
  if(requestTM) {
    if(!waitForRFReady(3, TX_WAIT)) {
      setError("TX timeout");
      return false;
    }
    receiveTMPacket();
    lastTMPacketAttempt = now;
  }
  stats.packets++;
  return true;                  
}