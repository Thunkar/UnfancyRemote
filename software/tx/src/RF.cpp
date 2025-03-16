#include "RF.h"

SX128XLT LT;

unsigned long frequency = config.channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;


unsigned long TMPeriod = 500;
unsigned long lastTMPacketReceived = 0;

unsigned int resetTMCounter = 0;

unsigned int RXIdentityMask = 0xFF;
unsigned int batteryVoltageMask = 0xFF00;

#define RX_IRQ_MASK 0x4022
#define TX_IRQ_MASK 0x4001

void resetTM() {
  state.boardVoltage = 0.0;
  state.boardCellVoltage = 0.0;
  state.isConnected = false;
}

void hardReset() {
  resetTMCounter++;
  if(resetTMCounter >= 5) {
    resetTM();
  }
  LT.config();
}

bool checkRFDone(uint16_t IRQMask) {
  uint16_t IRQStatus = LT.readIrqStatus();
  return IRQStatus & IRQMask;
}

bool waitForRFReady(long timeoutMs, uint16_t IRQMask) {
  long timeout = timeoutMs*1000;
  long ellapsed = 0;
  bool RFAvailable = false;
  unsigned long start = micros();
  while (!RFAvailable && (timeout-ellapsed) > 0) {
    RFAvailable = !digitalRead(RFBUSY) && checkRFDone(IRQMask);
    ellapsed = micros() - start;
  }
  LT.setMode(MODE_STDBY_RC);
  state.waitingForRF+=ellapsed;
  state.RFWaits++;
  return RFAvailable;
}

bool checkRXIRQError() {
  uint16_t IRQStatus = LT.readIrqStatus();
  return !(IRQStatus & (IRQ_HEADER_ERROR + IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT + IRQ_SYNCWORD_ERROR));
}

void processTMPacket() {    
  if(!checkRXIRQError()) {
    setError("IRQ Error");
    return;
  }   
  unsigned int RXIdentity = -1;
  unsigned int receivedData = 0;

  unsigned int measuredRXPacketLength = LT.readRXPacketL();
  int measuredSNR = 0;
  long measuredRSSI = 0;
  if(measuredRXPacketLength == TMPacketLength){
    LT.startReadSXBuffer(0);                
    receivedData = LT.readUint16();
    RXIdentity = receivedData & RXIdentityMask;
    LT.endReadSXBuffer(); 
    
    if(config.identity != RXIdentity) {
      char reason[50];
      sprintf(reason, "Incorrect identity %3d", config.identity);
      setError(reason);
    }
  } else {
    char reason[50];
    sprintf(reason, "Incorrect packet length %3d", measuredRXPacketLength);
    setError(reason);
  }
  
  if(!state.error) {
    state.TMPackets++;
    unsigned int decodedBatteryVoltage = (receivedData & batteryVoltageMask) >> 8;
    state.boardVoltage = map(decodedBatteryVoltage, 0, 255, 0, config.cellN * 420)/100.0;
    state.boardCellVoltage = state.boardVoltage/float(config.cellN);
    state.isConnected = true;
    resetTMCounter = 0;
  } 
}

void receiveTMPacket() {
  clearError();
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  if(!waitForRFReady(20, RX_IRQ_MASK)) {
    setError("RX timeout");
    hardReset();
    return;
  }
  processTMPacket();
}

bool sendThrottlePacket(unsigned long now) {
  clearError();
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(config.identity); 
  bool requestTM = (now - lastTMPacketReceived) > TMPeriod;
  unsigned int encodedData = (requestTM << 12) + state.encodedThrottleValue;                   
  LT.writeUint16(encodedData);          
  LT.endWriteSXBuffer();       
  LT.transmitSXBufferIRQ(0, throttlePacketLength, 0, TXpower, NO_WAIT);  
  if(requestTM) {
    if(!waitForRFReady(15, TX_IRQ_MASK)) {
      setError("TX timeout");
      return false;
    }
    receiveTMPacket();
    lastTMPacketReceived = now;
  }
  state.packets++;
  return true;                  
}