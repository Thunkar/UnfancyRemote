#include "RF.h"

SX128XLT LT;

unsigned int throttleMask = 0xFFF;
unsigned int TMRequestMask = 0x1000;

#define RX_WAIT 0
#define TX_WAIT 1

#define RX_IRQ_MASK IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT
#define TX_IRQ_MASK IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT

unsigned int resetCounter = 0;
const unsigned int MAX_RESET_COUNTER = 200 / 20; // 200ms desired timeout / 20ms per expected packet period

void connectionReset() {
  resetCounter++;
  if(resetCounter >= MAX_RESET_COUNTER) {
    setError("Connection lost");
    state.currentSNR = -100;
    state.currentRSSI = -100;
    state.isConnected = false;
    state.encodedThrottleValue = ENCODED_HALF;
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

bool processReceivedPacket() {
  if(!checkRXIRQError()) {
    setError("IRQ Error");
    return false;
  }                                               
  unsigned int TXIdentity = -1;
  unsigned int receivedData = ENCODED_HALF;
  bool TMRequest = false;
  int measuredSNR = 0;
  long measuredRSSI = 0;
  
  LT.startReadSXBuffer(0);                
  TXIdentity = LT.readUint8();         
  receivedData = LT.readUint16();     
  LT.endReadSXBuffer(); 
  measuredRSSI = LT.readPacketRSSI();      
  measuredSNR = LT.readPacketSNR(); 
      
  if(TXIdentity != config.identity) {
    char reason[30];
    sprintf(reason, "Incorrect identity %3d", TXIdentity);
    setError(reason);
  }
  
  if(!state.error) {
    state.isConnected = true;
    stats.packets++;
    state.currentSNR = measuredSNR;
    state.currentRSSI = measuredRSSI;
    state.encodedThrottleValue = (receivedData & throttleMask);
    resetCounter = 0;
    TMRequest = (receivedData & TMRequestMask) >> 12;
  }
  return TMRequest;
}

void sendTMPacket() {
  LT.startWriteSXBuffer(0);             
  unsigned int boardVoltageAsInt = roundAndCastToInt(state.boardVoltage);
  unsigned int encodedBoardVoltage = map(boardVoltageAsInt, 0, 420 * config.cellN, 0, 255) << 8;        
  LT.writeUint16(encodedBoardVoltage+config.identity);                            
  LT.endWriteSXBuffer();   
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, TM_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.transmitSXBufferIRQ(0, TM_PACKET_LENGTH, 0, TX_POWER, NO_WAIT);  
  if(!waitForRFReady(5, TX_WAIT)) {
    setError("TX timeout");
    return;
  }
  stats.TMPackets++;
}

bool receiveThrottlePacket(unsigned long now) {
  clearError();
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, THROTTLE_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  if(!waitForRFReady(5, RX_WAIT)) {
    connectionReset();
    return false;
  }
  bool TMRequest = processReceivedPacket();
  if(TMRequest) {
    sendTMPacket();
  }
  return true;
}
