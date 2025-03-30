#include "RF.h"

SX128XLT LT;

unsigned int resetCounter = 0;
unsigned long lastPacketTime = 0;

struct ReceptionResult { 
  bool success;
  bool TMRequest; 
};

void checkRXTimeout() {
  if((micros() - lastPacketTime) > DISCONNECT_TIMEOUT_US) {
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

bool waitForRFReady(long timeout, int waitFor) {
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

ReceptionResult processReceivedPacket() {
  unsigned int TXIdentity = -1;
  unsigned int receivedData = ENCODED_HALF;
  bool TMRequest = false;
  int measuredSNR = 0;
  long measuredRSSI = 0;

  if(!checkRXIRQError()) {
    setError("IRQ Error");
    return { false, false };
  } 

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
    return { false, false };
  }

  unsigned long now = micros();
  unsigned long ellapsed = now - lastPacketTime;
  stats.packets++;
  stats.packetTimes+=ellapsed;
  if(stats.maxPacketTime < ellapsed) {
    stats.maxPacketTime = ellapsed;
  }
  if(stats.minPacketTime > ellapsed) {
    stats.minPacketTime = ellapsed;
  }
  lastPacketTime = now;
  state.isConnected = true;
  state.currentSNR = measuredSNR;
  state.currentRSSI = measuredRSSI;
  state.encodedThrottleValue = (receivedData & THROTTLE_MASK);
  resetCounter = 0;
  TMRequest = (receivedData & TM_REQUEST_MASK) >> 12;

  return { true, TMRequest };
}

void sendTMPacket() {
  LT.startWriteSXBuffer(0);             
  unsigned int boardVoltageAsInt = roundAndCastToInt(state.boardVoltage);
  unsigned int encodedBoardVoltage = map(boardVoltageAsInt, 0, 420 * config.cellN, 0, 255) << 8;        
  LT.writeUint16(encodedBoardVoltage+config.identity);                            
  LT.endWriteSXBuffer();   
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, TM_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.transmitSXBufferIRQ(0, TM_PACKET_LENGTH, 0, TX_POWER, NO_WAIT);  
  if(!waitForRFReady(TX_TIMEOUT_US, TX_WAIT)) {
    setError("TX timeout");
    return;
  }
  stats.TMPackets++;
}

TaskResult receiveThrottlePacket(unsigned long now) {
  checkRXTimeout();
  clearError();
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, THROTTLE_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  if(!waitForRFReady(RX_TIMEOUT_US, RX_WAIT)) {
    return { false, !state.isConnected ? -1e3 : 0 }; // Slide the reception window if disconnected
  }
  long rxWait = micros() - now;

  ReceptionResult result = processReceivedPacket();
  if(result.success && result.TMRequest) {
    sendTMPacket();
  }

  // Try to schedule next task so the packet lands in the middle of the reception window
  double offset = result.success && (rxWait != RECEPTION_TIME_TARGET_US) ? (rxWait - RECEPTION_TIME_TARGET_US) : 0;
  stats.rxOffsets+=offset;
  return { true, offset };
}
