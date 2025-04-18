#include "RF.h"

SX128XLT LT;

unsigned int resetCounter = 0;
unsigned long lastPacketTime = 0;

struct ReceptionResult { 
  bool success;
  bool TMRequest; 
};

void checkRXTimeout() {
  if(state.isConnected && (micros() - lastPacketTime) > DISCONNECT_TIMEOUT_US) {
    stats.SNR = -100;
    stats.RSSI = -100;
    state.isConnected = false;
    state.encodedThrottleValue = ENCODED_HALF;
    setError(ERROR_CODE::DISCONNECTED);
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
  if(RFAvailable) {
    if(waitFor == RX_WAIT) {
      stats.timeWaitingForRX+=ellapsed;
      stats.RXWaits++;
    } else {
      stats.timeWaitingForTX+=ellapsed;
      stats.TXWaits++;
    }
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
    setError(ERROR_CODE::IRQ_ERROR);
    return { false, false };
  } 

  LT.startReadSXBuffer(0);                
  TXIdentity = LT.readUint8();         
  receivedData = LT.readUint16();     
  LT.endReadSXBuffer(); 
  measuredRSSI = LT.readPacketRSSI();      
  measuredSNR = LT.readPacketSNR(); 
      
  if(TXIdentity != config.identity) {
    setError(ERROR_CODE::INCORRECT_IDENTITY);
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
  stats.SNR = measuredSNR;
  if(measuredSNR > stats.maxSNR) {
    stats.maxSNR = measuredSNR;
  }
  if(measuredSNR < stats.minSNR) {
    stats.minSNR = measuredSNR;
  }
  stats.RSSI = measuredRSSI;
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
    setError(ERROR_CODE::TX_TIMEOUT);
    return;
  }
  stats.TMPackets++;
}

TaskResult receiveThrottlePacket(unsigned long now) {
  checkRXTimeout();
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, THROTTLE_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  if(!waitForRFReady(RX_TIMEOUT_US, RX_WAIT)) {
    setError(ERROR_CODE::RX_TIMEOUT);
    return { false, 0 }; 
  }
  long rxWait = micros() - now;

  ReceptionResult result = processReceivedPacket();
  // Send TM only if reception was successfull, flag was set and we still have time left
  if(result.success && result.TMRequest && (rxWait + TX_TIMEOUT_US) < TOTAL_TASK_TIME) {
    sendTMPacket();
  }

  // Try to schedule next instance of this task so we are listening when the packet lands
  // Constrain the approximation to avoid overshooting
  double offset = result.success ? constrain(rxWait - TARGET_RX_WAIT, -MAX_APPROX_SLIDE_STEP_US, MAX_APPROX_SLIDE_STEP_US) : 0;
  stats.rxOffsets+=offset;
  return { true, offset };
}
