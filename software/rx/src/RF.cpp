#include "RF.h"

SX128XLT LT;

unsigned long frequency = config.channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;

unsigned int throttleMask = 0xFFF;
unsigned int TMRequestMask = 0x1000;

#define RX_IRQ_MASK 0x4022
#define TX_IRQ_MASK 0x4001

unsigned int resetCounter = 0;

void connectionReset() {
  resetCounter++;
  if(resetCounter >= 10) {
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

bool waitForRFReady(long timeoutMs, uint16_t IRQMask) {
  long timeout = timeoutMs*1000;
  long ellapsed = 0;
  bool RFAvailable = false;
  unsigned long start = micros();
  while (!RFAvailable && (timeout-ellapsed) > 0) {
    RFAvailable = checkRFBusy() && checkRFDone(IRQMask);
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

bool processReceivedPacket() {
  if(!checkRXIRQError()) {
    setError("IRQ Error");
    LT.clearIrqStatus(IRQ_RADIO_ALL);
    return false;
  }                                               
  unsigned int TXIdentity = -1;
  unsigned int receivedData = ENCODED_HALF;
  unsigned int measuredRXPacketLength = LT.readRXPacketL();
  bool TMRequest = false;
  int measuredSNR = 0;
  long measuredRSSI = 0;
  
  if(measuredRXPacketLength == throttlePacketLength){
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
  } else {
    char reason[30];
    sprintf(reason, "Incorrect packet length %3d", measuredRXPacketLength);
    setError(reason);
  }
  
  if(!state.error) {
    state.isConnected = true;
    state.packets++;
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
  LT.transmitSXBufferIRQ(0, TMPacketLength, 0, TXpower, NO_WAIT);  
  if(!waitForRFReady(10, TX_IRQ_MASK)) {
    setError("TX timeout");
    return;
  }
  state.TMPackets++;
}

bool receiveThrottlePacket(unsigned long now) {
  clearError();
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  if(!waitForRFReady(10, RX_IRQ_MASK)) {
    connectionReset();
    return false;
  }
  bool TMRequest = processReceivedPacket();
  if(TMRequest) {
    sendTMPacket();
  }
  return true;
}
