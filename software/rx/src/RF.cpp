#include "RF.h"

SX128XLT LT;

volatile int RFAvailable = 1;
volatile int interruptCounter = 1;
bool forceRX = true;

unsigned long frequency = config.channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;

bool TMRequest = 0;
bool waitingForRX = false;
unsigned int maxWaitForReceive = 100;
unsigned int currentReceiveCycles = 0;

unsigned int throttleMask = 0xFFF;
unsigned int TMRequestMask = 0x1000;

void IRAM_ATTR processRFInterrupt() {
  RFAvailable = !digitalRead(RFBUSY);
  interruptCounter++;
}

void processReceivedPacket() {
  clearError();
  if(!checkRXIRQError()) {
    setError("IRQ Error");
    return;
  }                                               
  unsigned int TXIdentity = -1;
  unsigned int receivedData = ENCODED_HALF;
  unsigned int measuredRXPacketLength = LT.readRXPacketL();
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
    TMRequest = (receivedData & TMRequestMask) >> 12;
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

bool sendTMPacket(unsigned long now) {
  if(!TMRequest || !RFAvailable || !checkTXRXDone()) {
    return false;
  }
  LT.startWriteSXBuffer(0);             
  unsigned int boardVoltageAsInt = roundAndCastToInt(state.boardVoltage);
  unsigned int encodedBoardVoltage = map(boardVoltageAsInt, 0, 420 * config.cellN, 0, 255) << 8;        
  LT.writeUint16(encodedBoardVoltage+config.identity);                            
  LT.endWriteSXBuffer();   
  LT.transmitSXBufferIRQ(0, TMPacketLength, 0, TXpower, NO_WAIT);  
  state.TMPackets++;
  TMRequest = 0;
  return true;
}

bool receiveThrottlePacket(unsigned long now) {
  if(TMRequest) {
    currentReceiveCycles = 0;
    return false;
  }
  // Excluding tm receives, we have been waiting for more than 50ms for a throttle packet. Reset everything and try again!
  if(currentReceiveCycles >= maxWaitForReceive/periods[0]) { 
    currentReceiveCycles = 0;
    waitingForRX = false;
    forceRX = true;
    TMRequest = 0;
    state.currentSNR = -100;
    state.currentRSSI = -100;
    state.encodedThrottleValue = ENCODED_HALF;
    state.isConnected = false;
    setError("Receive timeout");
    LT.config();
    return false;
  }
  if((!checkTXRXDone() || !RFAvailable) && !forceRX) {
    currentReceiveCycles++;
    return false;
  }
  forceRX = false;
  if(!waitingForRX) {
    waitingForRX = true;
    LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
    return false;
  } else {
    processReceivedPacket();
    currentReceiveCycles = 0;
    waitingForRX = false;
    return true;
  }
}
