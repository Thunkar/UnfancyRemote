#include "RF.h"

SX128XLT LT;

unsigned long lastPacketTime = 0;
unsigned long lastRFWait = 0;

int consecutiveForceRXSetupModePackets = 0;

RF_STATE rfState = RF_STATE::READY_FOR_RX;

struct ReceptionResult { 
  bool success;
  bool TMRequest; 
};

void transitionState(RF_STATE newState) {
  if(rfState != newState) {
    rfState = newState;
    if (rfState == RF_STATE::RX_WAITING || rfState == RF_STATE::TX_WAITING) {
      lastRFWait = micros();
    }
  }
}

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

bool checkRXIRQError() {
  uint16_t IRQStatus = LT.readIrqStatus();
  return !(IRQStatus & (IRQ_CRC_ERROR + IRQ_RX_TX_TIMEOUT + IRQ_SYNCWORD_ERROR));
}

ReceptionResult processThrottlePacket() {
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
  TMRequest = (receivedData & TM_REQUEST_MASK) >> 12;
  bool forceSetupMode = (receivedData & FORCE_SETUP_MODE_MASK) >> 13;
  if(forceSetupMode) {
    consecutiveForceRXSetupModePackets++;
    if(consecutiveForceRXSetupModePackets > FORCE_RX_SETUP_THRESHOLD) {
      config.forceSetupMode = true;
      writeConfig();
      ESP.restart();
    }
  } else {
    consecutiveForceRXSetupModePackets = 0;
  }

  return { true, TMRequest };
}

TaskResult sendTMPacket(unsigned long now) {
  if(rfState != RF_STATE::READY_FOR_TX) {
    return { false, 0 };
  }
  LT.startWriteSXBuffer(0);             
  unsigned int boardVoltageAsInt = roundAndCastToInt(state.boardVoltage);
  unsigned int encodedBoardVoltage = map(boardVoltageAsInt, 0, 420 * config.cellN, 0, 255) << 8;        
  LT.writeUint16(encodedBoardVoltage+config.identity);                            
  LT.endWriteSXBuffer();   
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, TM_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.transmitSXBufferIRQ(0, TM_PACKET_LENGTH, 0, TX_POWER, NO_WAIT);  
  stats.TMPackets++;
  transitionState(RF_STATE::TX_WAITING);
  return { true, 0 };
}

TaskResult receiveThrottlePacket(unsigned long now) {
  if(rfState != RF_STATE::READY_FOR_RX) {
    return { false, 0 };
  }
  checkRXTimeout();
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, THROTTLE_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  transitionState(RF_STATE::RX_WAITING);
  return { true, 0 };
}

TaskResult handleThrottlePacket(unsigned long now) {
  if(rfState != RF_STATE::RX_DONE) {
    return { false, 0 };
  }
  ReceptionResult result = processThrottlePacket();
  if (result.success && result.TMRequest) {
    transitionState(RF_STATE::READY_FOR_TX);
  } else {
    transitionState(RF_STATE::READY_FOR_RX);
  }
  return { result.success, 0 };
}

TaskResult checkRFStatus(unsigned long now) {
  if(rfState != RF_STATE::RX_WAITING && rfState != RF_STATE::TX_WAITING) {
    return { false, 0 };
  }
  bool isRx = rfState == RF_STATE::RX_WAITING;
  unsigned long timeout = isRx ? RX_TIMEOUT_US : TX_TIMEOUT_US;
  if((now - lastRFWait) > timeout) {
    ERROR_CODE errorCode = isRx ? ERROR_CODE::RX_TIMEOUT : ERROR_CODE::RX_TIMEOUT;
    setError(errorCode);
    LT.setMode(MODE_STDBY_RC);
    transitionState(RF_STATE::READY_FOR_RX);
    return { false, 0 };
  }
  uint16_t mask = isRx ? RX_IRQ_MASK : TX_IRQ_MASK;
  bool RFAvailable = checkRFBusy() && checkRFDone(mask);
  if(RFAvailable) {
    LT.setMode(MODE_STDBY_RC);
    if(isRx) {
      transitionState(RF_STATE::RX_DONE);
    } else {
      transitionState(RF_STATE::READY_FOR_RX);
    }
    return { true, 0 };
  } else {
    return { false, 0 };
  }
}

void setupLoRa() {
  SPI.begin();

  if (!LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
  {
    #ifdef DEBUG
    Serial.println(F("Device error"));
    #endif
  }

  LT.setupLoRa(config.frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);
  LT.setPeriodBase(PERIODBASE_15_US);
}