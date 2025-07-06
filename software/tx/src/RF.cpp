#include "RF.h"

SX128XLT LT;

unsigned long lastTMPacketAttempt = 0;
unsigned long lastTMPacketReceived = 0;
unsigned long lastRFWait = 0;

int forceRxSetupPackets = 0;

bool requestTM = false;

RF_STATE rfState = RF_STATE::READY_FOR_TX;

void transitionState(RF_STATE newState) {
  if(rfState != newState) {
    rfState = newState;
    if (rfState == RF_STATE::RX_WAITING || rfState == RF_STATE::TX_WAITING) {
      lastRFWait = micros();
    }
  }
}

void checkTMTimeout() {
  if(state.isConnected && (micros() - lastTMPacketReceived) > TM_TIMEOUT_US) {
    state.boardVoltage = 0.0;
    state.boardCellVoltage = 0.0;
    state.isConnected = false;
    setError(ERROR_CODE::DISCONNECTED);
  }
}

void checkForceRXSetupReset() {
  if (state.forceRxSetup && forceRxSetupPackets < MAX_FORCE_RX_SETUP_PACKETS) {
    forceRxSetupPackets++;
  } else {
    state.forceRxSetup = false;
    forceRxSetupPackets = 0;
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

void processTMPacket() {    
  unsigned int RXIdentity = -1;
  unsigned int receivedData = 0;

  if(!checkRXIRQError()) {
    setError(ERROR_CODE::IRQ_ERROR);
    return;
  }
  LT.startReadSXBuffer(0);                
  receivedData = LT.readUint16();
  RXIdentity = receivedData & RX_IDENTITY_MASK;
  LT.endReadSXBuffer(); 
  
  if(config.identity != RXIdentity) {
    setError(ERROR_CODE::INCORRECT_IDENTITY);
    return;
  }
  
  stats.TMPackets++;
  unsigned int decodedBatteryVoltage = (receivedData & BATTERY_VOLTAGE_MASK) >> 8;
  state.boardVoltage = map(decodedBatteryVoltage, 0, 255, 0, config.cellN * 420)/100.0;
  state.boardCellVoltage = state.boardVoltage/float(config.cellN);
  state.isConnected = true;
  lastTMPacketReceived = micros();
}

TaskResult receiveTMPacket(unsigned long now) {
  if(rfState != RF_STATE::TX_DONE || !requestTM) {
    return { false, 0 };
  }
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, TM_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  lastTMPacketAttempt = now;
  requestTM = false;
  transitionState(RF_STATE::RX_WAITING);
  return { true, 0 };
}

TaskResult handleTMPacket(unsigned long now) {
  if(rfState != RF_STATE::RX_DONE) {
    return { false, 0 };
  }
  processTMPacket();
  transitionState(RF_STATE::READY_FOR_TX);
  return { true, 0 };
}

TaskResult sendThrottlePacket(unsigned long now) {
  if(rfState != RF_STATE::READY_FOR_TX) {
    return { false, 0 };
  }
  checkTMTimeout();
  checkForceRXSetupReset();
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(config.identity); 
  requestTM = (now - lastTMPacketAttempt) > TM_PERIOD_US;
  unsigned int encodedData = (state.forceRxSetup << 13) + (requestTM << 12) + state.encodedThrottleValue;  
  LT.writeUint16(encodedData);          
  LT.endWriteSXBuffer();     
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, THROTTLE_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.transmitSXBufferIRQ(0, THROTTLE_PACKET_LENGTH, 0, TX_POWER, NO_WAIT);  
  stats.packets++;
  transitionState(RF_STATE::TX_WAITING);
  return { true, 0 };                  
}

TaskResult checkRFStatus(unsigned long now) {
  if(rfState != RF_STATE::RX_WAITING && rfState != RF_STATE::TX_WAITING) {
    return { false, 0 };
  }
  bool isRx = rfState == RF_STATE::RX_WAITING;
  unsigned long timeout = isRx ? RX_TIMEOUT_US : TX_TIMEOUT_US;
  if((now - lastRFWait) > timeout) {
    requestTM = false;
    ERROR_CODE errorCode = isRx ? ERROR_CODE::RX_TIMEOUT : ERROR_CODE::RX_TIMEOUT;
    setError(errorCode);
    LT.setMode(MODE_STDBY_RC);
    transitionState(RF_STATE::READY_FOR_TX);
    return { false, 0 };
  }
  uint16_t mask = isRx ? RX_IRQ_MASK : TX_IRQ_MASK;
  bool RFAvailable = checkRFBusy() && checkRFDone(mask);
  if(RFAvailable) {
    LT.setMode(MODE_STDBY_RC);
    if(isRx) {
      transitionState(RF_STATE::RX_DONE);
    } else {
      transitionState(requestTM ? RF_STATE::TX_DONE : RF_STATE::READY_FOR_TX);
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