#include "RF.h"

SX128XLT LT;

unsigned long lastTMPacketAttempt = 0;
unsigned long lastTMPacketReceived = 0;

unsigned int resetTMCounter = 0;

void checkTMTimeout() {
  if(state.isConnected && (micros() - lastTMPacketReceived) > TM_TIMEOUT_US) {
    state.boardVoltage = 0.0;
    state.boardCellVoltage = 0.0;
    state.isConnected = false;
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
  resetTMCounter = 0;
  lastTMPacketReceived = micros();
}

void receiveTMPacket() {
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, TM_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.receiveSXBufferIRQ(0, 0, NO_WAIT);
  if(!waitForRFReady(RX_TIMEOUT_US, RX_WAIT)) {
    setError(ERROR_CODE::RX_TIMEOUT);
    return;
  }
  processTMPacket();
}

TaskResult sendThrottlePacket(unsigned long now) {
  checkTMTimeout();
  LT.startWriteSXBuffer(0);                     
  LT.writeUint8(config.identity); 
  bool requestTM = (now - lastTMPacketAttempt) > TM_PERIOD_US;
  unsigned int encodedData = (requestTM << 12) + state.encodedThrottleValue;                   
  LT.writeUint16(encodedData);          
  LT.endWriteSXBuffer();     
  LT.setPacketParams(PREAMBLE_LENGTH, LORA_PACKET_FIXED_LENGTH, THROTTLE_PACKET_LENGTH, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.transmitSXBufferIRQ(0, THROTTLE_PACKET_LENGTH, 0, TX_POWER, NO_WAIT);  
  if(requestTM) {
    lastTMPacketAttempt = now;
    if(!waitForRFReady(TX_TIMEOUT_US, TX_WAIT)) {
      setError(ERROR_CODE::TX_TIMEOUT);
      return { false, 0 };
    }
    receiveTMPacket();
  }
  stats.packets++;
  return { true, 0 };                  
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