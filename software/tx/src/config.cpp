#include "config.h"

Config config { 15, 224, 12, 0, 0, 0, 0, false, true };

void writeUInt(int address, unsigned int number){ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

unsigned int readUInt(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void readConfig() {
  config.channel = readUInt(10);
  config.identity = readUInt(12);
  config.cellN = readUInt(14);
  config.isDual = readUInt(16);
}

void readCalibration() {
  config.centerAcc = readUInt(0);
  config.calAcc = readUInt(2);
  config.calBrake = readUInt(4);
  config.inverted = readUInt(6);
  config.centerBrake = readUInt(8);
}

void writeCalibration() {
  writeUInt(0, config.centerAcc);
  writeUInt(2, config.calAcc);
  writeUInt(4, config.calBrake);
  writeUInt(6, config.inverted);
  writeUInt(8, config.centerBrake);
  EEPROM.commit();
}

void writeConfig() {
  writeUInt(10, config.channel);
  writeUInt(12, config.identity);
  writeUInt(14, config.cellN);
  writeUInt(16, config.isDual);
  EEPROM.commit();
}