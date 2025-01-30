#include "config.h"

Config config { 0, 0, 0, false, 0 };

void writeUInt(int address, unsigned int number){ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

unsigned int readUInt(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void readConfig() {
  config.centerAcc = readUInt(0);
  config.calAcc = readUInt(2);
  config.calBrake = readUInt(4);
  config.inverted = readUInt(6);
  config.centerBrake = readUInt(8);
}

void writeConfig() {
  writeUInt(0, config.centerAcc);
  writeUInt(2, config.calAcc);
  writeUInt(4, config.calBrake);
  writeUInt(6, config.inverted);
  writeUInt(8, config.centerBrake);
  EEPROM.commit();
}