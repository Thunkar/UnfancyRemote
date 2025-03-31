#include "config.h"

Config config { 15, 224, 12, 0 };

void writeUInt(int address, unsigned int number){ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

unsigned int readUInt(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void readConfig() {
  config.channel = readUInt(0);
  config.identity = readUInt(2);
  config.cellN = readUInt(4);
  config.frequency = config.channel * CH_BANDWIDTH_HZ + BASE_FREQUENCY;
}

void writeConfig() {
  writeUInt(0, config.channel);
  writeUInt(2, config.identity);
  writeUInt(4, config.cellN);
  EEPROM.commit();
}

void setupEEPROM() {
  EEPROM.begin(6);
}