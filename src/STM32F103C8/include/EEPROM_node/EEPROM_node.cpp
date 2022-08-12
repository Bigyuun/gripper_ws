#include <EEPROM_node.hpp>

using namespace EEPROMmanagerNode;


EEPROMmanager::EEPROMmanager()
{
  
}

EEPROMmanager::~EEPROMmanager()
{

}

long EEPROMmanager::EEPROMReadlong(long address) {

  long r_first_ = EEPROM.read(address + 3);
  long r_second_ = EEPROM.read(address + 2);
  long r_third_ = EEPROM.read(address + 1);
  long r_fourth_ = EEPROM.read(address);
  
  return ((r_fourth_ << 0) & 0xFF) + ((r_third_ << 8) & 0xFFFF) + ((r_second_ << 16) & 0xFFFFFF) + ((r_first_ << 24) & 0xFFFFFFFF);
}

void EEPROMmanager::EEPROMWritelong(int address, long value) {

  byte w_fourth_ = (value & 0xFF);
  byte w_third_ = ((value >> 8) & 0xFF);
  byte w_second_ = ((value >> 16) & 0xFF);
  byte w_first_ = ((value >> 24) & 0xFF);
  
  EEPROM.write(address    , w_fourth_);
  EEPROM.write(address + 1, w_third_);
  EEPROM.write(address + 2, w_second_);
  EEPROM.write(address + 3, w_first_);
}