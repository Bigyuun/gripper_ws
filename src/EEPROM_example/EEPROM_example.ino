// include headers
  /***
    Iterate through each byte of the EEPROM storage.
    Larger AVR processors have larger EEPROM sizes, E.g:
    - Arduino Duemilanove: 512 B EEPROM storage.
    - Arduino Uno:         1 kB EEPROM storage.
    - Arduino Mega:        4 kB EEPROM storage.
    Rather than hard-coding the length, you should use the pre-provided length function.
    This will make your code portable to all AVR processors.
  ***/
#include <EEPROM.h>

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  // EEPROM clear 0
  for(int i=0; i<EEPROM.length(); i++)
  {
    EEPROM.write(i,0);  
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // long rN = random(1363, 781539);

  // EEPROMWritelong(0, rN);
  // long val = EEPROMReadlong(0);

  // Serial.println(val);
}


long EEPROMReadlong(long address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void EEPROMWritelong(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}
