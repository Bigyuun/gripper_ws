#include <Arduino.h>
#include <EEPROM.h>


namespace EEPROMmanagerNode
{

class EEPROMmanager
{
private:
    /* data */
    long r_first_;
    long r_second_;
    long r_third_;
    long r_fourth_;

    byte w_first_;
    byte w_second_;
    byte w_third_;
    byte w_fourth_;

private:
    EEPROMmanager();
    ~EEPROMmanager();

public:
    long EEPROMReadlong(int address);
    void EEPROMWritelong(int address, long value);
};

} // namespace EEPROMmanager
