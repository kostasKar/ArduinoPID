#include "PIDGains.h"
#include <EEPROM.h>
#include "crc.h"

void PIDGains::saveToEEPROM(int startAddress){
    float gains[3] = {kp, ki, kd};
    EEPROM.put(startAddress + 4, gains);
    int crc = calcrc((char *)gains, sizeof(gains));
    EEPROM.put(startAddress, crc);
}
bool PIDGains::readFromEEPROM(int startAddress){
    float gains[3];
    int savedCrc, calculatedCrc;

    EEPROM.get(startAddress + 4, gains);
    EEPROM.get(startAddress, savedCrc);
    calculatedCrc = calcrc((char *)gains, sizeof(gains));

    if (calculatedCrc != savedCrc) {
        return false;
    } else {
        kp = gains[0];
        ki = gains[1];
        kd = gains[2];
        return true;
    }
}