#include "PIDGains.h"
#include <EEPROM.h>
#include "crc.h"

void PIDGains::saveToEEPROM(int slot){
    float gains[3] = {kp, ki, kd};
    int startAddress = slot * (sizeof(gains) + sizeof(int));
    EEPROM.put(startAddress, gains);
    int crc = calcrc((char *)gains, sizeof(gains));
    EEPROM.put(startAddress + sizeof(gains), crc);
}

bool PIDGains::readFromEEPROM(int slot){
    float gains[3];
    int savedCrc, calculatedCrc;
    int startAddress = slot * (sizeof(gains) + sizeof(int));
    EEPROM.get(startAddress, gains);
    EEPROM.get(startAddress + sizeof(gains), savedCrc);
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