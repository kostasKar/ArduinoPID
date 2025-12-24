#include "PIDGains.h"
#include <EEPROM.h>

bool PIDGains::saveToEEPROM(int slot){
    float gains[3] = {kp, ki, kd};
    int startAddress = slot * (sizeof(gains) + sizeof(int));
    EEPROM.put(startAddress, gains);
    int crc = calcrc((char *)gains, sizeof(gains));
    EEPROM.put(startAddress + sizeof(gains), crc);
    
    PIDGains test;
    return((test.readFromEEPROM(slot)) &&
           (kp == test.kp) &&
           (ki == test.ki) &&
           (kd == test.kd));
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

int PIDGains::calcrc(char *ptr, int count) {
	int  crc;
	char i;
	crc = 0;
	while (--count >= 0)
	{
		crc = crc ^ (int) *ptr++ << 8;
		i = 8;
		do
		{
			if (crc & 0x8000)
			crc = crc << 1 ^ 0x1021;
			else
			crc = crc << 1;
		} while(--i);
	}
	return (crc);
}