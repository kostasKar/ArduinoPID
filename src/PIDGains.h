#ifndef PIDGAINS_H_
#define PIDGAINS_H_


class PIDGains{

    public:
    float kp, ki, kd;
    void saveToEEPROM(int startAddress = 0);
    bool readFromEEPROM(int startAddress = 0);
    
};


#endif /* PIDGAINS_H_ */    