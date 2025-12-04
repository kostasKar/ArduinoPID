#ifndef PIDGAINS_H_
#define PIDGAINS_H_


class PIDGains{

    public:
    float kp, ki, kd;
    void saveToEEPROM(int slot = 0);
    bool readFromEEPROM(int slot = 0);
    
};


#endif /* PIDGAINS_H_ */    