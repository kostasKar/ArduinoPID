#ifndef PIDGAINS_H_
#define PIDGAINS_H_


class PIDGains{

    public:
    float kp, ki, kd;
    bool saveToEEPROM(int slot = 0);
    bool readFromEEPROM(int slot = 0);

    private:
    static int calcrc(char *ptr, int count);
    
};


#endif /* PIDGAINS_H_ */    