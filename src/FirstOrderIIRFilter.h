#ifndef FIRSTORDERIIRFILTER_H_
#define FIRSTORDERIIRFILTER_H_

#include <inttypes.h>


class FirstOrderIIRFilter{
    
    public:
    FirstOrderIIRFilter();
    void setParams(double gain, double cutoffFreqHz, double samplingFreqHz);
    //output is shifted by SCALING_SHIFT bits defined in .cpp file
    int32_t run(int32_t input);
    int32_t getLastOutput(); //without recalculating it and thus without affecting the internal state
    void reset();
    
    private:
    int32_t output;
    int32_t aCoefficient;
    int32_t bCoefficient;
    
};







#endif /* FIRSTORDERIIRFILTER_H_ */