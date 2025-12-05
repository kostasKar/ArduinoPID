#ifndef FIRSTORDERIIRFILTER_H_
#define FIRSTORDERIIRFILTER_H_

#include <inttypes.h>


class FirstOrderIIRFilter{
    
    public:
    FirstOrderIIRFilter();
    //scalingMultiplier is not reverted internally, user must descale
    void setParams(double gain, double cutoffFreqHz, double samplingFreqHz, double scalingMultiplier);
    int32_t run(int32_t input);
    int32_t getLastOutput(); //without recalculating it and thus without affecting the integrator state
    void reset();
    
    private:
    int32_t output;
    int32_t aCoefficient;
    int32_t bCoefficient;
    
};







#endif /* FIRSTORDERIIRFILTER_H_ */