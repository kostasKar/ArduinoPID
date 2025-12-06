#include "FirstOrderIIRFilter.h"
#include <math.h>

#define SCALING_SHIFT  8


FirstOrderIIRFilter::FirstOrderIIRFilter(){
    output = 0;
    aCoefficient = 0;
    bCoefficient = 0;
}

int32_t FirstOrderIIRFilter::run(int32_t input){
    
    //y[n] = b0*x[n] + a1*y[n-1]
	output = (bCoefficient * input + aCoefficient * (output >> SCALING_SHIFT));
	return output;
}

int32_t FirstOrderIIRFilter::getLastOutput(){
    return (int32_t)output;
}

void FirstOrderIIRFilter::reset(){
    output = 0;
}

void FirstOrderIIRFilter::setParams(double gain, double cutoffFreqHz, double samplingFreqHz){
    double a1 = expf(- 2 * M_PI * cutoffFreqHz / samplingFreqHz);
    
	//Calculate coefficients scaled by PARAM_MULT
	aCoefficient = (int32_t)(a1* (1 << SCALING_SHIFT));
	bCoefficient = (int32_t)(gain * (1.0 - a1) * (1 << SCALING_SHIFT));
	output = 0;
}