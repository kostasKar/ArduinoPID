#include "FirstOrderIIRFilter.h"
#include <math.h>


#define OUTPUT_MAX	(INT32_MAX)
#define SCALING_MULT	256
#define SCALING_BITS	8


FirstOrderIIRFilter::FirstOrderIIRFilter(){
    output = 0;
    aCoefficient = 0;
    bCoefficient = 0;
}

int32_t FirstOrderIIRFilter::run(int32_t input){
    
    //y[n] = b0*x[n] + a1*y[n-1]
	output = (int64_t(bCoefficient) * input + int64_t(aCoefficient) * output) >> SCALING_BITS;
    if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    else if (output < -OUTPUT_MAX) output = -OUTPUT_MAX;

	return (int32_t)output;
}

int32_t FirstOrderIIRFilter::getLastOutput(){
    return (int32_t)output;
}

void FirstOrderIIRFilter::reset(){
    output = 0;
}

void FirstOrderIIRFilter::setParams(double gain, double cutoffFreqRadSec, double samplingFreqHz){
    double a1 = expf(-cutoffFreqRadSec / samplingFreqHz);
    
	//Calculate coefficients scaled by PARAM_MULT
	aCoefficient = (int32_t)(a1* SCALING_MULT);
	bCoefficient = (int32_t)(gain * (1.0 - a1) * SCALING_MULT);
	output = 0;
}