
#include "DerivativeFiltered.h"




#define OUTPUT_MAX	(INT32_MAX)
#define SCALING_MULT	256
#define SCALING_BITS	8




int32_t DerivativeFiltered::run(int32_t input){

	output = (int64_t(coef1) * int64_t(input)) - (int64_t(coef2) * int64_t(outputSum) >> SCALING_BITS);

	if (output > OUTPUT_MAX){
		output = OUTPUT_MAX;
	} else if (output < -OUTPUT_MAX){
		output = -OUTPUT_MAX;
	}

	outputSum += output;
	return int32_t(output);
}

int32_t DerivativeFiltered::getLastOutput(){
	return int32_t(output);
}

void DerivativeFiltered::reset(){
	outputSum = 0;
}


void DerivativeFiltered::setParams(double cutoffRadPerSec, double Gain, double freqHz){
  outputSum = 0;
  
  coef1 = (int32_t) (Gain * cutoffRadPerSec);
  coef2 = (int32_t) (cutoffRadPerSec * SCALING_MULT / freqHz);
}

