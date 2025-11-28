
#include "DerivativeFiltered.h"




#define OUTPUT_MAX	(INT32_MAX)




int32_t DerivativeFiltered::run(int32_t input){

	output = (coef1 * input) - ((coef2 * outputSum) >> 10);

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
  coef2 = (int32_t) (cutoffRadPerSec * 1024.0 / freqHz);
}

