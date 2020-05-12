
#include "DerivativeFiltered.h"


int32_t DerivativeFiltered::output(int32_t input){

	_out = (coef1 * input) - ((coef2 * outputSum) >> 10);  //either outputSum has to be 64 bits, or the logic shift to happen first 
	outputSum += _out;
	return _out;
}

int32_t DerivativeFiltered::getLastOutput(){
	return _out;
}

void DerivativeFiltered::reset(){
	outputSum = 0;
}


void DerivativeFiltered::setParams(double _N, double _K, double _dtMs){
  outputSum = 0;
  
  coef1 = (int32_t) (_K * _N);
  coef2 = (int32_t) (_N * _dtMs);
}

