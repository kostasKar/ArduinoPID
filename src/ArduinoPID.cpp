/*
 * ArduinoPID.cpp
 *
 * Created: 09-Oct-19 8:35:00 PM
 *  Author: kostas
 */ 

#include "ArduinoPID.h"
#include <util/atomic.h>



ArduinoPID::ArduinoPID(float freqHz, int16_t min, int16_t max, DerivativeFiltering derivFiltering):
	frequencyHz(freqHz),
	derivativeFiltering(derivFiltering), 
	configError(PARAMS_UNCONFIGURED)
{
	if (min >= max){
		configError = OUTPUT_BOUNDS_INVALID;
	}
	minOutput = int64_t(min) * PARAM_MULT;
	maxOutput = int64_t(max) * PARAM_MULT;

}

void ArduinoPID::setParameters(PIDGains gains){
	setParameters(gains.kp, gains.ki, gains.kd);
}

void ArduinoPID::setParameters(float kp, float ki, float kd){
	
	if (kp < 0 || kp > PARAM_MAX){
		configError = KP_OUT_OF_RANGE;
		return;
	}

	if (ki < 0 || ki > PARAM_MAX){
		configError = KI_OUT_OF_RANGE;
		return;
	}

	if (kd < 0 || kd > PARAM_MAX){
		configError = KD_OUT_OF_RANGE;
		return;
	}

	pGain = (uint32_t)(kp * PARAM_MULT);
	iGain = (uint32_t)((ki / frequencyHz) * PARAM_MULT);
	dGain = (uint32_t)((kd * frequencyHz) * PARAM_MULT);

	switch (derivativeFiltering){
		case NO_FILTERING:
			break;
		case LOW_FILTERING:
			derFilter.setParams(0.80 * frequencyHz, kd, frequencyHz);
			break;
		case MEDIUM_FILTERING:
			derFilter.setParams(0.63 * frequencyHz, kd, frequencyHz);
			break;
		case HIGH_FILTERING:
			derFilter.setParams(0.19 * frequencyHz, kd, frequencyHz);
			break;
	}

	if (configError < OUTPUT_BOUNDS_INVALID){
		configError = NO_ERROR;
	}

}

void ArduinoPID::reset(){
	derFilter.reset();
	lastMeasurement = 0;
	integratorSum = 0;
}	

ConfigError ArduinoPID::getError(){
	return configError;
}

int16_t ArduinoPID::compute(int16_t setpoint, int16_t measurement){

    if (configError != NO_ERROR){
        return 0;
    }
	
	int32_t err = int32_t(setpoint) - int32_t(measurement);
	int64_t output;

	//Calculating Proportional term:
	output += int64_t(pGain) * err;

	//Calculating Integral with anti-windup clamping 
    if (!((outputMaxed && err > 0) || (outputMined && err < 0))){
        integratorSum += int64_t(iGain) * int64_t(err);
    }
	output += integratorSum;

	//Calculating Derivative term:
	if (dGain != 0){
		if (derivativeFiltering == NO_FILTERING){
			int32_t deriv = int32_t(measurement) - int32_t(lastMeasurement);
			lastMeasurement = measurement;
			if (deriv > DERIV_MAX){
				deriv = DERIV_MAX;
			} else if (deriv < DERIV_MIN){
				deriv = DERIV_MIN;
			}
			output -= int32_t(dGain) * deriv;
		} else {
			output -= derFilter.run(measurement);
		}
	}
	
	if(output > maxOutput){
		output = maxOutput;
        outputMaxed = true;
	} else if (output < minOutput){
		output = minOutput;
        outputMined = true;
	} else {
        outputMaxed = false;
        outputMined = false;
    }

	// Remove the integer scaling factor and apply fair rounding
	int16_t rval = output >> PARAM_SHIFT;
	if (output & (0x1ULL << (PARAM_SHIFT - 1))) {
		rval++;
	}

	return rval;
}