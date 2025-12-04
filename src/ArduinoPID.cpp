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
    executionTimer = 0;
    executionIntervalUs = 1000000 / freqHz;
}

void ArduinoPID::setParameters(PIDGains gains){
	setParameters(gains.kp, gains.ki, gains.kd);
}

void ArduinoPID::setParameters(float kp, float ki, float kd){
	
	if (kp < 0 || kp > PARAM_MAX){
		configError = KP_OUT_OF_RANGE;
		return;
	}

	if (ki < 0 || (ki  / frequencyHz) > PARAM_MAX){
		configError = KI_OUT_OF_RANGE;
		return;
	}

	if (kd < 0 || (kd * frequencyHz) > PARAM_MAX){
		configError = KD_OUT_OF_RANGE;
		return;
	}

	pGain = (int32_t)(kp * PARAM_MULT);
	iGain = (int32_t)((ki / frequencyHz) * PARAM_MULT);
	dGain = (int32_t)((kd * frequencyHz) * PARAM_MULT);

	switch (derivativeFiltering){
		case NO_FILTERING:
			break;
		case LOW_FILTERING:
			filter.setParams(kd * frequencyHz, 0.80 * frequencyHz, frequencyHz);
			break;
		case MEDIUM_FILTERING:
			filter.setParams(kd * frequencyHz, 0.60 * frequencyHz, frequencyHz);
			break;
		case HIGH_FILTERING:
			filter.setParams(kd * frequencyHz, 0.20 * frequencyHz, frequencyHz);
			break;
	}

	if (configError < OUTPUT_BOUNDS_INVALID){
		configError = NO_ERROR;
	}

    reset();
}

void ArduinoPID::reset(){
	filter.reset();
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
	
    //wrap-safe subtraction to avoid issues with int16_t overflow
	int32_t err =  (int16_t)(uint16_t(setpoint) - uint16_t(measurement));
	int64_t output = 0;

	//Calculating Proportional term:
	output += pGain * err;

	//Calculating Derivative term:
	if (dGain != 0){
        int32_t diff = (int16_t)(uint16_t(measurement) - uint16_t(lastMeasurement));
        lastMeasurement = measurement;
		if (derivativeFiltering == NO_FILTERING){
			output -= dGain * diff;
		} else {
			output -= filter.run(diff);
		}
	}
	
    //Adding integral term:
	output += integratorSum;

    //Output clamping and integrator update with back-calculation
	if(output > maxOutput){
        integratorSum -= (output - maxOutput); 
        if (integratorSum < 0) integratorSum = 0;
		output = maxOutput;
	} else if (output < minOutput){
        integratorSum += (minOutput - output); 
        if (integratorSum > 0) integratorSum = 0;
		output = minOutput;
	} else {
        integratorSum += iGain * err;
    }

	// Remove the integer scaling factor and apply fair rounding
	int16_t rval = output >> PARAM_SHIFT;
	if (output & (0x1ULL << (PARAM_SHIFT - 1))) {
		rval++;
	}

	return rval;
}


bool ArduinoPID::shouldExecuteInLoop(){
    uint32_t now = micros();
    if(now - executionTimer >= executionIntervalUs){
        executionTimer = now;
        return true;
    } else {
        return false;
    }
}
