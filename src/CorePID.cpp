#include "CorePID.h"
#include <util/atomic.h>



static inline int32_t addLimit32(int32_t a, int32_t b){
    if (b > 0 && a > INT32_MAX - b){
        return INT32_MAX;
    } else if (b < 0 && a < INT32_MIN - b){
        return INT32_MIN;
    } else {
        return a + b;
    }
}


CorePID::CorePID(float freqHz, int16_t min, int16_t max, DerivativeFiltering derivFiltering, float filterCutoffHz):
	frequencyHz(freqHz),
	derivativeFiltering(derivFiltering), 
	configError(PARAMS_UNCONFIGURED),
    filterCutoffHz(filterCutoffHz)
{
	if (min >= max){
		configError = OUTPUT_BOUNDS_INVALID;
        return;
	}

    if (frequencyHz <= 0){
        configError = SAMPLING_FREQ_ERROR;
        return;
    }

	minOutput = int32_t(min) * SCALING_MULT;
	maxOutput = int32_t(max) * SCALING_MULT;
    executionTimer = 0;
    executionIntervalUs = 1000000 / freqHz;
}

void CorePID::setParameters(PIDGains gains){
	setParameters(gains.kp, gains.ki, gains.kd);
}

void CorePID::setParameters(float kp, float ki, float kd){
	
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

	pGain = (uint16_t)(kp * SCALING_MULT);
	iGain = (uint16_t)((ki / frequencyHz) * SCALING_MULT);
	dGain = (uint16_t)((kd * frequencyHz) * SCALING_MULT);

	if (kp != 0 && pGain == 0) {
		configError = KP_OUT_OF_RANGE;
		return;
	}
    
	if (ki != 0 && iGain == 0) {
		configError = KI_OUT_OF_RANGE;
		return;
	}

	if (kd != 0 && dGain == 0) {
		configError = KD_OUT_OF_RANGE;
		return;
	}

	onlyPI = (dGain == 0);

	switch (derivativeFiltering){
		case NO_FILTERING:
			break;
		case LOW_FILTERING:
			filterCutoffHz = 0.15 * frequencyHz;
			break;
		case MEDIUM_FILTERING:
			filterCutoffHz = 0.05 * frequencyHz;
			break;
		case HIGH_FILTERING:
			filterCutoffHz = 0.01 * frequencyHz;
			break;
        case CUSTOM_CUTOFF_HZ:
            break;
	}
    filter.setParams(kd * frequencyHz, filterCutoffHz, frequencyHz);

	if (configError < OUTPUT_BOUNDS_INVALID){
		configError = NO_ERROR;
	}

    reset();
}

void CorePID::reset(int16_t currentMeasurement){
	filter.reset();
	lastMeasurement = currentMeasurement;
	integratorSum = 0;
}	

ConfigError CorePID::getConfigError(){
	return configError;
}

int16_t CorePID::compute(int16_t setpoint, int16_t measurement){

    if (configError != NO_ERROR){
        return 0;
    }
	
    //wrap-safe subtraction to avoid issues with int16_t overflow
	int16_t err =  (int16_t)(uint16_t(setpoint) - uint16_t(measurement));       //i16
    int32_t pTerm, dTerm = 0;
    int32_t output;

	//Calculating Proportional term:
    pTerm = (int32_t)pGain * err;                                               //i16*i16->i32

	//Calculating Derivative term:
	if (!onlyPI){
        int16_t diff = (int16_t)(uint16_t(measurement) - uint16_t(lastMeasurement));
        lastMeasurement = measurement;
		if (derivativeFiltering == NO_FILTERING){
			dTerm = -(int32_t)dGain * diff;                                     //i16*i16->i32
		} else {
			dTerm = -filter.run(diff);                                          //i32
		}
	}
	
    //Computing the output 
    int64_t temp = (int64_t)pTerm + dTerm + integratorSum;
    if (temp > INT32_MAX) temp = INT32_MAX;
    if (temp < INT32_MIN) temp = INT32_MIN;
    output = (int32_t)temp;                              

    //Output clamping and integrator update with back-calculation
	if(output > maxOutput){                                                     //sum, max, min: i24 (i16 << 8)      
        integratorSum -= (output - maxOutput); 
        if (integratorSum < 0) integratorSum = 0;
		output = maxOutput;                     
	} else if (output < minOutput){
        integratorSum += (minOutput - output); 
        if (integratorSum > 0) integratorSum = 0;
		output = minOutput;
	} else {
        integratorSum = addLimit32(integratorSum, (int32_t)iGain * err);
    }

	// Remove the integer scaling factor and apply rounding
	int16_t rval = output >> SCALING_SHIFT;
	rval += (output >> (SCALING_SHIFT - 1)) & 1;

	return rval;
}


bool CorePID::shouldExecuteInLoop(){
    uint32_t now = micros();
    if(now - executionTimer >= executionIntervalUs){
        executionTimer = now;
        return true;
    } else {
        return false;
    }
}
