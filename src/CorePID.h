#ifndef COREPID_H_
#define COREPID_H_

#include <inttypes.h>
#include "Arduino.h"
#include "FirstOrderIIRFilter.h"
#include "PIDGains.h"


#define SCALING_SHIFT  8
#define SCALING_MULT   (0x1ULL << SCALING_SHIFT) 
#define PARAM_BITS   16
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> SCALING_SHIFT) 


enum DerivativeFiltering{
	NO_FILTERING,
	LOW_FILTERING,
	MEDIUM_FILTERING,
	HIGH_FILTERING,
    CUSTOM_CUTOFF_HZ
};

enum ConfigError{
	NO_ERROR,
	PARAMS_UNCONFIGURED,
	KP_OUT_OF_RANGE,
	KI_OUT_OF_RANGE,
	KD_OUT_OF_RANGE,
	OUTPUT_BOUNDS_INVALID,
	SAMPLING_FREQ_ERROR
};


class CorePID{
	
	public:

	CorePID(float freqHz, int16_t min, int16_t max, DerivativeFiltering derivFiltering = LOW_FILTERING, float filterCutoffHz = 1.0);
	void setParameters(float kp, float ki, float kd);
	void setParameters(PIDGains gains);
	int16_t compute(int16_t setpoint, int16_t measurement);
	void reset(int16_t currentMeasurement = 0);
	ConfigError getConfigError();
    bool shouldExecuteInLoop();
	
	private:

	float frequencyHz;
	DerivativeFiltering derivativeFiltering;
	ConfigError configError;
	int32_t minOutput, maxOutput;
	FirstOrderIIRFilter filter;
    float filterCutoffHz;
	uint16_t pGain, iGain, dGain;
	int16_t lastMeasurement;
	int32_t integratorSum;
	bool onlyPI;

    uint32_t executionTimer;
    uint32_t executionIntervalUs;
	};







#endif /* INTPID_H_ */
