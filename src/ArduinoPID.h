/*
 * IntPID.h
 *
 * Created: 09-Oct-19 8:34:48 PM
 *  Author: kostas
 */ 


#ifndef INTPID_H_
#define INTPID_H_

#include <inttypes.h>
#include "Arduino.h"
#include "DerivativeFiltered.h"
#include "FirstOrderIIRFilter.h"
#include "PIDGains.h"


/*
* https://www.mathworks.com/videos/understanding-pid-control-part-3-expanding-beyond-a-simple-derivative-1531120808026.html
* https://www.mathworks.com/videos/understanding-pid-control-part-2-expanding-beyond-a-simple-integral-1528310418260.html
*
* implements clamping integral anti-windup, derivative on measurement and filtered derivative with cutoff frequency N rad/sec (the version with an integrator in feedback loop instead of a forward derivative, see video)
* cutoff freq in rad/sec is cutoff freq in Hz * 2 * pi
*
* A note about atomicity:
	getSetPoint() and getMeasurement() use atomic access.
	compute() also uses atomic access 
	setSetpoint(), setMEasurement(), incrementSetpoint(), incrementMeasurement() are not atomic.
	That is only a problem if one of the above is executed outside of an ISR and:
	- another one of the above affecting the same variable (setpoint or measurement) is also executed inside an ISR, or:
	- the compute() method is executed inside an ISR (eg. of a timer for timed execution)
	In the above cases, the above methods should be enclosed by cli(); ... sei();
*
*/

#define PARAM_SHIFT  8
#define PARAM_BITS   16
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> PARAM_SHIFT) 
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)) >> (PARAM_BITS - PARAM_SHIFT)) 


enum DerivativeFiltering{
	NO_FILTERING,
	LOW_FILTERING,
	MEDIUM_FILTERING,
	HIGH_FILTERING
};

enum ConfigError{
	NO_ERROR,
	PARAMS_UNCONFIGURED,
	KP_OUT_OF_RANGE,
	KI_OUT_OF_RANGE,
	KD_OUT_OF_RANGE,
	OUTPUT_BOUNDS_INVALID
};


class ArduinoPID{
	
	public:

	ArduinoPID(float freqHz, int16_t min, int16_t max, DerivativeFiltering derivFiltering = LOW_FILTERING);
	void setParameters(float kp, float ki, float kd);
	void setParameters(PIDGains gains);
	int16_t compute(int16_t setpoint, int16_t measurement);
	void reset();
	ConfigError getError();
	
	private:

	float frequencyHz;
	DerivativeFiltering derivativeFiltering;
	ConfigError configError;
	int64_t minOutput, maxOutput;
	bool outputMaxed, outputMined;
	FirstOrderIIRFilter filter;
	int32_t pGain, iGain, dGain, awGain;
	int16_t lastMeasurement;
	int64_t integratorSum;
	};







#endif /* INTPID_H_ */
