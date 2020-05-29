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
#include <avr/io.h>
#include <util/atomic.h>
#include "AutoTune.h"
#include "DerivativeFiltered.h"
#include <EEPROM.h>
#include "crc.h"

#define INTERNAL_MAX_OUTPUT 16384 //this has to be a power of two for easier calculations
#define INTERNAL_MAX_OUTPUT_BITS 14 // log2(INTERNAL_MAX_OUTPUT)

#define DERIVATIVE_ON_MEASUREMENT true
#define FILTERED_DERIVATIVE true


typedef void (*voidWriteInt32PtrType)(int32_t); //function pointer that is called with the PID output after its calculation




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


class ArduinoPID{
	
	public:
	
	ArduinoPID(voidWriteInt32PtrType  outPtr, double derivativeFilterCutoff, uint16_t frequencyHz, int32_t min, int32_t max, AutoTune * tuner = NULL);
	void autoSetGains();
	bool computeInLoop();
	bool compute();
	void reset();
	void stop();
	void start();
	bool isStopped();
	void setGains (double kp, double ki, double kd);
	void autoTune();
	void setSetpoint(int32_t s);
	void setMeasurement(int32_t m);
	int32_t getSetpoint();
	int32_t getMeasurement();
	void incrementSetpoint (int8_t s);
	void incrementMeasurement (int8_t m);
	bool isSpotOn();
	bool isSteady();
	
	
	private:
	
	volatile int32_t setpoint;
	volatile int32_t measurement;
	int32_t error;
	int32_t lastError;			//used if FILTRED_DERIVATIVE == false
	int32_t lastMeasurement;	//used if FILTRED_DERIVATIVE == false
	
	DerivativeFiltered derFiltered;
	uint32_t previousExecutionTime;
	int16_t pCoeff, iCoeff, dCoeff;		//the gains
	double derivativeFilterN;
	int32_t errorSum;
	double  dtMs;	//sampling period in ms
	uint32_t dtMicros;
	int32_t outputRange, minOutput;
	bool internalOutputMAXed, internalOutputMINed;
	voidWriteInt32PtrType outputPtr;
	bool stopped;
	
	int32_t autoTuneInput, autoTuneOutput;
	AutoTune * autoTuner;
	
	bool antiWindupNeeded();
	int32_t applyLimit(int32_t value);
	int32_t scaleToRange (int32_t value);
	void writeOutput(int32_t calculatedOutput);
	void writeAutoTunerGainsToEEPROM();
	bool readAutoTunerGainsFromEEPROM();
	};







#endif /* INTPID_H_ */
