
#ifndef AUTOTUNER_H_
#define AUTOTUNER_H_

#include <inttypes.h>
#include "PIDGains.h"

#define NUMBER_OF_PEAKS 20

enum GainTuningMethod{
    ZIEGLER_NICHOLS,  //Aggressive - fast
    TYREUS_LUYBEN,     //Robust, conservative
    PI_ONLY,           //No derivative term
    CUSTOM_BANDWIDTH_FACTOR,    //custom ratio of ultimate frequency to desired closed loop frequency
}; 


enum AutotunerStates {
	BELOW_SETPOINT,
	ABOVE_SETPOINT
};


struct Peak{
	int32_t peakValue;
	uint32_t timePointUs;
	uint32_t nextZeroCrossTimePointUs;
};


/*
 *  Performs the 'Relay' PID auto tuning method method. More info: 
 *  https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=2ahUKEwjFvNCOptToAhX6RhUIHRSnDEgQFjAAegQIAxAB&url=https%3A%2F%2Ffenix.tecnico.ulisboa.pt%2FdownloadFile%2F395139427476%2FResumo%2520Alargado%2520-%2520Auto-tuning%2520de%2520Controladores%2520PID%2520pelo%2520m%25C3%25A9todo%2520Relay.pdf&usg=AOvVaw3EmIs2o9sXAqL2Htx69UzN
 *  NOTE: Output bias trimming is not implemented but info about it is printed out through Serial. In my implementations no correction was needed
 */
class AutoTuner {

	public:

	AutoTuner(int32_t setpoint, int32_t outputStep, int32_t hysteresis);
	void init();
	int16_t run(int16_t measurement);
	bool isFinished();
    //customBandwidthFactor: ultimate frequency to desired closed loop frequency ratio
    //Values from 1.5 (highly aggresive) to 10.0 (very conservative)
	PIDGains getPIDGains(GainTuningMethod method = ZIEGLER_NICHOLS, double customBandwidthFactor = 3.2);

	private:
	
	int32_t setpoint;
	int32_t outputStep;
	int32_t hysteresis;
	bool completed;

	Peak peakCandidate;
	bool justStarted;
	AutotunerStates state;

	Peak maxs[NUMBER_OF_PEAKS];
	Peak mins[NUMBER_OF_PEAKS];
	int maxIndex;
	int minIndex;
	
    double Ku, Pu; //ultimate gain and period

	void analyzeMeasurements();

};




#endif
