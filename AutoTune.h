
#ifndef AUTOTUNE_H_
#define AUTOTUNE_H_


#define NUMBER_OF_PEAKS 20



typedef enum {
	BELOW_SETPOINT,
	ABOVE_SETPOINT
} States;


typedef struct {
	int32_t peakValue;
	uint32_t timePointUs;
	uint32_t nextZeroCrossTimePointUs;
} Peak;


/*
 *  Performs the 'Relay' PID auto tuning method method. More info: 
 *  https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=1&ved=2ahUKEwjFvNCOptToAhX6RhUIHRSnDEgQFjAAegQIAxAB&url=https%3A%2F%2Ffenix.tecnico.ulisboa.pt%2FdownloadFile%2F395139427476%2FResumo%2520Alargado%2520-%2520Auto-tuning%2520de%2520Controladores%2520PID%2520pelo%2520m%25C3%25A9todo%2520Relay.pdf&usg=AOvVaw3EmIs2o9sXAqL2Htx69UzN
 *  NOTE: Output bias trimming is not implemented but info about it is printed out through Serial. In my implementations no correction was needed
 */
class AutoTune {

	public:

	AutoTune(int32_t setpoint, int32_t outputStep, int32_t hysteresis);
	void init();
	void bindInputOutput(int32_t * in, int32_t * out);
	int execute();
	void analyzeMeasurements();
	
	
	double getKp();
	double getKi();
	double getKd();

	private:

	int32_t * inputPtr;
	int32_t * outputPtr;
	
	int32_t _setpoint;
	int32_t _outputStep;
	int32_t _hysteresis;

	Peak peakCandidate;
	bool justStarted;
	States state;

	Peak maxs[NUMBER_OF_PEAKS];
	Peak mins[NUMBER_OF_PEAKS];
	int maxIndex;
	int minIndex;
	
	double Kp, Ki, Kd;

};




#endif
