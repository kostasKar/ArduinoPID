#include <Arduino.h>
#include "AutoTuner.h"
#include <math.h> 


AutoTuner::AutoTuner(int32_t setpoint, int32_t outputStep, int32_t hysteresis):
	setpoint(setpoint),
	outputStep(outputStep),
	hysteresis(hysteresis)
{

}


void AutoTuner::init(){
	peakCandidate.peakValue = setpoint;
	peakCandidate.timePointUs = 0;
	state = BELOW_SETPOINT;
	maxIndex = 0;
	minIndex = 0;
	justStarted = true;
	completed = false;
}

int16_t AutoTuner::run(int16_t measurement){
	int16_t output;
	uint32_t now = micros();

	if (completed){
		return 0;
	}
		
	switch (state){
		case BELOW_SETPOINT:
		output = outputStep;
		if (measurement < peakCandidate.peakValue){
			peakCandidate.peakValue = measurement;
			peakCandidate.timePointUs = now;
		}
		if (measurement > setpoint + hysteresis / 2) {
			if (!justStarted){
				mins[minIndex].peakValue = peakCandidate.peakValue;
				mins[minIndex].timePointUs = peakCandidate.timePointUs;
				mins[minIndex].nextZeroCrossTimePointUs = now;
				minIndex++;
			}
			justStarted = false;
			output = -outputStep;
			state = ABOVE_SETPOINT;
		}
		break;
		
		case ABOVE_SETPOINT:
		output = -outputStep;
		if (measurement > peakCandidate.peakValue){
			peakCandidate.peakValue = measurement;
			peakCandidate.timePointUs = now;
		}
		if (measurement < setpoint - hysteresis / 2){
			maxs[maxIndex].peakValue = peakCandidate.peakValue;
			maxs[maxIndex].timePointUs = peakCandidate.timePointUs;
			maxs[maxIndex].nextZeroCrossTimePointUs = now;
			maxIndex++;
			output = outputStep;
			state = BELOW_SETPOINT;
		}
		break;
	}

	if (minIndex == NUMBER_OF_PEAKS){
		completed = true;
		output = 0;
		analyzeMeasurements();
	}

	return output;
}


void AutoTuner::analyzeMeasurements(){
	
	double averagePeriod = 0;
	double averageAmplitude = 0;
	
	Serial.println(F("Autotuning data:"));
	Serial.print(F("Input setpoint: ")); Serial.println(setpoint);
	Serial.print(F("Output step (d): ")); Serial.println(outputStep);
	Serial.print(F("hysteresis: ")); Serial.println(hysteresis);
	

	for (int i = NUMBER_OF_PEAKS - 1; i > 0; i--){
		uint32_t maxPeriod = maxs[i].timePointUs - maxs[i-1].timePointUs;
		uint32_t minPeriod = mins[i].timePointUs - mins[i-1].timePointUs;
		averagePeriod += maxPeriod + minPeriod;
		averageAmplitude += maxs[i].peakValue - mins[i].peakValue;
		uint32_t periodOverSetpoint = maxs[i].nextZeroCrossTimePointUs - mins[i-1].nextZeroCrossTimePointUs;  //these are just to check if we need outpout bias trimming
		uint32_t periodUnderSetpoint = mins[i].nextZeroCrossTimePointUs - maxs[i].nextZeroCrossTimePointUs;
		
		Serial.print(F("Maxes Period: ")); Serial.print(maxPeriod);
		Serial.print(F(" Mins Period: ")); Serial.print(minPeriod);
		Serial.print(F(" Duration over setpoint: ")); Serial.print(periodOverSetpoint);
		Serial.print(F(" Duration under setpoint: ")); Serial.println(periodUnderSetpoint);
		Serial.print(F(" Max: ")); Serial.print(maxs[i].peakValue);
		Serial.print(F(" Min: ")); Serial.println(mins[i].peakValue);
	}
	
	averagePeriod /= (NUMBER_OF_PEAKS - 1) * 2;
	averageAmplitude /= (NUMBER_OF_PEAKS - 1);
	
	Serial.print(F("Average Period (us): ")); Serial.println(averagePeriod);
	Serial.print(F("Average Amplitude: ")); Serial.println(averageAmplitude);
	
	Pu = averagePeriod / 1000000; //in sec
	Ku = (4.0 * outputStep) / (M_PI * sqrt(square(averageAmplitude) - square(hysteresis)));
		
	Serial.print(F("Ultimate Gain Ku: ")); Serial.println(Ku);
	Serial.print(F("Ultimate period Pu (sec): ")); Serial.println(Pu);  

    Serial.println(F("Example PID gains for different methods:"));
    Serial.println(F("Ziegler-Nichols:"));
    getPIDGains(ZIEGLER_NICHOLS).printout();
    Serial.println(F("Tyreus-Luyben:"));
    getPIDGains(TYREUS_LUYBEN).printout();
    Serial.println(F("PI only:"));
    getPIDGains(PI_ONLY).printout();
}

bool AutoTuner::isFinished(){
	return completed;
}	

PIDGains AutoTuner::getPIDGains(GainTuningMethod method, double customBandwidthFactor){
    double Kp, Ti, Td;
    double a = customBandwidthFactor;

    switch(method){
        case ZIEGLER_NICHOLS:
            Kp = 0.6 * Ku;
            Ti = 0.5 * Pu;
            Td = 0.125 * Pu;
            break;
        case TYREUS_LUYBEN:
            Kp = 0.31 * Ku;
            Ti = 2.2 * Pu;
            Td = 0.168 * Pu;
            break;
        case PI_ONLY:
            Kp = 0.45 * Ku;
            Ti = 0.83 * Pu;
            Td = 0.0;
            break;
        case CUSTOM_BANDWIDTH_FACTOR:
            if (a < 1.5) a = 1.5;
            if (a > 10.0) a = 10.0;
            Kp = Ku / a;
            Ti = a * Pu;
            Td = Pu / (4 * a);
            break;
    }

	PIDGains gains;
	gains.kp = Kp;
	gains.ki = Kp / Ti;
	gains.kd = Kp * Td;
	return gains;
}