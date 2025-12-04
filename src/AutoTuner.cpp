/*
 * AutoTune.cpp
 *
 * Created: 14-Dec-18 8:25:36 PM
 *  Author: kostas
 */ 

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
	
	double Pu = averagePeriod / 1000000; //in sec
	double Ku = (4.0 * outputStep) / (M_PI * sqrt(square(averageAmplitude) - square(hysteresis)));
	
	Kp = 0.6 * Ku;
	Ki = Kp / (0.5 * Pu);
	Kd = Kp * (0.125 * Pu);
	
	Serial.print(F("Kp: ")); Serial.println(Kp);
	Serial.print(F("Ki: ")); Serial.println(Ki);  
	Serial.print(F("Kd: ")); Serial.println(Kd);
	
}

bool AutoTuner::isFinished(){
	return completed;
}	

PIDGains AutoTuner::getPIDGains(){
	PIDGains gains;
	gains.kp = Kp;
	gains.ki = Ki;
	gains.kd = Kd;
	return gains;
}