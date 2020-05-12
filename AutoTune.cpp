/*
 * AutoTune.cpp
 *
 * Created: 14-Dec-18 8:25:36 PM
 *  Author: kostas
 */ 

#include <Arduino.h>
#include "AutoTune.h"
#include <math.h> 




void AutoTune::bindInputOutput(int32_t * in, int32_t * out){
	inputPtr = in;
	outputPtr = out;
}

AutoTune::AutoTune(int32_t setpoint, int32_t outputStep, int32_t hysteresis){
	_setpoint = setpoint;
	_outputStep = outputStep;
	_hysteresis = hysteresis;
}


void AutoTune::init(){
	peakCandidate.peakValue = _setpoint;
	peakCandidate.timePointUs = 0;
	state = BELOW_SETPOINT;
	maxIndex = 0;
	minIndex = 0;
	justStarted = true;
	*outputPtr = _outputStep;
}

int AutoTune::execute(){

	uint32_t now = micros();
	
	int32_t input = *inputPtr;
	
	switch (state){
		case BELOW_SETPOINT:
		if (input < peakCandidate.peakValue){
			peakCandidate.peakValue = input;
			peakCandidate.timePointUs = now;
		}
		if (input > _setpoint + _hysteresis / 2) {
			if (!justStarted){
				mins[minIndex].peakValue = peakCandidate.peakValue;
				mins[minIndex].timePointUs = peakCandidate.timePointUs;
				mins[minIndex].nextZeroCrossTimePointUs = now;
				minIndex++;
			}
			justStarted = false;
			*outputPtr = -_outputStep;
			state = ABOVE_SETPOINT;
		}
		break;
		
		case ABOVE_SETPOINT:
		if (input > peakCandidate.peakValue){
			peakCandidate.peakValue = input;
			peakCandidate.timePointUs = now;
		}
		if (input < _setpoint - _hysteresis / 2){
			maxs[maxIndex].peakValue = peakCandidate.peakValue;
			maxs[maxIndex].timePointUs = peakCandidate.timePointUs;
			maxs[maxIndex].nextZeroCrossTimePointUs = now;
			maxIndex++;
			*outputPtr = _outputStep;
			state = BELOW_SETPOINT;
		}
		break;
	}

	if (minIndex == NUMBER_OF_PEAKS){
		*outputPtr = 0;
		return 1;
	} else {
		return 0;
	}
}


void AutoTune::analyzeMeasurements(){
	
	double averagePeriod = 0;
	double averageAmplitude = 0;
	
	Serial.println("Autotuning data:");
	Serial.print("Input setpoint: ");
	Serial.println(_setpoint);
	Serial.print("Output step (d): ");
	Serial.print(_outputStep);
	Serial.println(" (max output: 16384)");
	Serial.print("hysteresis: ");
	Serial.println(_hysteresis);

	for (int i = NUMBER_OF_PEAKS - 1; i > 0; i--){
		uint32_t maxPeriod = maxs[i].timePointUs - maxs[i-1].timePointUs;
		uint32_t minPeriod = mins[i].timePointUs - mins[i-1].timePointUs;
		averagePeriod += maxPeriod + minPeriod;
		averageAmplitude += maxs[i].peakValue - mins[i].peakValue;
		uint32_t periodOverSetpoint = maxs[i].nextZeroCrossTimePointUs - mins[i-1].nextZeroCrossTimePointUs;  //these are just to check if we need outpout bias trimming
		uint32_t periodUnderSetpoint = mins[i].nextZeroCrossTimePointUs - maxs[i].nextZeroCrossTimePointUs;
		
		Serial.print("Maxes Period: ");
		Serial.print(maxPeriod);
		Serial.print(" Mins Period: ");
		Serial.print(minPeriod);
		Serial.print(" Duration over setpoint: ");
		Serial.print(periodOverSetpoint);
		Serial.print(" Duration under setpoint: ");
		Serial.println(periodUnderSetpoint);
		Serial.print(" Max: ");
		Serial.print(maxs[i].peakValue);
		Serial.print(" Min: ");
		Serial.println(mins[i].peakValue);
	}
	
	averagePeriod /= (NUMBER_OF_PEAKS - 1) * 2;
	averageAmplitude /= (NUMBER_OF_PEAKS - 1);
	
	Serial.print("Average Period (us): ");
	Serial.println(averagePeriod);
	Serial.print("Average Amplitude: ");
	Serial.println(averageAmplitude);
	
	double Pu = averagePeriod / 1000000; //in sec
	//double Ku = (4.0 * (_outputStep)) / (3.1415 * averageAmplitude);
	double Ku = (4.0 * (_outputStep)) / (M_PI * sqrt(square(averageAmplitude) - square(_hysteresis)));
	
	Kp = 0.6 * Ku;
	Ki = 1.2 * Ku / (Pu);
	Kd = 0.075 * Ku * Pu;
	
	Serial.print("Kp: ");
	Serial.println(Kp);
	Serial.print("Ki: ");
	Serial.println(Ki);  
	Serial.print("Kd: ");
	Serial.println(Kd);
	
}

double AutoTune::getKp(){
	return Kp;
}

double AutoTune::getKi(){
	return Ki;
}

double AutoTune::getKd(){
	return Kd;
}