/*
 * ArduinoPID.cpp
 *
 * Created: 09-Oct-19 8:35:00 PM
 *  Author: kostas
 */ 

#include "ArduinoPID.h"
#include <util/atomic.h>

ArduinoPID::ArduinoPID(voidWriteInt32PtrType  outPtr, double derivativeFilterCutoff, uint16_t frequencyHz, int32_t min, int32_t max, AutoTune * tuner):
	minOutput(min),
	outputRange(max - min),
	outputPtr(outPtr),
	derivativeFilterN(derivativeFilterCutoff),
	autoTuner(tuner),
	stopped(true)
	
{
	dtMs = 1000.0 / frequencyHz;
	dtMicros = (uint32_t) (dtMs * 1000);
	reset();		
}

void ArduinoPID::autoSetGains(){
	if (! readAutoTunerGainsFromEEPROM()){
		autoTune();
	}
}

bool ArduinoPID::computeInLoop(){
	uint32_t now = micros();
	if (now - previousExecutionTime >= dtMicros){
		previousExecutionTime = now;
		return compute();		
	}
	return false;
}

bool ArduinoPID::compute(){

	if (stopped){
		return false;
	}
	
	//ATOMIC access and local copy of measurement and setpoint:
	cli();
	int32_t tempMeasurement = measurement;
	int32_t tempSetpoint = setpoint;
	sei();
	error = (tempSetpoint - tempMeasurement);
	
	//Calculating Proportional term:
	int32_t output = pCoeff * error;

	//Calculating Derivative term:
	#if DERIVATIVE_ON_MEASUREMENT 
		#if FILTERED_DERIVATIVE
			output -=  derFiltered.output(tempMeasurement);
		#else
			output -=  dCoeff * (tempMeasurement - lastMeasurement);
			lastMeasurement = tempMeasurement;
		#endif
	#else
		#if FILTERED_DERIVATIVE
			output += derFiltered.output(error);
		#else
			output += dCoeff * (error - lastError);
			lastError = error;
		#endif
	#endif

	//Calculating Integral term:
	if (!antiWindupNeeded()){
		errorSum += error;
	}
	output += (iCoeff * errorSum) >> 10; //instead of doing /1000
	
	//Writing output:
	output = applyLimit(output);	
	writeOutput(scaleToRange(output));

	return true;
}


void ArduinoPID::reset(){
	setpoint = 0;
	measurement = 0;
	lastError = 0;
	lastMeasurement = 0;
	errorSum = 0;
	writeOutput(scaleToRange(0));
	derFiltered.reset();
}

void ArduinoPID::stop(){
	stopped = true;
}

void ArduinoPID::start(){
	stopped = false;
}

bool ArduinoPID::isStopped(){
	return stopped;
}

void ArduinoPID::setGains (double kp, double ki, double kd){
	pCoeff = (int16_t) kp;
	iCoeff = (int16_t) (ki * dtMs); //the / 1000 happens inside compute() so that we don't lose accuracy
	dCoeff = (int16_t) (kd * 1000 / dtMs);
	derFiltered.setParams(derivativeFilterN, kd, dtMs);
}


void ArduinoPID::autoTune(){
	
	if (!autoTuner){return;}

	bool wasOn = !isStopped();
	stop();
	
	autoTuner->bindInputOutput(&autoTuneInput, &autoTuneOutput);

	int completed = 0;
	autoTuner->init();
	
	while (!completed){
		ATOMIC_BLOCK(ATOMIC_FORCEON) {autoTuneInput = measurement;}
		completed = autoTuner->execute();
		writeOutput(scaleToRange(autoTuneOutput));
	}
	autoTuner->analyzeMeasurements();
	
	setGains(autoTuner->getKp(), autoTuner->getKi(), autoTuner->getKd());
	
	writeAutoTunerGainsToEEPROM();

	if (wasOn){
		start();
	}
	
}


void ArduinoPID::setSetpoint(int32_t s){
	setpoint = s;
}

void ArduinoPID::setMeasurement(int32_t m) {
	measurement = m;
}

int32_t ArduinoPID::getSetpoint(){
	cli();
	int32_t temp = setpoint;
	sei();
	return temp;
}

int32_t ArduinoPID::getMeasurement(){
	cli();
	int32_t temp = measurement;
	sei();
	return temp;
}

void ArduinoPID::incrementSetpoint (int8_t s){
	setpoint += s;
}

void ArduinoPID::incrementMeasurement (int8_t m){
	measurement += m;
}

bool ArduinoPID::isSpotOn(){
	return (error == 0);
}

bool ArduinoPID::isSteady(){
	return ((error == 0) && (derFiltered.getLastOutput() == 0));
}


bool ArduinoPID::antiWindupNeeded(){
	return ((internalOutputMAXed && (error > 0)) || (internalOutputMINed && (error < 0)));
}

int32_t ArduinoPID::applyLimit(int32_t value){
	
	if (value > INTERNAL_MAX_OUTPUT){
		internalOutputMAXed = true;
		internalOutputMINed = false;
		return INTERNAL_MAX_OUTPUT;
	} else if (value < -INTERNAL_MAX_OUTPUT){
		internalOutputMAXed = false;
		internalOutputMINed = true;
		return -INTERNAL_MAX_OUTPUT;
	} else {
		internalOutputMAXed = false;
		internalOutputMINed = false;
		return value;
	}
}


int32_t ArduinoPID::scaleToRange (int32_t value){
	return ((outputRange * (value + INTERNAL_MAX_OUTPUT)) >> (INTERNAL_MAX_OUTPUT_BITS + 1)) + minOutput;
}

void ArduinoPID::writeOutput(int32_t calculatedOutput) {outputPtr(calculatedOutput);}

void ArduinoPID::writeAutoTunerGainsToEEPROM(){
	double gains[3];
	gains[0] = autoTuner->getKp();
	gains[1] = autoTuner->getKi();
	gains[2] = autoTuner->getKd();
	EEPROM.put(4, gains);
	int crc = calcrc((char *)gains, sizeof(gains));
	EEPROM.put(0, crc);
}

bool ArduinoPID::readAutoTunerGainsFromEEPROM(){
	
	double gains[3];
	int savedCrc, calculatedCrc;
	
	EEPROM.get(4, gains);
	EEPROM.get(0, savedCrc);
	calculatedCrc = calcrc((char *)gains, sizeof(gains));
	
	if (calculatedCrc != savedCrc) {
		Serial.println("Invalid gains in EEPROM.");
		return false;
		} else {
		setGains(gains[0], gains[1], gains[2]);
		Serial.println("Read gains from EEPROM:");
		Serial.println(gains[0]);
		Serial.println(gains[1]);
		Serial.println(gains[2]);
		return true;
	}
}