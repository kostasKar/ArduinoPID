#include <Arduino.h>
#include <FastGPIO.h>
#include <ArduinoPID.h>
#include <AutoTuner.h>
#include <PIDGains.h>
#include <util/atomic.h>
#include "PulseScaler.h"
#include <Wire.h>




/*  
 *  Kostas Karouzos 2019
 *  
 *  ------------------------------------------------------------------
 *  CONTROLLER Operation:
 *  ------------------------------------------------------------------
 *  PID position control of a DC motor.
 *  The motor is driven using an H-bridge such as the DRV8871, with two
 *  signals (CW, CCW) used in Sign-Magnitude mode.
 *  
 *  As feedback from the motor, a quadrature optical encoder is used.
 *  The two encoder channels, A and B, are connected to external
 *  interrupts 0 and 1 of the processor.
 *
 *  The PID loop frequency, as well as the derivative filter cutoff
 *  frequency, can be configured.
 *  The gains of the PID controller are autotuned.
 *  The autotuner performs the relay auto-tuning method using the
 *  configured parameters.
 *
 *  The auto-tuned gains are stored in EEPROM. If no gains are present
 *  or the stored values are invalid, autotuning can be initiated by 
 *  pressing the designated button.
 *  
 *  During operation, the onboard LED is lit when the controller is in
 *  steady state.
 *  
 *  ------------------------------------------------------------------
 *  CLOCK Operation:
 *  ------------------------------------------------------------------
 *  The setpoint of the controller is the position of the hour hand of
 *  a clock. Therefore, it completes one full revolution every 12 hours.
 *  
 *  To achieve this, the resolution of the encoder must be used:
 *  STEPS_PER_ROTATION.
 *  The PulseScaler class generates STEPS_PER_ROTATION steps over
 *  12 hours (or 43,200 seconds), even if these numbers are mutually
 *  prime.
 *  
 *  The clock input is either the 1 Hz pulse train produced by an DS3231 
 *  RTC module (with USE_RTC = true), or the internal millis() function is
 *  used. The millis() function is not temperature-calibrated, so it may
 *  exhibit significant drift.
 *  
 *  The clock also has two external buttons:
 *  - SET: allows adjustment of the hand position while held down,
 *    without the controller counteracting the movement.
 *  - EXT_AUTOTUNE: an additional button to trigger autotuning, useful
 *    if such a button is desired on the clock chassis.
 *  
 */


/* --------------------------------------------------------------------
                         CONSTS - CONFIGURATION
  ---------------------------------------------------------------------
*/
//Motor driving configuration
#define PWM_FREQ    20000         //(Hz)
#define TIMER_TOP   ((F_CPU / PWM_FREQ)-1)  //The highest value of the PWM timer


//PID configuration
#define PID_FREQ_HZ   1000     //Sampling frequency of the PID loop (Hz)

//Autotuner configuration:
#define AUTOTUNER_SETPOINT    50        //(steps)
#define AUTOTUNER_OUTPUT_STEP ((int32_t)(0.75 * TIMER_TOP))     
#define AUTOTUNER_HYSTERESIS  10        //(steps)

//clock specific
#define STEPS_PER_ROTATION    1336
#define SECS_PER_ROTATION     43200 //12*3600 
#define USE_RTC               true

//Spot-on indication threshold
#define SPOT_ON_THRESHOLD 5       //(steps)

//IO pins:
const int CH_A =            2;   //interrupt 0
const int CH_B =            3;   //interrupt 1
const int AUTOTUNE_BUTTON = 7;
const int MOTOR_OUT_A =     9;   //OC1A
const int MOTOR_OUT_B =     10;  //OC1B
const int ONBOARD_LED =     13;
const int CLOCK_1_HZ_INPUT= 4;   //PCB STEPS input
const int EXT_AUTOTUNE =    5;   //PCB DIR input
const int SET_TIME_BUTTON = 6;   //PCB ENABLE input
const int RTC_I2C_SDA =     A4;  //handled automatically by Wire library
const int RTC_I2C_SCL =     A5;  //handled automatically by Wire library 

/* --------------------------------------------------------------------
                             GLOBALS
  ---------------------------------------------------------------------
*/

volatile int16_t positionMeasurement = 0;
volatile int16_t positionSetpoint = 0; 

PIDGains pidGains;
AutoTuner autoTuner(AUTOTUNER_SETPOINT, AUTOTUNER_OUTPUT_STEP, AUTOTUNER_HYSTERESIS);
ArduinoPID pid(PID_FREQ_HZ, -TIMER_TOP, TIMER_TOP, MEDIUM_FILTERING);
PulseScaler scaler(SECS_PER_ROTATION, STEPS_PER_ROTATION);

#if USE_RTC
#include <RtcDS3231.h>
RtcDS3231<TwoWire> Rtc(Wire);
#endif

uint32_t previousMillis;

/* --------------------------------------------------------------------
                          EXTERNAL INTERRUPTS
  ---------------------------------------------------------------------
*/
ISR(INT0_vect) {
  if(FastGPIO::Pin<CH_A>::isInputHigh() != FastGPIO::Pin<CH_B>::isInputHigh()){
    positionMeasurement++;
  } else {
    positionMeasurement--;
  }
}

ISR(INT1_vect) {
  if(FastGPIO::Pin<CH_A>::isInputHigh() != FastGPIO::Pin<CH_B>::isInputHigh()){
    positionMeasurement--;
  } else {
    positionMeasurement++;
  }
}

ISR(PCINT2_vect) {
  if (FastGPIO::Pin<CLOCK_1_HZ_INPUT>::isInputHigh()){
    if (scaler.step()){
      positionSetpoint--;
    }
  }
}

/* --------------------------------------------------------------------
                            INITIALIZERS
  ---------------------------------------------------------------------
*/
void PwmSetup() {
  //setting PWM generator (timer1) and starting it to count:
  ICR1 = TIMER_TOP;
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011001;
  /*Fast-PWM mode.
    Top counting value: register ICR1
    Compare value for channel A: register OCR1A
    Channel A: non-inverted (or inverted if INVERT_DIR == true)
    Channel B: inverted (or non-inverted if INVERT_DIR == true)
    No prescaler
  */
  pinMode(MOTOR_OUT_A, OUTPUT);
  pinMode(MOTOR_OUT_B, OUTPUT);
}

void ConfigureExternalInperrupts() {
  //configure the appropriate interrupts
  EICRA |= (1 << ISC00);  // sense any change on the INT0 pin
  EIMSK |= (1 << INT0);
  EICRA |= (1 << ISC10);  // sense any change on the INT1 pin
  EIMSK |= (1 << INT1);
  // Enable pin-change interrupt on PD4 - PCINT20
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
  PCIFR |= (1 << PCIF2); // Clear its interrupt flag by writing a 1.
  pinMode(CH_A, INPUT_PULLUP);
  pinMode(CH_B, INPUT_PULLUP);
  pinMode(CLOCK_1_HZ_INPUT, INPUT_PULLUP);
  sei();
}


/* --------------------------------------------------------------------
                             FUNCTIONS
  ---------------------------------------------------------------------
*/
//abs(outputValue) must not be greater than TIMER_TOP
void setMotorSpeed (int32_t outputValue) {
  if (outputValue >= 0) {
    OCR1A = outputValue;
    OCR1B = 0;
  } else {
    OCR1A = 0;
    OCR1B = -outputValue;
  }
}

void resetPosition(){
    ATOMIC_BLOCK(ATOMIC_FORCEON){
        positionMeasurement = 0;
        positionSetpoint = 0;
    }
}

void checkPIDError(){
    if (pid.getError() != NO_ERROR) {
        Serial.print(F("PID configuration error: ")); Serial.println(pid.getError());
    } else {
        Serial.println(F("PID configured successfully"));
    }
} 

void performAutoTune(){
    Serial.println(F("Starting Autotune..."));
    setMotorSpeed(0);
    delay(1000);
    int32_t measurement;

    resetPosition();
    autoTuner.init();
    while (!autoTuner.isFinished()){
        ATOMIC_BLOCK(ATOMIC_FORCEON){ measurement = positionMeasurement; }
        setMotorSpeed(autoTuner.run(measurement));
    }   

    pidGains = autoTuner.getPIDGains();
    pidGains.saveToEEPROM();
    pid.setParameters(pidGains);
    pid.reset();
    checkPIDError();
}

bool setTimeButtonIsPressed() {
  static bool setTimeButtonPressed = false;
  static uint32_t setTimePreviousMillis = 0;

  if (!FastGPIO::Pin<SET_TIME_BUTTON>::isInputHigh()){
    if ((setTimeButtonPressed) && (millis() - setTimePreviousMillis > 50)){
      return true;
    } else if (!setTimeButtonPressed){
      setTimeButtonPressed = true;
      setTimePreviousMillis = millis();
      return false;
    } else {
      return false;
    }
  } else {
     setTimeButtonPressed = false;
     return false;
  }
  
}

bool autotuneButtonIsPressed() {
  static bool autoTuneButtonPressed = false;
  static uint32_t autoTunePreviousMillis = 0;

  if ((!FastGPIO::Pin<AUTOTUNE_BUTTON>::isInputHigh()) || (!FastGPIO::Pin<EXT_AUTOTUNE>::isInputHigh())){
    if ((autoTuneButtonPressed) && (millis() - autoTunePreviousMillis > 50)){
      return true;
    } else if (!autoTuneButtonPressed) {
      autoTuneButtonPressed = true;
      autoTunePreviousMillis = millis();
      return false;
    } else {
      return false;
    }
  } else {
     autoTuneButtonPressed = false;
     return false;
  }
  
}


/* --------------------------------------------------------------------
                             SETUP
  ---------------------------------------------------------------------
*/

void setup() {
  Serial.begin(115200);
  ConfigureExternalInperrupts();
  PwmSetup();
  pinMode(AUTOTUNE_BUTTON, INPUT_PULLUP);
  pinMode(EXT_AUTOTUNE, INPUT_PULLUP);
  pinMode(SET_TIME_BUTTON, INPUT_PULLUP);
  pinMode(ONBOARD_LED, OUTPUT);

  if (pidGains.readFromEEPROM()) {
      Serial.println(F("Loaded PID gains from EEPROM:"));
      Serial.print(F("Kp: ")); Serial.println(pidGains.kp);
      Serial.print(F("Ki: ")); Serial.println(pidGains.ki);
      Serial.print(F("Kd: ")); Serial.println(pidGains.kd);
      pid.setParameters(pidGains);
  } else {
      Serial.println(F("No valid PID gains in EEPROM"));
  }

  checkPIDError();

#if USE_RTC
  Rtc.Begin();
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeClock);
  Rtc.SetSquareWavePinClockFrequency(DS3231SquareWaveClock_1Hz);
#endif
  
}


/* --------------------------------------------------------------------
                             MAIN LOOP
  ---------------------------------------------------------------------
*/
void loop() {

  int16_t setpoint, measurement;

  if (pid.shouldExecuteInLoop()){
    ATOMIC_BLOCK(ATOMIC_FORCEON){
      setpoint = positionSetpoint;
      measurement = positionMeasurement;
    }
    setMotorSpeed(pid.compute(setpoint, measurement));
  }

#if USE_RTC == false
 if (millis() - previousMillis >= 1000) {
   previousMillis = millis();
   if (scaler.step()){
      positionSetpoint--;
   }
 }
#endif

  if (setTimeButtonIsPressed()) { resetPosition(); pid.reset();}
  if (autotuneButtonIsPressed()) { performAutoTune();}
  if (abs(measurement - setpoint) <= SPOT_ON_THRESHOLD) {FastGPIO::Pin<ONBOARD_LED>::setOutputLow();} else {FastGPIO::Pin<ONBOARD_LED>::setOutputHigh();}
}
