
#include <Arduino.h>
#include <FastGPIO.h>
#include <ArduinoPID.h>
#include <AutoTuner.h>
#include <PIDGains.h>
#include <util/atomic.h>



/*
 *  Kostas Karouzos 2020 
 * 
 *  PID position control of DC motor 
 *  The motor is driven using an H-bridgr as the DRV8871, with two signals (CW, CCW) used in Sign-Magnitude mode.
 *  
 *  As feedback from the motor, a quadrature optical encoder should be used
 *  The two encoder channels, A and B of the encoder are driven to the external interrupts 0 and 1 of the processor
 *  
 *  The inputs of the controller are signals STEPS, DIR, ENABLE 
 *  Each STEPS pulse advances the setpoint by 1 or -1, according to DIR. ENABLE switches on or off the output driver
 *  
 *  The PID loop frequency, as well as the derivative filter cutoff frequency can be configured.
 *  The gains of the PID controller can be either autotuned or manually set.
 *  The autotuner performs the auto-tuning relay method with the configured parameters. 
 *  The auto tuned gains are held in EEPROM. If there are no or invalid gains in EEPROM, the controller does not run.
 *  Autotuning can be performed by pressing the correspoding button.
 *  
 *  During operation, the onboard LED is on when the controller is within configurable limits from the setpoint.
 *  Additionally to the STEPS/DIR interface, the setpoint can be set by sending an integer (position setpoint in steps units) through Serial
 *  
 * EXTERNAL LIB DEPENDENCIES: 
 *  - pololu/FastGPIO 
 * 
 */



/* --------------------------------------------------------------------
 *                       CONSTS - CONFIGURATION
 *---------------------------------------------------------------------
 */
//Motor driving configuration
#define PWM_FREQ        20000         //(Hz)
#define TIMER_TOP       ((F_CPU / PWM_FREQ)-1)  //The highest value of the PWM timer
#define DIR_INPUT_INV   false         //invert the logic of the DIR input signal`

//PID configuration
#define PID_FREQ_HZ     1000          //Sampling frequency of the PID loop

//Autotuner configuration:
#define AUTOTUNER_SETPOINT    50      //(steps)
#define AUTOTUNER_OUTPUT_STEP ((int32_t)(0.75 * TIMER_TOP))
#define AUTOTUNER_HYSTERESIS  10      //(steps)

//Spot-on indication threshold
#define SPOT_ON_THRESHOLD 5       //(steps)

//IO pins:
const int CH_A =          2;          //interrupt 0
const int CH_B =          3;          //interrupt 1
const int STEPS =         4;          //interrupt pc20
const int DIR =           5;
const int ENABLE =        6;
const int AUTOTUNE =      7;
const int MOTOR_OUT_A =   9;          //OC1A
const int MOTOR_OUT_B =   10;         //OC1B
const int ONBOARD_LED =	  13;


/* --------------------------------------------------------------------
 *                           GLOBALS
 *---------------------------------------------------------------------
 */

volatile int16_t positionMeasurement = 0;
volatile int16_t positionSetpoint = 0; 

PIDGains pidGains;
AutoTuner autoTuner(AUTOTUNER_SETPOINT, AUTOTUNER_OUTPUT_STEP, AUTOTUNER_HYSTERESIS);
ArduinoPID pid(PID_FREQ_HZ, -TIMER_TOP, TIMER_TOP, MEDIUM_FILTERING);


/* --------------------------------------------------------------------
 *                        EXTERNAL INTERRUPTS
 *---------------------------------------------------------------------
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
  if (FastGPIO::Pin<STEPS>::isInputHigh()){
	 if(FastGPIO::Pin<DIR>::isInputHigh() ^ DIR_INPUT_INV){
        positionSetpoint++;
     } else{
        positionSetpoint--;
     }  
  }
}



/* --------------------------------------------------------------------
 *                          INITIALIZERS
 *---------------------------------------------------------------------
 */
void PwmSetup(){
  //setting PWM generator (timer1) and starting it to count:
  ICR1 = TIMER_TOP;

  TCCR1A =
    (1 << COM1A1) | //non inverted mode on OC1A
    (0 << COM1A0) |
    (1 << COM1B1) | //non inverted mode on OC1B
    (0 << COM1B0) |
    (1 << WGM11)  | //part of Fast PWM mode 
    (0 << WGM10);

  TCCR1B =
    (0 << ICNC1) |
    (0 << ICES1) |
    (1 << WGM13) | //part of Fast PWM mode
    (1 << WGM12) |
    (0 << CS12)  | //no prescaler
    (0 << CS11)  |
    (1 << CS10);

  OCR1A = 0;
  OCR1B = 0;	

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
  pinMode(STEPS, INPUT_PULLUP);
  pinMode(DIR, INPUT_PULLUP);
  sei();
}



/* --------------------------------------------------------------------
 *                           FUNCTIONS
 *---------------------------------------------------------------------
 */
//abs(outputValue) must not be greater than TIMER_TOP
void setMotorSpeed (int16_t outputValue){

  if (!FastGPIO::Pin<ENABLE>::isInputHigh()){
    OCR1A = 0;
    OCR1B = 0;
    return;
  }

  if (outputValue >= 0){
    OCR1A = outputValue;
    OCR1B = 0;
  } else {
    OCR1A = 0;
    OCR1B = -outputValue;
  }
}

void readSerialSetpoint(){
  if (Serial.available()){
	positionSetpoint = Serial.parseInt();
    Serial.print("new Setpoint:");
    Serial.println(positionSetpoint);
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

  


/* --------------------------------------------------------------------
 *                           SETUP
 *---------------------------------------------------------------------
 */

void setup() {
  Serial.begin(115200);
  ConfigureExternalInperrupts();
  PwmSetup();
  
  pinMode(ENABLE, INPUT_PULLUP);
  pinMode(AUTOTUNE, INPUT_PULLUP);
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
}


/* --------------------------------------------------------------------
 *                           MAIN LOOP
 *---------------------------------------------------------------------
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
  
  if (!FastGPIO::Pin<AUTOTUNE>::isInputHigh()){performAutoTune();}
  if (abs(measurement - setpoint) <= SPOT_ON_THRESHOLD) {FastGPIO::Pin<ONBOARD_LED>::setOutputLow();} else {FastGPIO::Pin<ONBOARD_LED>::setOutputHigh();}
  readSerialSetpoint();
}
