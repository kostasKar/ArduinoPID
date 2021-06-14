
#include <FastGPIO.h>
#include <ArduinoPID.h>
#include <AutoTune.h>
#include <EEPROM.h>



/*
 *  Kostas Karouzos 2020 
 * 
 *  PID position control of DC motor 
 *  The motor is driven using an H-bridgr as the DRV8871, with two signals (CW, CCW) used in Sign-Magnitude or Anti-phase lock mode.
 *  Driving mode can be configured by the ANTIPHASE_LOCK parameter. The frequency of the PWM signals is also configured
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
 *  The auto tuned gains are held in EEPROM. If there are no or invalid gains in EEPROM, autotuning is fired during startup
 *  Otherwise, Autotuning can be performed by pressing the correspoding button.
 *  
 *  During operation, the onboard LED is on when the controller is in steady state
 *  
 *  Additionally to the STEPS/DIR interface, the setpoint can be set by sending an integer (position setpoint in steps units) through Serial
 *  
 */



/* --------------------------------------------------------------------
 *                       CONSTS - CONFIGURATION
 *---------------------------------------------------------------------
 */
//Motor driving configuration
#define PWM_FREQ		    20000			//(Hz)
#define TIMER_TOP	((F_CPU / PWM_FREQ)-1)	//The highest value of the PWM timer
#define ANTIPHASE_LOCK	false			//antiphase-lock driving mode of the motor. Alternative is sign-magnitude
#define DIR_INPUT_INV   false     //invert the logic of the DIR input signal`

//PID configuration
#define D_FILTER_N		  300.0					//the derivative filter's cutoff (rad/sec)
#define PID_FREQ_HZ     1000					//Sampling frequency of the PID loop
#define PID_TUNE_OVRRD	false					//If true, no AutoTune is performed an the PID gains are given in the below defines
#define K_P_OVRRD		    203.72				//Kp hard set value
#define K_I_OVRRD		    23967.57			//Ki hard set value
#define K_D_OVRRD		    0.43					//Kd hard set value

//Autotuner configuration:
#define AUTOTUNER_SETPOINT		50				//(steps)
#define AUTOTUNER_OUTPUT_STEP	12000			//output value - must be less than 16384 (INTERNAL_MAX_OUTPUT found in MyIntPID.h)
#define AUTOTUNER_HYSTERESIS	10				//(steps)

//IO pins:
const int CH_A =          2;					//interrupt 0
const int CH_B =          3;					//interrupt 1
const int STEPS =         4;					//interrupt pc20
const int DIR =           5;
const int ENABLE =        6;
const int AUTOTUNE =	    7;
const int MOTOR_OUT_A =   9;					//OC1A
const int MOTOR_OUT_B =   10;					//OC1B
const int ONBOARD_LED =	  13;


/* --------------------------------------------------------------------
 *                           GLOBALS
 *---------------------------------------------------------------------
 */

AutoTune autoTuner(AUTOTUNER_SETPOINT, AUTOTUNER_OUTPUT_STEP, AUTOTUNER_HYSTERESIS);

void setMotorSpeed (int32_t outputValue);
#if ANTIPHASE_LOCK
ArduinoPID pid(setMotorSpeed, D_FILTER_N, PID_FREQ_HZ, 0, TIMER_TOP, &autoTuner);
#else
ArduinoPID pid(setMotorSpeed, D_FILTER_N, PID_FREQ_HZ, -TIMER_TOP, TIMER_TOP, &autoTuner);
#endif



/* --------------------------------------------------------------------
 *                        EXTERNAL INTERRUPTS
 *---------------------------------------------------------------------
 */
ISR(INT0_vect) {
  pid.incrementMeasurement((FastGPIO::Pin<CH_A>::isInputHigh() != FastGPIO::Pin<CH_B>::isInputHigh()) ?  1 : -1);
}

ISR(INT1_vect) {
  pid.incrementMeasurement((FastGPIO::Pin<CH_A>::isInputHigh() != FastGPIO::Pin<CH_B>::isInputHigh()) ? -1 : 1);
}

//TODO maybe use one of INT0 or INT1 for this so that we can have it triggered only on rising edge and ommit the if statement
ISR(PCINT2_vect) {
  if (FastGPIO::Pin<STEPS>::isInputHigh()){
	  pid.incrementSetpoint((FastGPIO::Pin<DIR>::isInputHigh() ^ DIR_INPUT_INV) ? 1 : -1);
  }
}



/* --------------------------------------------------------------------
 *                          INITIALIZERS
 *---------------------------------------------------------------------
 */
void PwmSetup(){
  //setting PWM generator (timer1) and starting it to count:
  ICR1 = TIMER_TOP;

#if ANTIPHASE_LOCK
    TCCR1A = 0b10110010; 
#else
    TCCR1A = 0b10100010;  
#endif 

  TCCR1B = 0b00011001;
  stopMotor();
  
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
  pinMode(STEPS, INPUT_PULLUP);
  pinMode(DIR, INPUT_PULLUP);
  sei();
}



/* --------------------------------------------------------------------
 *                           FUNCTIONS
 *---------------------------------------------------------------------
 */
//abs(outputValue) must not be greater than TIMER_TOP
void setMotorSpeed (int32_t outputValue){

    if (!FastGPIO::Pin<ENABLE>::isInputHigh()){
        stopMotor();
        return;
    }
    
#if ANTIPHASE_LOCK
	OCR1A = outputValue;
	OCR1B = outputValue;
#else
    if (outputValue >= 0){
      OCR1A = outputValue;
      OCR1B = 0;
    } else {
      OCR1A = 0;
      OCR1B = -outputValue;
    }
#endif
}

void stopMotor(){
#if ANTIPHASE_LOCK
	OCR1A = TIMER_TOP / 2;
	OCR1B = TIMER_TOP / 2;
#else
	OCR1A = 0;
	OCR1B = 0;
#endif
	
}

void readSetpoint(){
  if (Serial.available()){
	int32_t setPoint = Serial.parseInt();
    pid.setSetpoint(setPoint);
    Serial.print("new Setpoint:");
    Serial.println(setPoint);
  }
}

void performAutoTune(){
	stopMotor();
	delay(1000);
	pid.reset();
	pid.autoTune();
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

#if PID_TUNE_OVRRD
	pid.setGains(K_P_OVRRD, K_I_OVRRD, K_D_OVRRD);
#else	
	pid.autoSetGains();
#endif
  
  pid.start();
}


/* --------------------------------------------------------------------
 *                           MAIN LOOP
 *---------------------------------------------------------------------
 */
void loop() {
  
  pid.computeInLoop();
#if PID_TUNE_OVRRD == false
  if (!FastGPIO::Pin<AUTOTUNE>::isInputHigh()){performAutoTune();}
#endif
  if (pid.isSteady()){FastGPIO::Pin<ONBOARD_LED>::setOutputLow();} else {FastGPIO::Pin<ONBOARD_LED>::setOutputHigh();}
  readSetpoint();
}
