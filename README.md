# ArduinoPID

ArduinoPID is an optimized PID controller library with an integrated autotuner for Arduino.
## Features

* Fast, optimized, fixed point, integer-only internal arithmetic 
* Wrap-safe (encoder-friendly) setpoint and measurement handling
* Selectable output range 
* Derivative filtering with selectable cutoff frequency
* Derivative on measurement
* Integrator anti-windup using tracking back-calculation
* Autotuner (relay method) with analysis printout
* Floating-point PID gains stored in EEPROM with CRC integrity check

## Performance

The duration of a call to `compute()` was benchmarked on ATmega328P (Arduino UNO)

| Scenario                                | Time (µs) |
| ----------------------------------------| --------- |
| PID control — derivative **filtered**   | 43.68     |
| PID control — derivative **unfiltered** | 36.96     |
| PI control (Kd = 0)                     | 27.84     |

> **Note:** Integral anti-windup is enabled in all scenarios.

```shouldExecuteInLoop()``` is to be called inside ```loop()``` and checks ```micros()``` to maintain timing.<br>
```compute(setpoint, measurement)``` called directly by the application 


 ## Usage

 ### 1. Create a PID controller

```
#include <ArduinoPID.h>

// Arguments:
// (sampling frequency Hz, outputMin, outputMax, derivativeFiltering, customCutoffHz)
ArduinoPID pid(
    1000.0,          // 1 kHz sampling rate
    -255, 255,       // output limits
    LOW_FILTERING,   // derivative filtering level
    1.0              // cutoff frequency (used only when CUSTOM_CUTOFF_HZ is selected)
);

```

 ### 2. Configure PID gains

 ```
 pid.setParameters(1.2f, 0.5f, 0.0f);   // Kp, Ki, Kd
 ```

 Or using a PIDGains object:

 '''
PIDGains gains = {1.2f, 0.5f, 0.0f};
pid.setParameters(gains);
 '''

### 3. Autotuning

 Create and configure an Autotuner object:
 
 ```
 #include <AutoTuner.h>

 AutoTuner autoTuner(
    AUTOTUNER_SETPOINT,     //Setpoint around which the plant will oscillate
    AUTOTUNER_OUTPUT_STEP,  //The output to the actuator (positive and negative)
    AUTOTUNER_HYSTERESIS    //A small hysteresis makes the method more robust (setpoint domain)
 );
 ```

 Initialize for running the autotuning procedure:

 ```
 autoTuner.init();
 ``` 

 The autotuning loop (appApplyOutput is user defined and drives the actuator)
 measurement is the current measurement

 ```
 while (!autotuner.isFinished()){
    appApplyOutput(autotuner.run(measurement));
 }
 ```

 Get the autotuner results PIDGains object. Store them to EEPROM
 ```
 PIDGains pidGains = autotuner.getPIDGains();
 pidGains.saveToEEPROM();
 ```

 The application can check for existing gains in eeprom to avoid autotuning on each execution:
 ```
 PIDGains pidGains;
 if (pidGains.readFromEEPROM()){
    pid.setParameters(pidGains);
 }
 ```

 Check for configuration errors. The controller will not run if any

```
ConfigError err = pid.getConfigError();
if (err != NO_ERROR) {
    Serial.println("PID configuration error");
}
```

 ### 3. Run inside loop()

 Call shouldExecuteInLoop() inside loop().
 When it returns true, call compute().

```
void loop() {
    if (pid.shouldExecuteInLoop()) {
        appApplyOutput(pid.compute(setpoint, measurement));
    }
}
```

 ### 4. Reset the controller

 The internal state of the controller can be reset.
 Optionally, the user can apply the current measurement during reset so that 
 there is no derivative kick if it is reset at a non-zero position

```
pid.reset();
pid.reset(currentMeasurement);
```
 ---

 ## Derivative Filtering Options

| Option             | Description                          |
| ------------------ | ------------------------------------ |
| `NO_FILTERING`     | Raw derivative (fastest, most noise) |
| `LOW_FILTERING`    | Light smoothing (default)            |
| `MEDIUM_FILTERING` | more aggresive smoothing             |
| `HIGH_FILTERING`   | heavy filtering                      |
| `CUSTOM_CUTOFF_HZ` | Use custom cutoff frequency          |

 ---


## License
[GNU GPLv3](https://choosealicense.com/licenses/gpl-3.0/)
