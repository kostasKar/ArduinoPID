# ArduinoPID

ArduinoPID is an optimized PID controller library with an integrated autotuner for Arduino.
## Features

* Fast, optimized, fixed point, integer-only internal arithmetic 
* Wrap-safe encoder friendly setpoint and measurement
* Selectable output range 
* Derivative filtering with selectable cutoff frequency
* Derivative on measurement
* Integrator anti-windup using tracking back-calculation
* Autotuner (relay method) with analysis output
* floating point controller gains stored in EEPROM with CRC. 

## `compute()` Performance Benchmark on ATmega328P

| Scenario                                | Time (µs) |
| ----------------------------------------| --------- |
| PID control — derivative **filtered**   | 43.68     |
| PID control — derivative **unfiltered** | 36.96     |
| PI control (Kd = 0)                     | 27.84     |

> **Note:** Integral anti-windup is enabled in all scenarios.

```shouldExecuteInLoop()``` is to be called inside ```loop()``` and checks ```micros()```<br>
```compute(setpoint, measurement)``` called directly by the application 



## License
[GNU GPLv3](https://choosealicense.com/licenses/gpl-3.0/)
