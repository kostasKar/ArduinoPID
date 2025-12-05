# ArduinoPID

ArduinoPID is an optimized PID controller library with an integrated autotuner for Arduino.
## Features

* Fast, optimized, fixed point, integer-only internal arithmetic 
* 16-bit integer setpoint and measurement values. 
* Selectable output range 
* Derivative filtering with selectable cutoff frequency
* Derivative on measurement
* Integrator anti-windup using tracking back-calculation
* Autotuner (relay method) with analysis output
* floating point controller gains stored in EEPROM with CRC. 


## Performance

* calculate() executes in 44 μs on an Arduino Uno with all features enabled.
* Tested up to 10 kHz control loop frequency

```shouldExecuteInLoop()``` is to be called inside ```loop()``` and checks ```micros()```.
```compute(setpoint, measurement)``` called directly by the application 



## License
[GNU GPLv3](https://choosealicense.com/licenses/gpl-3.0/)
