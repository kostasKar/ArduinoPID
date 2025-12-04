# ArduinoPID

ArduinoPID is an optimized PID controller library with autotuner for the arduino

## Features

* Fast optimized fixed point integer-only internal arithmetic 
* 16-bit integer setpoint and measurement values. Selectable output range 
* Derivative filtering with selectable cutoff frequency
* Derivative on measurement
* Integrator anti-windup with tracking back-calculation
* Autotuner (relay method) with analysis output
* floating point controller gains stored in EEPROM with crc. 


## Performance

~ 48 μsec per execution with all features enabled in an arduino uno. Tested up to 10kHz control loop frequency.
```shouldExecuteInLoop()``` is to be called inside ```loop()``` and checks ```micros()```.
```compute(setpoint, measurement)``` called directly by the application 



## License
[GNU GPLv3](https://choosealicense.com/licenses/gpl-3.0/)
