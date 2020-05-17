# ArduinoPID

ArduinoPID is an optimized PID controller library with autotuner for the arduino

## Features

* Fast optimized integer-only internal arithmetic manipulations 
* 32-bit integer setpoint and measurement values. Selectable output range 
* Function pointer used for direct output handling
* Derivative filtering with selectable cutoff frequency
* Derivative on measurement or on error
* Integrator anti-windup
* Autotuner (relay method) with analysis output
* floating point controller gains stored in EEPROM with crc. 


## Performance

~ 69 μsec per execution with all features enabled. Tested up to 10kHz control loop frequency.
```computeInLoop()``` is to be called inside ```loop()``` and checks the ```micros()```.
```compute()``` can be alternatively called inside a timer ISR for even better performance 



## License
[GNU GPLv3](https://choosealicense.com/licenses/gpl-3.0/)
