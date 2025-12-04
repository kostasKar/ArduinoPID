#ifndef PULSESCALER_H
#define PULSESCALER_H

#include <inttypes.h>



/*
 *	we want the output to produce y output pulses in the duratin of x input pulses
 *  y steps can onlly happen together at the time of a x step
 *  symmetricOutput produces a y pulse train that is symmetric. 
 *  I.E. the unequal spots of the y steps are symmetrically distributed within the total length period of the y pulse train
 *  e.g. *'_':step() returned false, '.':step() returned true:
 *  __._____.____._____.____._____.____.____._____.____._____.____._____.__  //symmetric output of PulseScaler(71, 13, true)
 *  _____.____._____.____._____.____._____.____._____.____._____.____.____.  //not symmetric output of PulseScaler(71, 13, false)
 *  Note that step() function may return more than one step per execution, in case that Y is larger thatn X.  
 */

class PulseScaler {

public:
	PulseScaler(unsigned int X = 1, unsigned int Y = 1, bool symmetricOutput = false);
	uint8_t step(); //call for each input step. True if output step needed
	void reset(unsigned int newX, unsigned int newY);
	void reset();

private:
	unsigned int x, y, D;
  	bool symmetric;

};


#endif
