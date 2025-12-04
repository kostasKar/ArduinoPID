
#ifndef DERIVATIVEFILTERED_H
#define DERIVATIVEFILTERED_H

#include <inttypes.h>




//The Transfer Function of a plain derivative is s
//The TF of a single pole filter with cutoff angular velocity N rad/s is N / (s + N)
//A filter with a derivative in cascade give a TF of sN / (s + N)
//The same can be obtained by using a negative feedback loop with a single gain N on the direct path, and a simple integrator on the feedback path
//See video https://www.mathworks.com/videos/understanding-pid-control-part-3-expanding-beyond-a-simple-derivative-1531120808026.html
//Also, since the derivative is going to be multiplied by the gain Kp, we are embedding it into two coefs, so that we save operations
//Important Note!: The cotoff  frequency N should be chosen less than the sampling frequency 1/dt. Or else oscillations will appear 

//                       +
//error--------- K -------O--------- N ----------------------- out
//                        | -                           |
//                        |          1                  |
//                         --------  -  ----------------
//                                   s

//Transfer function: G(s) = KNs / (s + N)


class DerivativeFiltered{

	public:
	int32_t run(int32_t input);
	int32_t getLastOutput(); //without recalculating it and thus without affecting the integrator state

	void reset();

    void setParams(double cutoffRadPerSec, double Gain, double freqHz);

	private:
	int64_t output;
	int64_t outputSum;
	
	int32_t coef1;
	int32_t coef2;
	
};






#endif
