#include "PulseScaler.h"
#include <inttypes.h>

/* 
Bresenhams algorithm from wikipedia
plotLine(x0,y0, x1,y1)
    dx = x1 - x0
    dy = y1 - y0
    D = 2*dy - dx
    y = y0

    for x from x0 to x1
        plot(x,y)
        if D > 0
               y = y + 1
               D = D - 2*dx
        end if
        D = D + 2*dy
*/

	
//-----------------------------------------------------------------------


/* Testing example:
int main()
{
    int x = 5;
    int y = 74;
    
    int counter = 0;
    
    PulseScaler pS(x,y);
    
    
    for (int i=0; i<x; i++ ){
        unsigned int steps = pS.step();
        cout << steps << ' ';
        counter += steps;
    }
    
    cout << endl << "total steps: " << counter << endl;

    return 0;
}

 */



PulseScaler::PulseScaler(unsigned int X, unsigned int Y, bool symmetricOutput) : x(X), y(Y), symmetric(symmetricOutput) {
	reset();
}

void PulseScaler::reset(unsigned int newX, unsigned int newY){
	x = newX;
	y = newY;
	reset();
}

void PulseScaler::reset(){
	D = (symmetric) ? x/2 : 0;
}

uint8_t PulseScaler::step(){
  D += y;

  uint8_t ret = 0;
  while(D>=x){
    D-=x;
    ret++;
  }
  return ret;
}
	
	
	
	
	
	
	
	
	
	
	
