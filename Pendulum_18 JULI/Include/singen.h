#ifndef SINGEN
#define SINGEN

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "global.h"
double countsin=0,countsinsweep=0;
const double loffset=10.0,roffset=70.0,period=4,periodsweep=90.0;
double w = 2.0 * M_PI  / period; // frekuensi sudut
double wsweep = 2.0 * M_PI  / periodsweep; // frekuensi sudut
double outsin=0;
void loopSin(){
	countsinsweep=countsinsweep+0.005;
	countsin=countsin+0.005*pow(3,sin(countsinsweep*wsweep));
	
	
	if (countsin > 300.0 * M_PI) countsin -= 299.0 * M_PI;
	if (countsinsweep > 300.0 * M_PI) countsinsweep -= 299.0 * M_PI;
	outsin=sin(countsin*w)*(roffset-loffset)/2.0+loffset+(roffset-loffset)/2.0;
}

#endif
