#ifndef SINGEN
#define SINGEN

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "global.h"
double countsin=0,countsinswap=0;
const double loffset=10.0,roffset=70.0,period=4,periodswap=90.0;
double w = 2.0 * M_PI  / period; // frekuensi sudut
double wswap = 2.0 * M_PI  / periodswap; // frekuensi sudut
double outsin=0;
void loopSin(){
	countsinswap=countsinswap+0.01;
	countsin=countsin+0.01*pow(3,sin(countsinswap*wswap));
	
	//coba reset
	if (countsin > 300.0 * M_PI) countsin -= 299.0 * M_PI;
	if (countsinswap > 300.0 * M_PI) countsinswap -= 299.0 * M_PI;
	outsin=sin(countsin*w)*(roffset-loffset)/2.0+loffset+(roffset-loffset)/2.0;
}

#endif
