#ifndef MILLIS
#define MILLIS

#include <stdint.h>

#include "main.h"
	static volatile unsigned long int _ulTickCount=0 ;

	uint32_t millis( void )
	{
	// todo: ensure no interrupts
		return HAL_GetTick() ;
	}


#endif
