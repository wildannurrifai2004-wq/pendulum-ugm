#ifndef DELAYDEF
#define DELAYDEF

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "millis.h"
#define TOTAL_WD 8
long unsigned wd_time_init[TOTAL_WD];
long unsigned wd_waiting_time[TOTAL_WD];

void setWait(int idx, double wait_for_x_second){
  wd_time_init[idx] = millis();
  wd_waiting_time[idx] = wait_for_x_second * 1000;
}

bool isBeeping(int idx){
  if (millis() - wd_time_init[idx]>=wd_waiting_time[idx]){
    return true;
  }
  return false;
}

float percentPassed(int idx){
  return (float)(millis() - wd_time_init[idx]) / (float)(wd_waiting_time[idx]);
}


#endif
