#ifndef FSMMAIN
#define FSMMAIN
// FSM VAC STECHOQ
//
#define TOTAL_STATE 4
#include <stdbool.h>
#include <stdint.h>
#include "global.h"
#include "stdio.h"
#include "standardWithoutDelayStechoq.h"
#include "Encoder_db.h"
#include "COM_CDC.h"
#include "step_motor.h"
#include "singen.h"
#include "lipmcontrol.h"
void initTrial()
{
}

//int rotatecount = 0;
double targetspeed=0;
void loopMain()
{		
	
	
	switch(state_main){
		case 0:						//Initialize
			state_main=1;
			step_motor(0);
			break;
		case 1:
			if(gJoy.buttons==8 ){
				state_main=2;
			}
			step_motor(0);
			break;
		case 2:			//HOMING
			setspeed=-6000;
			if(limitKiri){
				Encoder_B=0;
				setspeed=-3000;//MAX(-3000,((double)(gJoy.ax))*6000.0/32000);
				state_main=201;
			}
			
			step_motor(setspeed);
			break;
		case 201:
			setspeed=6000;
			if(cmX>40.0){
			
				state_main=3;
			}
			
			step_motor(setspeed);
			break;
		case 3:
			if(HAL_GetTick()-gJoy.last_update_ms<300){
				if(!limitKiri && !limitKanan){
					setspeed=((double)(gJoy.ax))*6000.0/32000;
				}else{
					if(limitKiri){
						Encoder_B=0;
						setspeed=MAX(-3000,((double)(gJoy.ax))*6000.0/32000);
					}else if(limitKanan){
						setspeed=MIN(3000,((double)(gJoy.ax))*6000.0/32000);
					}else{
						setspeed=0;
					}
				}
			}else{
				setspeed=0;
			}
			step_motor(setspeed);
			if(gJoy.buttons==8 ){
				state_main=4;
			}
			if(gJoy.buttons==4 ){
				state_main=6;
			}
			if(gJoy.buttons==1 ){
				state_main=7;
			}
			break;
		case 4:{
			loopSin();
			targetspeed=(outsin-cmX)*5;
			if(targetspeed<-0.1){
				setspeed=-3400+targetspeed*180;
			}else if (targetspeed>0.1){
				setspeed=3400+targetspeed*180;
			}else{
				setspeed=0;
			}
			
			if(setspeed>speed_clamp){
				setspeed=speed_clamp;
			}
			if(setspeed<-speed_clamp){
				setspeed=-speed_clamp;
			}
			step_motor(setspeed);
			if(gJoy.buttons==2 ){
				state_main=5;
			}
			if((cmX>75.0) || (cmX<5.0) || limitKanan || limitKiri){
				state_main=5;
			}
			break;
			
		}
		case 6:
			swingup_controller();
			step_motor(setspeed);
			if( (degree<2&&degree>-2)){
				state_main=7;
			}
			if(gJoy.buttons==2 ){
				state_main=5;
			}
			break;
		case 7:
			balance_controller();
			step_motor(setspeed);
			if( (degree>20||degree<-20)){
				state_main=5;
			}
			if((cmX>75.0) || (cmX<5.0) || limitKanan || limitKiri){
				state_main=5;
			}
			if(gJoy.buttons==2 ){
				state_main=5;
			}
			break;
		case 5:
			
			step_motor(0);
			break;
			
		default:
			state_main=0;
			step_motor(0);
			break;
			
		}
}

#endif
