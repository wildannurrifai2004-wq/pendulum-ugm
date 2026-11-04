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

void resetControllerVariables(void)
{
    // Reset integral term
    e_x_int = 0.0f;
    
    // Reset velocity command
    v_cmd = 0.0f;
    
    // Reset energy (untuk swingup)
    energy_now = 0.0f;
    
    // Reset counters
    counte_x = 0;
    counte_xd = 0;
    
    // Reset errors
    e_x = 0.0f;
    e_x_d = 0.0f;
    
    // Clear joystick fresh flag
    gJoy.fresh = 0;
}

void loopMain()
{		
	// Check for reset request dari GUI
    if (resetReq) {
        resetReq = false;  // Clear flag
        
        // Reset variables
        resetControllerVariables();
        
        // Kembali ke state READY (manual control)
        state_main = 1;
        
        // Optional: print debug
        sprintf(dbg, "RESET OK");
    }
	
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
			setspeed=-8000;
			if(limitKiri){
				Encoder_B=0;
				setspeed=-5000;//MAX(-3000,((double)(gJoy.ax))*6000.0/32000);
				state_main=201;
			}
			
			step_motor(setspeed);
			break;
		case 201:
			setspeed=8000;
			if(cmX>34.5){
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
				resetControllerVariables();
				state_main=6;
			}
			if(gJoy.buttons==1 ){
				resetControllerVariables();
				state_main=7;
			}
			break;
			
		case 4:
			loopSin();
			{
				float e_x = (float)(gChirp.outsin - cmX);
				float e_v = (float)(gChirp.target_vel - x_dot_cm_s);
				float u_cmd = (160.0f * e_x) + (20.0f * e_v);

				if (u_cmd > speed_clamp) u_cmd = speed_clamp;
				if (u_cmd < -speed_clamp) u_cmd = -speed_clamp;
				setspeed = (double)u_cmd;
			}
			
			// SISTEM KEAMANAN KERETA: Anti keluar rel, anti overspeed, & limit switch
			if ((cmX > 68.0) || (cmX < 12.0) || limitKanan || limitKiri || (fabsf(x_dot_cm_s) > 85.0f)) {
				state_main = 5; // STOP & TRIP KEAMANAN
				setspeed = 0;
				step_motor(0);
				break;
			}
			
			step_motor(setspeed);
			if (gJoy.buttons == 2) {
				state_main = 5;
				setspeed = 0;
				step_motor(0);
			}
			break;
		
		case 6:
			swingup_controller();
			step_motor(setspeed);
			// Masuk mode balance jika posisi dekat atas (|degree| < 12) dan kecepatan rotasi wajar (|theta_dot_rad| < 3.5)
			if ((fabsf((float)degree) < gSwing.catch_deg) && (fabsf(theta_dot_rad) < gSwing.catch_vel)) {
				resetControllerVariables();
				state_main = 7;
			}
			if(gJoy.buttons==2 ){
				state_main=5;
			}
			if((cmX > 75.0) || (cmX < 5.0) || limitKanan || limitKiri){
				state_main = 5;
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
