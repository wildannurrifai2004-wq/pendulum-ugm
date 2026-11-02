#define TASK_NUM 6
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "millis.h"
#include "standardWithoutDelayStechoq.h"
#include "Encoder_db.h"
#include "COM_CDC.h"
#include "global.h"
#include "step_motor.h"
#include "singen.h"
#include "FSMmain.h"
#include "lipmcontrol.h"
/** Tick Counter united by ms */


bool led;
double task_delay[TASK_NUM] = {
 // 0.05, 0.01, 0.1, 0.01, 0.05, 0.2
	0.05, 0.005, 0.1, 0.001, 0.02, 0.005

};
char task_name[TASK_NUM][10]  = {
  "MAIN", "CONTROL", "UICOM", "DEBUG COM"
};
double percentUsage[TASK_NUM] = {
  0, 0, 0, 0, 0, 0
};
void tes() {

};
unsigned long ltime;;
void initTask() {
  for (int i = 0; i < TASK_NUM; i++) {
    setWait(i, task_delay[i]);
  }
}
float loopspeed[TASK_NUM] = {0};
long unsigned lotime[TASK_NUM];

void loopTask() {
	//counterglobal=counterglobal+1;
//	char str[32];


  for (int i = 0; i < TASK_NUM; i++) {
		
     
    if (isBeeping(i) ) {
			setWait(i, task_delay[i]);
       percentUsage[i] += 1;
      loopspeed[i] = loopspeed[i] + ((float)(millis() - lotime[i]) - loopspeed[i]) / ((float)1 / (float)0.02);
						//matlabLoop();
            lotime[i] = millis();
      switch (i) {
        case 0 : {
						//if(Encoder_A>0.001){
						//	Encoder_A=Encoder_A-0.001;
						//}else if(Encoder_A<-0.001){
						//	Encoder_A=Encoder_A+0.001;
						//}
					 // PARSING RX byte-per-byte
					checkLimit();
					
					if (gJoy.fresh) {           // paket lengkap sudah diterima & valid CRC
						gJoy.fresh = 0;
						// pakai gJoy.ax, gJoy.ay, gJoy.rx, gJoy.ry, gJoy.buttons, gJoy.seq
					}
            break;
          }
        case 1 : {
						//setspeed=0;
						step_motor(0);
						
						energy_now=calculateEnergy();
						//swingup_controller();
						loopMain();
						
						//loopSin();
            break;
          }
        case 2 : {
						
						//const char *hb = "HB\r\n";
						//COM_Write((const uint8_t*)hb, strlen(hb));
					

					/* contoh: baca data masuk */
					
					 //uint32_t n = COM_Available() ? COM_Read(buf, sizeof(buf)) : 0;
					//if (n){
						// proses data n byte
					//}
					
					//COM_PollAndParseJoystick(); 
            break;
          }
        case 3 : {
					Baca_Encoder();
					update_states_from_sensors();
						break;
          }
        case 4 : {
						cs->logtick=millis();
						cs->cmX=cmX;
						cs->degree=degree;
						cs->setspeed=setspeed;
						//cs->reserved[0]=outsin;
						cs->reserved[0]=energy_now;
						cs->reserved[1]=theta_dot_rad;
						cs->reserved[2]=theta_rad;
						//extern float X_CENTER;  // from lipmcontrol.h
						cs->reserved[3] = X_CENTER;  // TAMBAH INI!
						
						Send_ControlStatus(cs);

            break;
          }

        case 5 : 
					
					break;

        default : break;
      }
     
      break;

    }
		
    //Serial.print("P");
    //Serial.print(i);
    //Serial.print(" ");
    //Serial.println(percentPassed(i));

  }
  for (int i = 0; i < TASK_NUM; i++) {
    percentUsage[i] *= 0.9999;
  };

  //Serial.println(millis()-ltime);
  ltime = millis();
}
