#ifndef GLOBAL
#define GLOBAL
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#include "compiler.h"

#define M_PI 3.14159265358979323846264338327950288
bool beat=0;
bool resetReq=0;
int state_con = 0;
int statemod = 0;
int eepTrig=0;

#define ACTIVE_HIGH 1
#define ACTIVE_LOW  0
#define LEFT  0
#define MOVE  1
#define STOP  0
#define SPEED   0

#define TOTAL_MOTOR 2
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
bool motor_con[TOTAL_MOTOR]= {STOP,STOP};
double  motor_spd[TOTAL_MOTOR]= {SPEED,SPEED};
double counterspeed=0;
int battery=0;

union IntToBytes {
  int raw;
  uint8_t bytes[4];
};
union IntToBytes cmo,zpr,dark,eepromlife,spr,lho,lmi;

const char eeprom_name[][16]={"ee_cyc","cmo","zpr","vi_mode","span","volume","drk","lpwr","ldt","pron","jerk","spdl","posl","norm","hist","here"};
const int rsteepval[16]={0,10000,0,0,1,1,0,0,0,0,0,0,0,0,0,0};


int stepee=0;
union IntToBytes eeNext[16];
int parIdx;
bool startTrig,stopTrig;

char dbg[100]="dsdsdsd";

bool debugNext = 1;//1;//0;//false;
bool uartNext =1;
int state_cut = 0;
int state_trial_step = 0;
int state_cyc_speed =0;
int state_main = 0;

int countstep=0;
UART_HandleTypeDef huart2;

TIM_HandleTypeDef htim10;

uint8_t mydata[20];
uint8_t qwblue[20];
uint8_t iwblue[20];
uint8_t RxData[60];
uint8_t tempRxData[60];
bool isSent;

//typedef PACKED(struct) Tcontrol_status {

//	uint32_t logtick;
//	double degree;
//	double cmX;
//	double setspeed;
//	double   reserved[3];
//} control_status;
typedef struct {
    uint32_t logtick;
    double   degree;
    double   cmX;
    double   setspeed;
    double   reserved[3];
} control_status;
static control_status cs_obj;     // storage nyata, aligned
control_status *cs = &cs_obj;
uint8_t buf[64];
double setspeed;
#endif
