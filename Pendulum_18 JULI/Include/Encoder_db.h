#ifndef ENC_DB
#define ENC_DB
	#include <stdint.h>
	#include <stdbool.h>
	#include "main.h"
	#include <math.h>
	double deltaRot[4],targetRot[4];
	int16_t lastpos[4],posnow[4];
	int32_t hdposnow[4];
	int32_t startRot[4];
	int32_t tmplastpos;
	int32_t movement;
	double ymotorspeed[4];
	double imotorspeed[4];
	
	int v1_encoder, v2_encoder, last_encoder1, last_encoder, Encoder_A, Encoder_B;
	bool limitKiri,limitKanan;
	void checkLimit(){
		limitKiri=!(HAL_GPIO_ReadPin(Limit2_GPIO_Port, Limit2_Pin));
		limitKanan=!(HAL_GPIO_ReadPin(Limit1_GPIO_Port, Limit1_Pin));
	}
	double degree=0;
double cmX=0;
double encoderRes=4230.0;
void enc2Degree(){
	double wrapped = fmod((double)Encoder_A, encoderRes);
	if (wrapped < 0.0) wrapped += encoderRes;
	degree = wrapped * 360.0 / encoderRes - 180.0;
}
void enc2cm(){
	cmX=(double)(Encoder_B)*69.0/14258.0;
}
static inline float deg2rad(float deg) {
    return deg * 3.14159265358979f / 180.0f;
}
int16_t tim2read;
void zeroPendulumAngle(void){
	Encoder_A = 0;
}
static float theta_rad      = 0.0f;
static float theta_dot_rad  = 0.0f;
static float filtered_theta_dot_rad  = 0.0f;

const float DERIV_ALPHA = 0.33f;   // 0..1, makin besar = lebih halus
void Baca_Encoder(){
	tim2read=(int16_t)(TIM2->CNT);
	Encoder_A = Encoder_A + tim2read;
	Encoder_A = (Encoder_A+2*((int)encoderRes))%((int)encoderRes);
	Encoder_B = Encoder_B - (int16_t)(TIM4->CNT);
	TIM2->CNT=TIM2->CNT-tim2read;
	TIM4->CNT=0;
	enc2Degree();
	enc2cm();
	
	static float last_theta = 0.0f;
	float theta_rad_now = deg2rad(degree);   // degree dari -179 ... +179

    // 2) Hitung SELISIH sudut pakai jarak terpendek di lingkaran
    float dtheta = theta_rad_now - last_theta;
	if (dtheta >  3.14159265f) dtheta -= 2.0f * 3.14159265f;
    if (dtheta < -3.14159265f) dtheta += 2.0f * 3.14159265f;

    float raw_theta_dot = dtheta / 0.001;
	
	theta_dot_rad = 0.99f * theta_dot_rad +
                   0.01f * raw_theta_dot;
	filtered_theta_dot_rad = 0.99f * filtered_theta_dot_rad +
                   0.01f * raw_theta_dot;
									 
    // 4) Simpan sudut sekarang untuk loop berikutnya
    theta_rad = theta_rad_now;
    last_theta = theta_rad_now;
}
#endif
