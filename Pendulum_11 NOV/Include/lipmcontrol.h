#ifndef LIPM
#define LIPM

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "global.h"
#include "step_motor.h"
#include "Encoder_db.h"
	// --- Parameter fisik pendulum (silakan sesuaikan) ---
const float g       = 9.81f;   // gravitasi
const float m_p     = 2.0f;    // massa pendulum (kg)
const float l_p     = 0.375f;   // panjang pendulum (m) (pivot -> CoM)

// Moment inersia aproksimasi batang homogen sekitar pivot
const float J_p     = m_p * l_p * l_p; 

// --- Parameter loop kontrol ---
const float Ts      = 0.01f;   // 10 ms loop control

// --- Batas rel / posisi ---
const float X_MIN   = 5.0f;    // cm (kiri)
const float X_MAX   = 75.0f;   // cm (kanan)
//const float X_CENTER= 40.0f;   // cm (tengah rel, kira-kira)

// --- Batas sudut untuk masuk mode balance ---
const float THETA_BALANCE_DEG = 10.0f;  // |theta| < ini -> kita anggap sudah di dekat atas
const float THETA_DOT_BALANCE = 2.0f;   // rad/s, boleh kamu kecilkan nanti

enum PendulumMode {
    PEND_IDLE = 0,
    PEND_SWINGUP,
    PEND_BALANCE,
    PEND_FAULT
};

enum PendulumMode pend_mode = PEND_IDLE;



// Turunan dan state internal

static float x_dot_cm_s     = 0.0f;

// Filter derivatif

// Konversi derajat -> radian


// Sign function
inline float sgn(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

void update_states_from_sensors()
{
    
    static float last_x     = 0.0f;

    // 1) Baca sensor & konversi ke radian
    // Sekarang top = 0°, bottom ˜ ±180° ? kamu bisa langsung pakai degree
    

    // koreksi kalau lompat lewat -p / +p
    

    // 3) Low-pass filter derivatif
    


    // ---------------- posisi cart, sama seperti sebelumnya ----------------
    float raw_x_dot = (cmX - last_x) / 0.001; // cm/s
    x_dot_cm_s = 0.99 * x_dot_cm_s +
                (1.0f - 0.99) * raw_x_dot;
    last_x = cmX;
}
float energy_now=0;
double kinC=0.5f * J_p*0.6;
double potC=m_p * g * l_p;
float calculateEnergy(){
	if(state_main==6){
		if(degree>150 || degree<-150){
			return energy_now;////1.8 * (theta_dot_rad * theta_dot_rad) + 147 * (1 + cosf(theta_rad));
		
		}else{
				return //1.8 * (theta_dot_rad * theta_dot_rad) + 147 * (1 + cosf(theta_rad));
				energy_now+(kinC * theta_dot_rad * theta_dot_rad
            + potC * (1.0f + cosf(theta_rad))-energy_now)*0.3;
		}
	}else{
	return //1.8 * (theta_dot_rad * theta_dot_rad) + 147 * (1 + cosf(theta_rad));
		energy_now+(kinC* theta_dot_rad * theta_dot_rad
            +potC * (1.0f + cosf(theta_rad))-energy_now)*0.3;
}
}

// --- helper nonlinear ---
float shape_nonlinear(float u)
{
    float s   = (u >= 0.0f) ? 1.0f : -1.0f;
    float mag = fabsf(u);

    // sqrt nonlinearity
    float mag_shaped = sqrtf(mag);      // atau powf(mag, 0.4f), silakan coba

    const float kV = 8;              // tuning
    return s * kV * mag_shaped;
}
float u_energy ;
float E;
 float v_cmd ;
float swingup_controller()
{
    // 1) Hitung energi
    E = energy_now;//0.5f * J_p * theta_dot_rad * theta_dot_rad
          //  + m_p * g * l_p * (1.0f + cosf(theta_rad));
    float E_des = 2.0f * m_p * g * l_p*1.05;  //14.7f;  // energi di posisi tegak atas dalam definisi ini
		float E_tilde = E - E_des;            // selisih energi
    // 2) Gain energi (silakan tuning)
    const float kE = 8.0f;   // coba 10..100, lihat respon
		
		if(degree>150 || degree<-150){
			if(theta_dot_rad * cosf(theta_rad)>0){
				u_energy= kE * (E - E_des) * 1* fabs(cosf(theta_rad));
			}else{
				u_energy= kE * (E - E_des) * -1* fabs(cosf(theta_rad));
			}
		}else if(degree>90||degree<-90){
			u_energy=u_energy/pow(1.06,fabs(theta_dot_rad)/4.0);
		}else{
			u_energy=0;
		}
		float v_energy = shape_nonlinear(u_energy);
    // 3) Penjaga posisi: tarik ke tengah rel
    const float kx_swing = 0.5f;   // cm ? command
    float x_err = cmX - 40.0;
    float u_pos = -kx_swing * x_err;

    v_cmd =  u_pos+v_energy;//;   // di sini kita anggap "kecepatan cart target"
		if(v_cmd<-0.1){
				setspeed=-3400+v_cmd*180;
			}else if (v_cmd>0.1){
				setspeed=3400+v_cmd*180;
			}else{
				setspeed=0;
			}
		if(cmX<5.0||cmX>75){
			setspeed=0;
		}
    // 4) Konversi ke setspeed (sementara linear saja, seperti punyamu)
		//setspeed = (v_cmd - 3483.0f) / 190.2f;  // membalik poly: x = (f - p2)/p1
		
    // 5) Clamp
    if (setspeed >  speed_clamp) setspeed =  speed_clamp;
    if (setspeed < -speed_clamp) setspeed = -speed_clamp;

    return setspeed;
}
float e_x ,e_x_int;
float u ;
float u_theta,u_theta_d,u_e_x,u_e_xd,u_e_xint;
float braker=0;
float degreemean=0;
float X_CENTER;
// Fs = 100 Hz (Ts = 0.01 s)
const float const_a = 0.27f;   // cutoff sekitar 5 Hz
const float const_b = 0.99f;   // drift DC pelan-pelan turun (~1 s time constant)
int counte_x=0;
int counte_xd=0;
float e_x_d ;
float balance_controller()
{
		
		X_CENTER=40;
    // Error sudut dan posisi
    float e_theta   = theta_rad;              // 0 rad = atas
    float e_theta_d = filtered_theta_dot_rad;
		// Cascade scheduling
		counte_x++;
		counte_xd++;
		if(counte_xd>0){
			counte_xd=0;
			e_x_d  = x_dot_cm_s;
		}
		if(counte_x>0){
			counte_x=0;
			e_x       = (X_CENTER-cmX);//0.5*(X_CENTER-cmX)+0.5*fabs(X_CENTER-cmX)*(X_CENTER-cmX)/40.0 ;
			
		}
      
		e_x_int += e_x * 0.01;
		if(e_x_int>40){
			e_x_int=40;
		}
		if(e_x_int<-40){
			e_x_int=40;
		}
    // Gain (silakan tuning, ini hanya contoh angka awal)

    float K_TH   = -2.50f*57.0*12.0;    // rad
    const float K_TH_D = -0.030f*57.0*18.0;     // rad/s
		float K_X    = 3.0f;      // cm
		
		float K_X_D  = -1.6*2;//-0.8f;      // cm/s
		float K_X_INT  = 0;//-1.0f;//-0.8f;      // cm/s

		u_theta=K_TH   * e_theta;
		u_theta_d=K_TH_D * e_theta_d;
		u_e_x=K_X    * e_x;
		u_e_xd=K_X_D  * e_x_d;
		u_e_xint=K_X_INT*e_x_int;
    u= -( u_theta
               + u_theta_d
               + u_e_x
               + u_e_xd
								+u_e_xint);
		
		
    v_cmd= u;//v_cmd+(u-v_cmd)*0.7;
		if(v_cmd>200){
			v_cmd=200;
		}else if(v_cmd<-200){
			v_cmd=-200;
		}
		if(v_cmd<-0.1){
				setspeed=-3400+v_cmd*180;
				if(setspeed>0){
					setspeed=0;
				}
			}else if (v_cmd>0.1){
				setspeed=3400+v_cmd*180;
				if(setspeed<0){
					setspeed=0;
				}
			}else{
				setspeed=0;
			}
		if(cmX<5.0||cmX>75){
			setspeed=0;
		}
		if(degree>20.0||degree<-20){
			setspeed=0;
		}
    // clamp
    if (setspeed >  speed_clamp) setspeed =  speed_clamp;
    if (setspeed < -speed_clamp) setspeed = -speed_clamp;

    return setspeed;
}

float balance_controller2()
{
		
		X_CENTER=40;
    // Error sudut dan posisi
		e_x       =  (X_CENTER-cmX);
    float e_theta   = theta_rad;//+e_x/400;              // 0 rad = atas
    float e_theta_d = filtered_theta_dot_rad;
	
    //fabs(X_CENTER-cmX)*(X_CENTER-cmX)/40.0 ;
		
    float e_x_d     = x_dot_cm_s;

    // Gain (silakan tuning, ini hanya contoh angka awal)

    float K_TH   = -2.50f*57.0*8.0;    // rad
    const float K_TH_D = -0.030f*57.0*12.0;     // rad/s
		float K_X    = 8.0f;      // cm
		
		float K_X_D  = -0.8f;      // cm/s
		
		u_theta=K_TH   * e_theta;
		u_theta_d=K_TH_D * e_theta_d;
		u_e_x=0;//K_X    * e_x;
		u_e_xd=0;//K_X_D  * e_x_d;
    u= -( u_theta
               + u_theta_d
               + u_e_x
               + u_e_xd );
		
	
    v_cmd= v_cmd+u*0.03;
		if(v_cmd>200){
			v_cmd=200;
		}else if(v_cmd<-200){
			v_cmd=-200;
		}
		if(v_cmd<-0.1){
				setspeed=-3400+v_cmd*180;
				if(setspeed>0){
					setspeed=0;
				}
			}else if (v_cmd>0.1){
				setspeed=3400+v_cmd*180;
				if(setspeed<0){
					setspeed=0;
				}
			}else{
				setspeed=0;
			}
		if(cmX<5.0||cmX>75){
			setspeed=0;
		}
		if(degree>20.0||degree<-20){
			setspeed=0;
		}
    // clamp
    if (setspeed >  speed_clamp) setspeed =  speed_clamp;
    if (setspeed < -speed_clamp) setspeed = -speed_clamp;

    return setspeed;
}
#endif
