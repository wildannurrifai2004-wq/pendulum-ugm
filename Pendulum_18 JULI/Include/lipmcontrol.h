#ifndef LIPM
#define LIPM

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "global.h"
#include "step_motor.h"
#include "Encoder_db.h"
#include "COM_CDC.h"
	// --- Parameter fisik pendulum ---
// Nilai default hardcode (akan ditimpa GUI via paket 0x04 => gPhysics)
const float g       = 9.81f;   // gravitasi

// Gunakan gPhysics agar GUI bisa update secara live
#define m_p   (gPhysics.mass_kg)    // massa pendulum [kg]
#define l_p   (gPhysics.length_m)   // panjang pendulum [m] (pivot -> CoM)

// Moment inersia - dihitung ulang tiap loop
#define J_p   (m_p * l_p * l_p)

// --- Parameter loop kontrol ---
const float Ts      = 0.01f;   // 10 ms loop control

// --- Batas rel / posisi ---
const float X_MIN   = 5.0f;    // cm (kiri)
const float X_MAX   = 64.0f;   // cm (kanan)
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
    // Sekarang top = 0, bottom +-180 -> kamu bisa langsung pakai degree
    

    // koreksi kalau lompat lewat -p / +p
    

    // 3) Low-pass filter derivatif
    



    // ---------------- posisi cart, sama seperti sebelumnya ----------------
    float raw_x_dot = (cmX - last_x) / 0.001; // cm/s
    x_dot_cm_s = 0.99 * x_dot_cm_s +
                (1.0f - 0.99) * raw_x_dot;
    last_x = cmX;
}

// FIX: kinC dan potC tidak bisa diinisialisasi di scope global dengan
// ekspresi non-konstan (gPhysics struct). Dideklarasikan tanpa nilai
// awal, lalu dihitung ulang di dalam calculateEnergy() setiap pemanggilan.
float energy_now = 0;
double kinC; // dihitung runtime di calculateEnergy()
double potC; // dihitung runtime di calculateEnergy()

float calculateEnergy(){
    // Hitung kinC dan potC dari nilai gPhysics terkini (runtime)
    kinC = 0.5 * J_p * 0.6;
    potC = m_p * g * l_p;
    if(state_main==6){
        if(degree>150 || degree<-150){
            return energy_now;
        }else{
            return energy_now+(kinC * theta_dot_rad * theta_dot_rad
                + potC * (1.0f + cosf(theta_rad))-energy_now)*0.3;
        }
    }else{
        return energy_now+(kinC * theta_dot_rad * theta_dot_rad
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
    float E_des = 2.0f * m_p * g * l_p * gSwing.margin_E;  //14.7f;  // energi di posisi tegak atas dalam definisi ini
		float E_tilde = E - E_des;            // selisih energi
    // 2) Gain energi (disesuaikan untuk m_p = 0.158 kg agar magnitude u_energy ~50 saat ayunan awal)
    float kE = gSwing.kE; // dinamis dari gSwing via GUI   // sebelumnya 8.0f saat massa salah (1.58 kg)
		
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
    float kx_swing = gSwing.kx_swing; // dinamis dari gSwing via GUI   // cm -> command
    float x_err = cmX - 40.0;
    float u_pos = -kx_swing * x_err;

    v_cmd =  u_pos+v_energy;//;   // di sini kita anggap "kecepatan cart target"
		if(v_cmd<-0.1){
				setspeed=-4400+v_cmd*180;
			}else if (v_cmd>0.1){
				setspeed=4400+v_cmd*180;
			}else{
				setspeed=0;
			}
		if(cmX < 5.0f || cmX > 64.0f){
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
float X_CENTER = 34.43;
// Fs = 100 Hz (Ts = 0.01 s)
const float const_a = 0.27f;   // cutoff sekitar 5 Hz
const float const_b = 0.99f;   // drift DC pelan-pelan turun (~1 s time constant)
int counte_x=0;
int counte_xd=0;
float e_x_d ;

void stickMove(){
// Deadzone untuk stick (ignore small movements)
    const int16_t DEADZONE = 2000;  // +-2000 = deadzone tengah
		// Cek apakah joystick masih fresh (terima data dalam 300ms terakhir)
    if (HAL_GetTick() - gJoy.last_update_ms < 300) {
        
        // Hitung shift berdasarkan analog input
        if (abs(gJoy.ax) > DEADZONE) {
            // Map ax (-32000 to +32000) ke shift rate
            // Max shift = +-0.3 cm per loop (at 100 Hz = +-30 cm/s max)
            float normalized = (float)gJoy.ax / 32000.0f;  // -1.0 to +1.0
            float shift_rate = normalized * 0.3f;  // cm per loop
            
            X_CENTER += shift_rate;
            
            // Clamp X_CENTER ke batas aman
            if (X_CENTER < 5.0f)  X_CENTER = 5.0f;
            if (X_CENTER > 75.0f) X_CENTER = 75.0f;
        }
    }
    else {
        // Joystick timeout - kembali ke center default
        // Smooth transition ke 40cm
        float target = 40.0f;
        float diff = target - X_CENTER;
        X_CENTER += diff * 0.01f;  // Slow drift back to center
    }
}

float balance_controller()
{
		stickMove();
		//_CENTER=40;
    
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
      
		e_x_int += e_x * 0.005f; // loop period 5ms
		if(e_x_int > 40.0f){
			e_x_int = 40.0f;
		}
		if(e_x_int < -40.0f){
			e_x_int = -40.0f;
		}
    // Gain (silakan tuning, ini hanya contoh angka awal)
		float K_TH     = gGains.K_TH;
    float K_TH_D   = gGains.K_TH_D;
    float K_X      = gGains.K_X;
    float K_X_D    = gGains.K_X_D;
    float K_X_INT  = gGains.K_X_INT;

		u_theta   = K_TH   * e_theta;
		u_theta_d = K_TH_D * e_theta_d;
		u_e_x     = K_X    * e_x;
		u_e_xd    = K_X_D  * e_x_d;
		u_e_xint  = K_X_INT* e_x_int;
    u = -( u_theta
         + u_theta_d
         + u_e_x
         + u_e_xd
         + u_e_xint);
		
		
    v_cmd= u;//v_cmd+(u-v_cmd)*0.7;
		if(v_cmd>200){
			v_cmd=200;
		}else if(v_cmd<-200){
			v_cmd=-200;
		}
		if(v_cmd<-0.1){
				setspeed=-4400+v_cmd*180;
				if(setspeed>0){
					setspeed=0;
				}
			}else if (v_cmd>0.1){
				setspeed=4400+v_cmd*180;
				if(setspeed<0){
					setspeed=0;
				}
			}else{
				setspeed=0;
			}
		if(cmX < 5.0f || cmX > 64.0f){
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
				setspeed=-4400+v_cmd*180;
				if(setspeed>0){
					setspeed=0;
				}
			}else if (v_cmd>0.1){
				setspeed=4400+v_cmd*180;
				if(setspeed<0){
					setspeed=0;
				}
			}else{
				setspeed=0;
			}
		if(cmX < 5.0f || cmX > 64.0f){
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
