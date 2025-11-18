/*
 * step_motor.h
 *
 *  Created on: Sep 19, 2025
 *      Author: TulusS'26
 */

#ifndef STEP_MOTOR_H_
#define STEP_MOTOR_H_
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "global.h"
const int speed_clamp=25000;
void step_motor(int pwm1){
	
	if(pwm1<-speed_clamp){
		pwm1=-speed_clamp;
	}else if(pwm1>speed_clamp){
		pwm1=speed_clamp;
	}
	int pwmper10=pwm1*(10.0/65.0);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

	//uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(&htim12);


	if (pwmper10>0){
		htim12.Instance->CCR1 = pwmper10;
		htim12.Instance->CCR2 = 0;
	}
	else if (pwmper10<0){
		htim12.Instance->CCR1 = 0;
		htim12.Instance->CCR2 = -pwmper10;
	}
	else if (pwmper10==0){
		htim12.Instance->CCR1 = 0;
		htim12.Instance->CCR2 = 0;
	}

}

void stop(){
	step_motor(0);
}

void geserKanan(){
	  if (HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin) == GPIO_PIN_RESET){
			// sprintf(msg, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
			 //CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

			 if ( HAL_GPIO_ReadPin(Limit2_GPIO_Port, Limit2_Pin) == GPIO_PIN_RESET){
				// sprintf(lmt, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
				 //CDC_Transmit_FS((uint8_t*)lmt, strlen(lmt));
				 stop();
			 }
			 else {
				 step_motor(5000);
				 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			 }

	  }
}

void geserKiri(){
	   if (HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin) == GPIO_PIN_RESET){
//			 sprintf(msg, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
//			 CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

			 if (HAL_GPIO_ReadPin(Limit1_GPIO_Port, Limit1_Pin) == GPIO_PIN_RESET){
//				 sprintf(lmt, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
//				 CDC_Transmit_FS((uint8_t*)lmt, strlen(lmt));
				 stop();
			 }
			 else {
				 step_motor(-5000);
				 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

			 }
	   }
}

#endif /* STEP_MOTOR_H_ */
