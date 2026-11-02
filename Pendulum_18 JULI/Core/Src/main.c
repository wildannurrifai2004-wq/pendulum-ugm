/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Encoder_db.h"
#include "taskMan.h"
#include "StandardNextion.h"
#include "step_motor.h"
#include "COM_CDC.h"
uint16_t Enc_sampling, Enc_sampling2;
uint16_t Enc_Count, Enc_Count1, enkoder1, enkoder;
int Limit_kiri, Limit_kanan;
uint32_t lastSend = 0;

int angka = 0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t usart3_rx;
uint8_t usart3_buf[100];
uint16_t usart3_len = 0;

char lmt[128];
char msg[65];
char kirim_angka[99];
char text[] = "HELLO\r\n";
int encoder_cnt = 0;
int last_cnt = 0;
int delta = 0;
float angleValue=0;
char printMessage[200]={'\0'};
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

uint8_t usart3_rx;
char kirim[64];



void Encoder_Init(){
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //Encoder
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //Encoder motor

    TIM2->CNT = 0;      // mulai dari 0
    TIM4->CNT = 0;

    Enc_Count = 0;
    Enc_Count1 = 0;
}

void Reset_Enc(){
    Enc_sampling = 0;
    Enc_Count = 0;
    Enc_Count1 = 0;

    Encoder_A = 0;
    Encoder_B = 0;

    last_encoder = 0;
    last_encoder1 = 0;

    TIM2->CNT = 0;
    TIM5->CNT = 0;

}

uint16_t get_encoder (){
    int16_t delta = (int16_t)(TIM2->CNT);  // konversi langsung jadi signed
    TIM2->CNT = 0;                         // reset lagi ke 0
    Enc_Count += delta;
    return Enc_Count;
}

uint16_t get_encoder1 (){
	int16_t delta1 = (int16_t)(TIM4->CNT);
	TIM5->CNT = 0;
	Enc_Count1 += delta1;
	return Enc_Count1;
}



void limit_sens(){
	char lmt[64];

	int Limit_kanan = (HAL_GPIO_ReadPin(Limit1_GPIO_Port, Limit1_Pin) == GPIO_PIN_RESET);
	int Limit_kiri = (HAL_GPIO_ReadPin(Limit2_GPIO_Port, Limit2_Pin) == GPIO_PIN_RESET);
	HAL_Delay(100);

	if(Limit_kiri){
		sprintf(lmt, "Limit1 - PE13: %d\r\n", Limit_kiri);
		CDC_Transmit_FS((uint8_t*)lmt, strlen(lmt));
	}
	else if (Limit_kanan){
		sprintf(lmt, "Limit2 - PE15: %d\r\n", Limit_kanan);
		CDC_Transmit_FS((uint8_t*)lmt, strlen(lmt));
	}
	else {
		sprintf(lmt, "NOTHING \n");
		CDC_Transmit_FS((uint8_t*)lmt, strlen(lmt));
	}
	HAL_Delay(10);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        // Simpan data ke buffer
        if (usart3_len < sizeof(usart3_buf) - 1) {
            usart3_buf[usart3_len++] = usart3_rx;
        }

        // Restart RX interrupt
        Nextion_UART_RxCallback();
        HAL_UART_Receive_IT(&huart3, &usart3_rx, 1);
    }
}


void Nextion_ProcessData_new(uint8_t *buf, uint8_t len) {
    if (len < 4) return;

    // Event tombol dari Nextion = 0x65, page, compID, event
    if (buf[0] == 0x65) {
        uint8_t comp_id = buf[2];
        uint8_t event   = buf[3];

        // Tombol b0 biasanya comp_id = 1
        if (comp_id == 1) {
            if (event == 1) {
               step_motor(6000);   // LED2 ON
            } else {
                stop(); // LED2 OFF
            }
        }

        // -----------------
        // Tombol b1
        // -----------------
        if (comp_id == 2) {
            if (event == 1) {
                step_motor(-6000);   // LED3 ON
            } else {
                stop(); // LED3 OFF
            }
        }
    }
}


void receive_from_nextion(){
    if (Nextion_Available()) {
        const char* msg = Nextion_GetString();
        char kirim[47];

        sprintf(kirim, "RX: %s\r\n", msg);
        CDC_Transmit_FS((uint8_t*)kirim, strlen(kirim));

        if (msg != NULL) {
            if (strcmp(msg, "01") == 0) {
                geserKanan();  // LED ON
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
                 sprintf(kirim, "CEKK \r\n");
                CDC_Transmit_FS((uint8_t*)kirim, strlen(kirim));
            }

            else if (strcmp(msg, "02") == 0){
            	geserKiri();
                sprintf(kirim, "DUAA \r\n");
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
               CDC_Transmit_FS((uint8_t*)kirim, strlen(kirim));
            }
            else {
            	stop();
               sprintf(kirim, "MATII \r\n");
               CDC_Transmit_FS((uint8_t*)kirim, strlen(kirim));
            }

        }

        Nextion_ClearBuffer(); // reset buffer setelah dipakai
        HAL_Delay(100);
    }

}


void Send_ENC(){
	char send[99];
	 sprintf(send, "KIRI -> Encoder_A: %d\r\n KANAN -> Encoder_B: %d\r\n", last_encoder, last_encoder1);
	 CDC_Transmit_FS((uint8_t*)send, strlen(send));
	 HAL_Delay(10);
}

void data_acak(int angka) {
    char alpha[4];      // 3 huruf alphabet
    // Buat alphabet acak
    for (int i = 0; i < 3; i++) {
        alpha[i] = 'A' + (rand() % 26);  // A-Z
    }
    alpha[3] = '\0';

    // Gabungkan alphabet + # + angka
    sprintf(kirim_angka, "%s#%d\r\n", alpha, angka);

    // Kirim ke USB CDC
    CDC_Transmit_FS((uint8_t*)kirim_angka, strlen(kirim_angka));
    HAL_Delay(100);
}


void send_to_Nextion(){
    char enc_text[50];
	sprintf(enc_text,"%d", last_encoder); //counter1
	Nextion_SendText("t1", enc_text);

	sprintf(enc_text,"%d", last_encoder1); //counter1
	Nextion_SendText("t0", enc_text);

	if (Nextion_Available()) {
				// Proses touch event atau data lain
				Nextion_ProcessData();
		}
	HAL_Delay(100);
}
int pinconA[4],pinconB[4];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);


  Encoder_Init();
  Nextion_Init();
	initTask();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		loopTask();
		loopCOM();
		COM_PollAndParseJoystick();
		pinconA[0]= HAL_GPIO_ReadPin(ENC1A_GPIO_Port, ENC1A_Pin);
		pinconA[1]= HAL_GPIO_ReadPin(ENC1B_GPIO_Port, ENC1B_Pin);
		pinconB[0]= HAL_GPIO_ReadPin(ENC_2A_GPIO_Port, ENC_2A_Pin);
		pinconB[1]= HAL_GPIO_ReadPin(ENC_2B_GPIO_Port, ENC_2B_Pin);
		pinconB[2]= HAL_GPIO_ReadPin(Limit1_GPIO_Port, Limit1_Pin);
		pinconB[3]= HAL_GPIO_ReadPin(Limit2_GPIO_Port, Limit2_Pin);
		// sprintf(msg, "Encoder_A: %d -> Encoder_B: %d\r\n", last_encoder, last_encoder1);
		 //CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
			
		 
		
		/*
		 send_to_Nextion();
//		 counter();
		 receive_from_nextion();
		// limit_sens();
		 //Send_ENC();
		 //data_acak(ctrr++);  //TESS KIRIM USB CDC


		// receive_from_nextion();

		 if (Nextion_Available()) {
		     const char* msg = Nextion_GetString();

		     if (msg != NULL) {
		         char debug[50];
		         sprintf(debug, "RX:%s\r\n", msg);
		         CDC_Transmit_FS((uint8_t*)debug, strlen(debug));
		     }

		     Nextion_ClearBuffer();
		 }

	    Nextion_ClearBuffer();




	  if (HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin) == GPIO_PIN_RESET){
//				 sprintf(msg, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
//				 CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

			 if ( HAL_GPIO_ReadPin(Limit2_GPIO_Port, Limit2_Pin) == GPIO_PIN_RESET){

//				 sprintf(lmt, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
//			 	 CDC_Transmit_FS((uint8_t*)lmt, strlen(lmt));
				 stop();
			 }
			 else {
				 step_motor(48000);
				 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				// HAL_UART_Transmit(&huart3, last_encoder, 100, 100);
			 }

	  }
	  else if (HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin) == GPIO_PIN_RESET){
//			 sprintf(msg, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
//			 CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

			 if (HAL_GPIO_ReadPin(Limit1_GPIO_Port, Limit1_Pin) == GPIO_PIN_RESET){
//				 sprintf(lmt, "KIRI -> Encoder_A: %d\r\n Encoder_B: %d\r\n", last_encoder, last_encoder1);
//				 CDC_Transmit_FS((uint8_t*)lmt, strlen(lmt));
				 stop();
			 }
			 else {
				 step_motor(-48000);
				 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			//	 HAL_UART_Transmit(&huart3, last_encoder, 100, 100);

			 }
	  }
	  else {
		 stop();
		 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

	  }

	  if(HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin) == GPIO_PIN_RESET){
		  Reset_Enc();
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	  }		*/



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 10000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65535;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1C10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Limit1_Pin Limit2_Pin */
  GPIO_InitStruct.Pin = Limit1_Pin|Limit2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1C10_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1C10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT3_Pin BT2_Pin */
  GPIO_InitStruct.Pin = BT3_Pin|BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
