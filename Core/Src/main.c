/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4


#include <float.h>
#include "filter_coefficients.h"
#include "audio_out_filters.h" //Store various filters here so that bandwidth can be switched?
#include "arm_math.h"

#include "si5351.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t read_enc(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLOAT_MAX FLT_MAX
#define FLOAT_MIN (-FLT_MAX)

#define BLOCK_SIZE_FLOAT 512
#define BLOCK_SIZE_U16 2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

arm_fir_instance_f32 firsettings_l, firsettings_r, firsettings_rl_sum;

arm_biquad_cascade_df2T_instance_f32 S;

//Measurement values: pwr, swr, voltages, currents, temperature etc...


//Mode
uint16_t mode = 0; //LSB = 0, USB = 1, TODO in future read this from  "EEPROM"
uint16_t tx_flag = 0; //tx enable = 1

//VFO variables
uint32_t vfo_a = 12e6;			//Load from memory on start
uint32_t vfo_b = 100e6;			//Load from memory on start
uint32_t tayloe_f = 120e6;  	//4x current VFO
uint16_t step = 1e3;			//Load from memory on start
uint16_t current_vfo = 0; 		//Load from memory on start


uint16_t encoderValue = 32768;
uint16_t tmp_encValue = 32768;
uint32_t last_debounce = 0;
uint32_t now_debounce = 0;

//PLL config

si5351OutputConfig_t out_conf;

si5351PLLConfig_t pll_conf;

//Volume and amplitude tuning
float rx_volume = 1;
float tx_power = 1;

float rx_amp_cal = 1;
float tx_amp_cal = 1.005;

//Filters states, sample buffers variables
//fir state size is always number_of_samples + number_of_fir_tabs - 1
float fir_l_state [BLOCK_SIZE_FLOAT + FILTER_TAP_NUM - 1];
float fir_r_state [BLOCK_SIZE_FLOAT + FILTER_TAP_NUM - 1];

float fir_rl_sum_state [BLOCK_SIZE_FLOAT + FILTER_TAP_NUM - 1];

uint16_t rxBuf[BLOCK_SIZE_U16*2];
uint16_t txBuf[BLOCK_SIZE_U16*2];
float l_buf_in [BLOCK_SIZE_FLOAT*2];
float r_buf_in [BLOCK_SIZE_FLOAT*2];
float l_buf_out [BLOCK_SIZE_FLOAT*2];
float r_buf_out [BLOCK_SIZE_FLOAT*2];


float sum_buf_rl [BLOCK_SIZE_FLOAT*2];
float audio_out [BLOCK_SIZE_FLOAT*2];

float32_t *InputValuesf32_ptr = &sum_buf_rl[0];  // declare Input pointer
float32_t *OutputValuesf32_ptr = &audio_out[0]; // declare Output pointer

uint8_t callback_state = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  TIM4->CNT = encoderValue;

  const int32_t correction = -2e3;
  //si5351 advanced interface config for IQ
  si5351_Init(correction);

  //si5351_SetupCLK0(12e6, SI5351_DRIVE_STRENGTH_4MA);
  //si5351_SetupCLK2(12e6, SI5351_DRIVE_STRENGTH_4MA);
  //si5351_EnableOutputs((1<<0) | (1<<2));

  arm_fir_init_f32(&firsettings_l, FILTER_TAP_NUM, &filter_p45[0], &fir_l_state[0], BLOCK_SIZE_FLOAT);
  arm_fir_init_f32(&firsettings_r, FILTER_TAP_NUM, &filter_m45[0], &fir_r_state[0], BLOCK_SIZE_FLOAT);

  arm_fir_init_f32(&firsettings_rl_sum, AUDIO_FIR_LEN, &audio_out_filter24_n150[0], &fir_rl_sum_state[0], BLOCK_SIZE_FLOAT); //Works fine with 100 coefficients, 250 is too much processing wise for real time!


  //Test of IIR
  //arm_biquad_cascade_df2T_init_f32(&S, NUM_STAGE_IIR, &iirCoeffs[0], &iirState[0]);

  float32_t *InputValuesf32_ptr = &sum_buf_rl[0];  // declare Input pointer
  float32_t *OutputValuesf32_ptr = &audio_out[0]; // declare Output pointer


  HAL_NVIC_EnableIRQ(EXTI0_IRQn);


  //start i2s with 2048 samples transmission => 4096*u16 words
  HAL_I2SEx_TransmitReceive_DMA (&hi2s2, txBuf, rxBuf, BLOCK_SIZE_U16);

  int offset_r_ptr;
  int offset_w_ptr, w_ptr;


  //Test of IQ from si5351O, move later on to VFO section
  si5351_CalcIQ(vfo_a, &pll_conf, &out_conf);

  uint8_t phaseOffset = (uint8_t)out_conf.div;
  si5351_SetupOutput(0, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &out_conf, 0);
  si5351_SetupOutput(1, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &out_conf, phaseOffset);

  si5351_SetupPLL(SI5351_PLL_A, &pll_conf);
  si5351_EnableOutputs((1<<0) | (1<<1));

  VFO_set_frequency(vfo_a);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //VFO_set_frequency(vfo_a); //This generates direct IQ signals for 1:4 multilex

	  //encoderValue = __HAL_TIM_GET_COUNTER(&htim4);
	  encoderValue = read_enc();
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	  if (callback_state != 0) {

		  //decide if it was half or cplt callback
		  if (callback_state == 1)   {
			  	  offset_r_ptr = 0;
			  	  offset_w_ptr = 0;
			  	  w_ptr = 0;
			  }

		  else if (callback_state == 2) {
			  offset_r_ptr = BLOCK_SIZE_U16;
			  offset_w_ptr = BLOCK_SIZE_FLOAT;
			  w_ptr = BLOCK_SIZE_FLOAT;
		  }


		  //restore input sample buffer to float array
		  for (int i=offset_r_ptr; i<offset_r_ptr+BLOCK_SIZE_U16; i=i+4) {
			  l_buf_in[w_ptr] = (float) ((int) (rxBuf[i]<<16)|rxBuf[i+1]);
			  r_buf_in[w_ptr] = (float) ((int) (rxBuf[i+2]<<16)|rxBuf[i+3]);
			  w_ptr++;
		  }

		if(tx_flag == 0){
		  //RX path
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  //process FIR +/-45 degree Hilbert
		  //TODO investigate magnitude of these filters. Can act as LPF but below 300 Hz phase != 90.
		  arm_fir_f32 (&firsettings_l, &l_buf_in[offset_w_ptr], &l_buf_out[offset_w_ptr], BLOCK_SIZE_FLOAT);
		  arm_fir_f32 (&firsettings_r, &r_buf_in[offset_w_ptr], &r_buf_out[offset_w_ptr], BLOCK_SIZE_FLOAT);

		  arm_scale_f32(l_buf_out, rx_amp_cal, l_buf_out, 1024);

		  //summation of two signals 90 degree out of phase.

		  if(mode == 1){
			  arm_add_f32(l_buf_out, r_buf_out, sum_buf_rl, 1024);
		  }
		  else{
			  arm_sub_f32(l_buf_out, r_buf_out, sum_buf_rl, 1024);
		  }

		  //Check for overflow. TODO Potentially scale data before?, TODO check if this is necessary (arm_add might include saturation?)

		  for (uint32_t i = 0; i < BLOCK_SIZE_FLOAT; i++) {
		          if (sum_buf_rl[i] > FLOAT_MAX) {
		        	  sum_buf_rl[i] = FLOAT_MAX; // Apply saturation to prevent overflow
		          } else if (sum_buf_rl[i] < FLOAT_MIN) {
		        	  sum_buf_rl[i] = FLOAT_MIN; // Apply saturation to prevent overflow
		          }
		      }

		  arm_scale_f32(sum_buf_rl, rx_volume, sum_buf_rl, 1024);


		  //SSB audio output filter. Removes below 300 Hz to get rid off the Hilbert imperfection,
		  arm_fir_f32 (&firsettings_rl_sum, &sum_buf_rl[offset_w_ptr], &audio_out[offset_w_ptr], BLOCK_SIZE_FLOAT);

		  //IIR output filter
		  /*
		    for (uint32_t k = 0; k < NUM_BLOCKS; k++)
		    {
		    	arm_biquad_cascade_df2T_f32 (&S, InputValuesf32_ptr + (k*BLOCK_SIZE_FLOAT), OutputValuesf32_ptr + (k*BLOCK_SIZE_FLOAT), BLOCK_SIZE_FLOAT);
		    }
		    */

		  }
		  else{
			  //TX path
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			  //Bandpass the audio
			  arm_fir_f32 (&firsettings_rl_sum, &l_buf_in[offset_w_ptr], &sum_buf_rl[offset_w_ptr], BLOCK_SIZE_FLOAT);

			  //Sets the output power, this is in voltage -3 dB is 0.701, -6 dB is 0.5 etc... this is confirmed to be working
			  arm_scale_f32(sum_buf_rl, tx_power, sum_buf_rl, BLOCK_SIZE_FLOAT*4);

			  //TODO add compressor here

			  //Create IQ
			  if(mode == 0){
				  arm_fir_f32 (&firsettings_l, &sum_buf_rl[offset_w_ptr], &audio_out[offset_w_ptr], BLOCK_SIZE_FLOAT);
			  	  arm_fir_f32 (&firsettings_r, &sum_buf_rl[offset_w_ptr], &l_buf_out[offset_w_ptr], BLOCK_SIZE_FLOAT);

			  	  arm_scale_f32(l_buf_out, tx_amp_cal, l_buf_out, BLOCK_SIZE_FLOAT*4);
			  	 // arm_scale_f32(audio_out, 1, audio_out, BLOCK_SIZE_FLOAT*4);
			  }
			  else if (mode == 1){
				  arm_fir_f32 (&firsettings_r, &sum_buf_rl[offset_w_ptr], &audio_out[offset_w_ptr], BLOCK_SIZE_FLOAT);
			  	  arm_fir_f32 (&firsettings_l, &sum_buf_rl[offset_w_ptr], &l_buf_out[offset_w_ptr], BLOCK_SIZE_FLOAT);

			  	  arm_scale_f32(l_buf_out, tx_amp_cal, l_buf_out, BLOCK_SIZE_FLOAT*4);
			  }

		  }


//TODO change the txBuf to audio output
		  //restore processed float-array to output sample-buffer
		  w_ptr = offset_w_ptr;

		  for (int i=offset_r_ptr; i<offset_r_ptr+BLOCK_SIZE_U16; i=i+4) {
				txBuf[i] =  (((int)l_buf_out[w_ptr])>>16)&0xFFFF;
				txBuf[i+1] = ((int)l_buf_out[w_ptr])&0xFFFF;
				txBuf[i+2] = (((int)audio_out[w_ptr])>>16)&0xFFFF;
				txBuf[i+3] = ((int)audio_out[w_ptr])&0xFFFF;
				w_ptr++;
		  }

		  callback_state = 0;

	  }
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_mode_Pin */
  GPIO_InitStruct.Pin = Button_mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_mode_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t read_enc(void){
	//Changes frequency of LO based on vfo knob
	now_debounce = HAL_GetTick();
	if(now_debounce - last_debounce >= 300){
		last_debounce = now_debounce;

	encoderValue = TIM4->CNT;

	if(encoderValue > tmp_encValue){
		//Encoder moved left
        if (current_vfo == 0) {
        	vfo_a += step;
        	VFO_set_frequency(vfo_a);
            //si5351_SetupCLK0(vfo_a, SI5351_DRIVE_STRENGTH_4MA);		//TODO add 4* after debugging
        } else {
        	vfo_b += step;
        	VFO_set_frequency(vfo_b);
            //si5351_SetupCLK0(vfo_b, SI5351_DRIVE_STRENGTH_4MA);
        }
	}
	if(encoderValue < tmp_encValue){
		//Encoder moved right
        if (current_vfo == 0) {
        	vfo_a -= step;
        	VFO_set_frequency(vfo_a);
            //si5351_SetupCLK0(vfo_a, SI5351_DRIVE_STRENGTH_4MA);
        } else {
        	vfo_a -= step;
        	VFO_set_frequency(vfo_b);
            //si5351_SetupCLK0(vfo_b, SI5351_DRIVE_STRENGTH_4MA);
        }

	}

	encoderValue = 32768;
	tmp_encValue = 32768;
	TIM4->CNT = 32768;
	}
}


void VFO_set_frequency(uint32_t rx_frequency){ //Generates local IQ as the VFO for the dectector and encoder
	si5351_CalcIQ(rx_frequency, &pll_conf, &out_conf);

	uint8_t phaseOffset = (uint8_t)out_conf.div;
	si5351_SetupOutput(0, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &out_conf, 0);
	si5351_SetupOutput(1, SI5351_PLL_A, SI5351_DRIVE_STRENGTH_4MA, &out_conf, phaseOffset);

	si5351_SetupPLL(SI5351_PLL_A, &pll_conf);
	si5351_EnableOutputs((1<<0) | (1<<1));
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){

	callback_state = 1;

}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){

	callback_state = 2;

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == Button_mode_Pin) {
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
	  mode = (mode == 0) ? 1 : 0;
	  //HAL_Delay(50);
  } else {
      __NOP();
  }
}

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

#ifdef  USE_FULL_ASSERT
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
