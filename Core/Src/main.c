/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Data is loaded onto the microcontroller in "frames" (chunks),
// effectively making this a ring buffer with NFRAME items of length
// FRAME_SAMPLES.
// Note the STM32F030F4 has only 4K SRAM so this buffer must stay small
#define NFRAME 4
#define FRAME_SAMPLES 16*2
uint8_t wav_buffer[NFRAME][FRAME_SAMPLES];
volatile uint8_t frame_idx;  // Where to read sample data
volatile uint8_t frame_buf_idx; // Where to write the next incoming frame
volatile uint16_t sample_idx; // Which sample in frame_idx to read next
// PCM treats 0x00 as -amp_max and 0xff as amp_max.
#define WAV_ZERO 0x80

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == htim16.Instance) {
    	// Toggle Motor A direction
    	HAL_GPIO_TogglePin(GPIOA, AIN1_Pin|AIN2_Pin);
  } else if (htim->Instance == htim17.Instance) {
    	// Toggle A3/A4, Motor B direction
    	HAL_GPIO_TogglePin(GPIOA, BIN1_Pin|BIN2_Pin);
  } else if (htim->Instance == htim14.Instance) {
	  // htim14 handles wav data frame advancement

      if (frame_idx == frame_buf_idx) {
        return; // We're caught up; no more data to read.
      }

      // Load next WAV PCM sample
      uint8_t vA = wav_buffer[frame_idx][sample_idx++];
      uint8_t vB = wav_buffer[frame_idx][sample_idx++];

      // Advance to the next frame if we've read the current one,
      // Wrapping around when we reach the end.
      if (sample_idx >= FRAME_SAMPLES) {
        sample_idx = 0;
        frame_idx = (frame_idx + 1) % NFRAME;
      }
      
      // Set direction based on whether we're in the top or lower half of the waveform
      HAL_GPIO_WritePin(GPIOA, AIN1_Pin, (vA >= WAV_ZERO) ? GPIO_PIN_SET   : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, AIN2_Pin, (vA >= WAV_ZERO) ? GPIO_PIN_RESET : GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, BIN1_Pin, (vB >= WAV_ZERO) ? GPIO_PIN_SET   : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, BIN2_Pin, (vB >= WAV_ZERO) ? GPIO_PIN_RESET : GPIO_PIN_SET);

      // We shift and remove the sign bit to get the absolute amplitude
      uint8_t vccrA = ((uint8_t)(vA - 128)) & (~0x80);
      htim3.Instance->CCR1 = vccrA;
      uint8_t vccrB = ((uint8_t)(vB - 128)) & (~0x80);
      htim3.Instance->CCR2 = vccrB;
  } else {
    // "This should never happen"
    Error_Handler();
  }
}

#define CMD_PACKET_SZ 6
#define F_CLK 48000000
#define PSC 4
#define VLIM 256

uint16_t last_freqs[2] = {0,0};
uint8_t last_vols[2] = {0,0};
void setFreq(uint8_t channel, uint16_t freq, uint8_t vol) {
	// Frequency mode uses tim16 (motor 1) and tim17 (motor 2) to
	// trigger interrupts which are used to toggle the GPIO pins specifying direction.
	// Volume is controlled by setting the duty cycle of tim3.
	// Registers are only set if the frequency has changed - this prevents
	// blips/noise when setting the same frequency

	// 2*freq as we're triggering an interrupt every half-period
	freq *= 2;

	volatile uint32_t* volccr;
	TIM_HandleTypeDef* tim;
	uint16_t p1, p2;
	if (channel == 0) {
		volccr = &(htim3.Instance->CCR1);
		tim = &htim16;
		p1 = AIN1_Pin;
		p2 = AIN2_Pin;
	} else {
		volccr = &(htim3.Instance->CCR2);
		tim = &htim17;
		p1 = BIN1_Pin;
		p2 = BIN2_Pin;
	}

	HAL_TIM_Base_Stop_IT(tim);
	if (last_vols[channel] != vol) {
		*volccr = (vol * VLIM) / 256;
	}
	if (freq == 0) {
	  HAL_GPIO_WritePin(GPIOA, p1|p2, GPIO_PIN_RESET);
	} else if (last_freqs[channel] != freq) {
		// Freq = Fclk / ((ARR+1)*(PSC+1))
		// (Fclk / (Freq * (PSC+1))) - 1 = ARR
		uint16_t arr =  F_CLK / (freq * (PSC+1)) - 1;
		tim->Instance->ARR = arr;
		tim->Instance->CCR1 = arr / 2; // 50% duty cycle
		HAL_GPIO_WritePin(GPIOA, p1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, p2, GPIO_PIN_SET);
	}

	if (vol != 0 && freq != 0) {
	  HAL_TIM_Base_Start_IT(tim);
	}

	last_vols[channel] = vol;
	last_freqs[channel] = freq;
}

void startWavMode() {
  // Wav mode uses tim14 to advance the data buffer and set the next PCM
  // sample. 

  // Disable freq mode timers so they don't vibrate the brush
  setFreq(0, 0, 0);
  setFreq(1, 0, 0);  

  HAL_TIM_Base_Stop_IT(&htim14);
  frame_idx = 0;
  sample_idx = 0;
  frame_buf_idx = 0;
  HAL_TIM_Base_Start_IT(&htim14);
}

void startFreqMode() {
  HAL_TIM_Base_Stop_IT(&htim14);
  setFreq(0, 0, 0);
  setFreq(1, 0, 0);
}

void demoWav() {
  startWavMode(); // Start consuming data
  while (1) {
    // Keep the buffer filled
    if ((frame_buf_idx+1) % NFRAME != frame_idx) {
      for (uint16_t i = 0; i < FRAME_SAMPLES; i+= 2) { // Skip every other byte; only fill channel 1
          wav_buffer[frame_buf_idx][i] = i & 0xff; // Simple sawtooth, 8 bits @ 44.1kHz ~= 172 Hz tone
      }
      frame_buf_idx = (frame_buf_idx+1) % NFRAME;
    } 
  }
}

void demoFreq() {
  while (1) {
	setFreq(0, 440, 127);
	setFreq(1, 220, 127);
	HAL_Delay(500);
	setFreq(0, 220, 127);
	setFreq(1, 440, 127);
	HAL_Delay(500);
	setFreq(0, 220, 127);
	setFreq(1, 880, 127);
	HAL_Delay(500);
	setFreq(0, 440, 255);
	setFreq(1, 220, 255);
	HAL_Delay(500);
	setFreq(0, 220, 255);
	setFreq(1, 440, 255);
	HAL_Delay(500);
	setFreq(0, 220, 255);
	setFreq(1, 880, 255);
	HAL_Delay(500);
	setFreq(0, 0, 0);
	setFreq(1, 0, 0);
	HAL_Delay(500);
  }
}

uint8_t freq_buffer[CMD_PACKET_SZ];
enum  Mode {
  MODE_UNKNOWN,
  MODE_FREQ,
  MODE_WAV,
} mode = MODE_UNKNOWN;

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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  startFreqMode();
  mode = MODE_FREQ; // Default to frequency mode

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	switch (mode) {
      case MODE_FREQ:
        if (HAL_I2C_Slave_Receive(&hi2c1, freq_buffer, CMD_PACKET_SZ, HAL_MAX_DELAY) == HAL_OK) {
          uint16_t fA = freq_buffer[0] + (freq_buffer[1] << 8);
          uint16_t fB = freq_buffer[2] + (freq_buffer[3] << 8);
          uint8_t vA = freq_buffer[4];
          uint8_t vB = freq_buffer[5];

          // TODO handle switching to and from WAV mode once WAV mode is useful
//          if (fA == 0 && fB == 0 && vA == 255 && vB == 255) {
//            // Interpret screaming at 0Hz as "I want to use wav mode"
//            mode = MODE_WAV;
//            startWavMode();
//          } else {
//            setFreq(0, fA, vA);
//            setFreq(1, fB, vB);
//          }
          setFreq(0, fA, vA);
          setFreq(1, fB, vB);
        }
        break;  
      case MODE_WAV:
        if (HAL_I2C_Slave_Receive(&hi2c1, wav_buffer[frame_buf_idx], FRAME_SAMPLES, HAL_MAX_DELAY) == HAL_OK) {
          if (strncmp((char*)(wav_buffer[frame_buf_idx]), "\ff\00\ff\00FREQ", 8) == 0) {
            mode = MODE_FREQ;
            startFreqMode();
          } else {
            frame_buf_idx = (frame_buf_idx + 1) % NFRAME;
          }
        }
        break;
      case MODE_UNKNOWN:
      default:
        break;
    }
//	  if (HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t *)"OK", 2, HAL_MAX_DELAY) != HAL_OK) {
//		  continue;
//		  //Error_Handler();
//	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 126;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 256;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 256;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  // tim3 should always be running, as it's the carrier upon which volume is expressed
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);


  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */
  // 48000000 / 44100 = 1088.43537
  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1088;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 4;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim16, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 4;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim17, TIM_CHANNEL_1);
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */
  HAL_TIM_Base_Start_IT(&htim17);

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|BIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN1_Pin|BIN2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : AD0_Pin AD1_Pin */
  GPIO_InitStruct.Pin = AD0_Pin|AD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S_STAY_Pin */
  GPIO_InitStruct.Pin = S_STAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(S_STAY_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
