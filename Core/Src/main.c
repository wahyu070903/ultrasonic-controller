/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PORT   GPIOA
#define TRIG_PIN1   GPIO_PIN_11
#define TRIG_PIN2   GPIO_PIN_10
#define TRIG_PIN3   GPIO_PIN_9
#define TRIG_PIN4   GPIO_PIN_8

#define SEL_PORT    GPIOB
#define SEL1_PIN    GPIO_PIN_5
#define SEL2_PIN    GPIO_PIN_4
#define SEL3_PIN    GPIO_PIN_3

#define ECHO_PORT   GPIOA
#define ECHO_PIN    GPIO_PIN_0

#define BUZZ_PORT   GPIOA
#define BUZZ_PIN    GPIO_PIN_6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BURST_CYCLE            8U          // 8 siklus per burst
#define BLANKING_US            1000UL      // dead time (us)
#define LISTEN_US              3000UL     // window tunggu echo (us) 30000
#define CYCLE_MS               0UL       // interval antar sequence (ms) 60
#define SPEED_OF_SOUND_CM_US   0.0343f
#define DIST_OFFSET            -5.0f

#define FILTER_ALPHA        0.3f    // EMA weight makin kecil makin lambat respon
#define MAX_JUMP_CM         15.0f
#define OUTLIER_ALPHA       0.1f
#define NO_ECHO_CONFIRM     3
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
typedef enum
{
    STATE_IDLE,
    STATE_BURST,
    STATE_BLANKING,
    STATE_LISTEN
} SystemState;

volatile SystemState state = STATE_IDLE;

volatile uint8_t  active_channel   = 0;
volatile uint8_t  halfCycleCount   = 0;
volatile uint8_t  burstActive      = 0;

volatile uint32_t burstStartTime    = 0;
volatile uint32_t blankingStartTime = 0;
volatile uint32_t listenStartTime   = 0;
volatile uint32_t lastSequenceTime  = 0;

volatile uint32_t echoTime            = 0;
volatile uint8_t  echo_detected       = 0;
volatile uint32_t echo_burstStartTime = 0;
volatile uint32_t spacious_counter    = 0;

static volatile uint32_t last_cyc_for_micros = 0;
static volatile uint32_t us_accum           = 0;
static volatile uint32_t rem_cyc            = 0;

float distance_set[4] = {
		999,
		999,
		999,
		999
};

static float   filtered_distance[4]   = {999, 999, 999, 999};
static uint8_t filter_initialized[4]  = {0, 0, 0, 0};
static uint8_t no_echo_count[4]       = {0, 0, 0, 0};

const float distance_close = 24.0;
const float distance_medium = 30.0;
const float distance_safe = 50.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static inline uint32_t micros(void);
static inline void delay_us(uint32_t us);

static void uart_print(const char *str);
static void float_to_str(float val, char *buf, uint8_t decimals);

void allTriggerIdle(void);
void selectChan(uint8_t channel);
void sendBurst(uint8_t channel);
void processEcho(void);
void buzzerNotify();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t micros(void)
{
	uint32_t cyc_per_us = SystemCoreClock / 1000000U;

	uint32_t cyc      = DWT->CYCCNT;
	uint32_t cyc_diff = cyc - last_cyc_for_micros;
	last_cyc_for_micros = cyc;

	uint32_t total = cyc_diff + rem_cyc;
	us_accum += total / cyc_per_us;
	rem_cyc   = total % cyc_per_us;

	return us_accum;
}

static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}

static void uart_print(const char *str)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

static void float_to_str(float val, char *buf, uint8_t decimals)
{
    int32_t intPart = (int32_t)val;
    float frac = val - (float)intPart;
    if (frac < 0) frac = -frac;

    int32_t mul = 1;
    for (uint8_t i = 0; i < decimals; i++) mul *= 10;

    int32_t fracPart = (int32_t)(frac * mul + 0.5f);

    char fmt[16];
    sprintf(fmt, "%%ld.%%0%dld", decimals);
    sprintf(buf, fmt, (long)intPart, (long)fracPart);
}

void allTriggerIdle(void)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN4, GPIO_PIN_SET);
}

void selectChan(uint8_t channel)
{
    switch (channel)
    {
        case 0:
            HAL_GPIO_WritePin(SEL_PORT, SEL1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SEL_PORT, SEL2_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SEL_PORT, SEL3_PIN, GPIO_PIN_SET);

            break;

        case 1:
            HAL_GPIO_WritePin(SEL_PORT, SEL1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SEL_PORT, SEL2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SEL_PORT, SEL3_PIN, GPIO_PIN_SET);

            break;

        case 2:
            HAL_GPIO_WritePin(SEL_PORT, SEL1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SEL_PORT, SEL2_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SEL_PORT, SEL3_PIN, GPIO_PIN_RESET);
            break;

        case 3:
            HAL_GPIO_WritePin(SEL_PORT, SEL1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(SEL_PORT, SEL2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SEL_PORT, SEL3_PIN, GPIO_PIN_RESET);
            break;

        default:
            break;
    }
}

void sendBurst(uint8_t channel)
{
    allTriggerIdle();

    active_channel  = channel;
    halfCycleCount  = 0;
    burstActive     = 1;

    switch (active_channel)
    {
        case 0: HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN1, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN2, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN3, GPIO_PIN_SET); break;
        case 3: HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN4, GPIO_PIN_SET); break;
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0);
    burstStartTime = micros();
    state = STATE_BURST;

    HAL_TIM_Base_Start_IT(&htim1);
}

void processEcho(void)
{
    char buf[96];
    char num[24];
    uint8_t ch = active_channel;

    if (echo_detected)
    {
        uint32_t tof_us = echoTime - echo_burstStartTime;
        float distance_cm = (tof_us * SPEED_OF_SOUND_CM_US) / 2.0f - DIST_OFFSET;

        no_echo_count[ch] = 0;

        if (!filter_initialized[ch])
        {
            filtered_distance[ch] = distance_cm;
            filter_initialized[ch] = 1;
        }
        else
        {
            float delta = distance_cm - filtered_distance[ch];
            if (delta < 0) delta = -delta;

            if (delta > MAX_JUMP_CM)
            {
                filtered_distance[ch] += (distance_cm - filtered_distance[ch]) * OUTLIER_ALPHA;
            }
            else
            {
                filtered_distance[ch] += (distance_cm - filtered_distance[ch]) * FILTER_ALPHA;
            }
        }

        distance_set[ch] = filtered_distance[ch];

        float_to_str(filtered_distance[ch], num, 2);
        sprintf(buf, "| CH%d %s cm", ch + 1, num);
        uart_print(buf);
    }
    else
    {
        no_echo_count[ch]++;

        if (no_echo_count[ch] >= NO_ECHO_CONFIRM)
        {
            distance_set[ch]        = 999;
            filtered_distance[ch]   = 999;
            filter_initialized[ch]  = 0;

            sprintf(buf, "| CH%d No Echo", ch + 1);
        }
        else
        {
            distance_set[ch] = filtered_distance[ch];

            sprintf(buf, "| CH%d No Echo (hold)", ch + 1);
        }

        uart_print(buf);
    }
}

void buzzerNotify(){
	float closest_point = 999;
	uint8_t closest_index = 99;
	for(uint8_t i = 0 ; i < 4 ; i++){
		if(distance_set[i] < closest_point){
			closest_point = distance_set[i];
			closest_index = i;
		}
	}

	if(closest_index != 99){
		if(closest_point < distance_close){
			static uint32_t last_toggle = 0;
			if (HAL_GetTick() - last_toggle >= 50){
				last_toggle = HAL_GetTick();
				HAL_GPIO_TogglePin(BUZZ_PORT, BUZZ_PIN);
			}
		}else if(closest_point > distance_close && closest_point < distance_medium){
			static uint32_t last_toggle = 0;
			if (HAL_GetTick() - last_toggle >= 150){
				last_toggle = HAL_GetTick();
				HAL_GPIO_TogglePin(BUZZ_PORT, BUZZ_PIN);
			}
		}else{
			HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
		}
	}else{
		HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        switch (active_channel)
        {
            case 0: HAL_GPIO_TogglePin(TRIG_PORT, TRIG_PIN1); break;
            case 1: HAL_GPIO_TogglePin(TRIG_PORT, TRIG_PIN2); break;
            case 2: HAL_GPIO_TogglePin(TRIG_PORT, TRIG_PIN3); break;
            case 3: HAL_GPIO_TogglePin(TRIG_PORT, TRIG_PIN4); break;
        }

        halfCycleCount++;
        if (halfCycleCount >= BURST_CYCLE * 2)
        {
            allTriggerIdle();

            HAL_TIM_Base_Stop_IT(&htim1);
            burstActive = 0;

            state = STATE_BLANKING;
            blankingStartTime = micros();
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ECHO_PIN)
    {
        if (state == STATE_LISTEN && !echo_detected)
        {
			echoTime = micros();
			echo_burstStartTime = burstStartTime;
			echo_detected = 1;
			spacious_counter++;
        }
    }
}
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
  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  allTriggerIdle();
  selectChan(0);

  HAL_TIM_Base_Stop_IT(&htim1);

  state = STATE_IDLE;
  lastSequenceTime = micros();

  uart_print("Ultrasonic system ready\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t now = micros();

	  if (state == STATE_IDLE)
	  {
		  if ((int32_t)(now - lastSequenceTime) >= (int32_t)(CYCLE_MS * 1000UL))
		  {
			  echo_detected = 0;
			  selectChan(active_channel);
			  sendBurst(active_channel);
		  }
	  }

	  else if (state == STATE_BLANKING)
	  {
		  if ((int32_t)(now - blankingStartTime) >= (int32_t)BLANKING_US)
		  {
			  echo_detected = 0;
			  listenStartTime = micros();
			  state = STATE_LISTEN;
		  }
	  }

	  else if (state == STATE_LISTEN)
	  {
		  if (echo_detected)
		  {
			  processEcho();

			  active_channel++;
			  if (active_channel >= 4)
			  {
				  active_channel = 0;
				  lastSequenceTime = micros();
				  uart_print("\r\n");
			  }
			  state = STATE_IDLE;
		  }

		  else if ((int32_t)(now - listenStartTime) >= (int32_t)LISTEN_US)
		  {
			  processEcho();

			  active_channel++;
			  if (active_channel >= 4)
			  {
				  active_channel = 0;
				  lastSequenceTime = micros();
				  uart_print("\r\n");
			  }
			  state = STATE_IDLE;
		  }
	  }
	  buzzerNotify();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 899;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG1_Pin|TRIG2_Pin|TRIG3_Pin|TRIG4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEL1_Pin|SEL2_Pin|SEL3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZ_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG1_Pin TRIG2_Pin TRIG3_Pin TRIG4_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin|TRIG2_Pin|TRIG3_Pin|TRIG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL1_Pin SEL2_Pin SEL3_Pin */
  GPIO_InitStruct.Pin = SEL1_Pin|SEL2_Pin|SEL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
