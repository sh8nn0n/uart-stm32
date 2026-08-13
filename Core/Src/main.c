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
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_PIN GPIO_PIN_5
#define MOTOR_PORT GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t rx_byte;
volatile uint8_t byte_ready = 0;
uint8_t rx_buffer[32] = {0};
uint8_t rx_index = 0;
uint8_t received_LF = 0;
uint8_t ready = 1;
volatile uint32_t motor_count;
uint16_t remember = 0;
uint8_t msg_length = 0;
uint16_t target_rpm = 0;
uint16_t Kp = 0;
uint16_t Ki = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void process_message(void);
uint8_t hex_pair_to_byte(uint8_t high, uint8_t low);
uint8_t hex_val_to_char(uint8_t c);
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
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (ready)
	  {
		//calculate pwm speed
		ready = 0;

		process_message();

		uint8_t newline[] = "\r\n";
		HAL_UART_Transmit(&huart2, newline, 2, 1000);

		//find motor speed
		static uint32_t last_check = 0;
		uint32_t now = HAL_GetTick();
		uint32_t elapsed = now-last_check;

		if (elapsed)
		{
			uint32_t count = motor_count;
			motor_count = 0;
			last_check = now;

			uint32_t rpm = ((count*60000)/elapsed) / 6;

			char msg[32];
			int len = snprintf(msg, sizeof(msg), "RPM: %lu \r\n",
								(unsigned long)rpm);

			//find proportion term
			uint32_t actual_rpm = rpm;
			static uint16_t duty = 0;
			int32_t error = (int32_t)target_rpm - (int32_t)actual_rpm;
			int32_t P_term = Kp*error;

			//find integral term
			static int32_t integral = 0;
			integral = integral + (error*(int32_t)elapsed);
			int32_t I_term = Ki*integral;

	        //clamp input to 0-699
			int32_t new_duty = duty + P_term + I_term;
			if (new_duty > 699) new_duty = 699;
			if (new_duty < 0) new_duty = 0;
			duty = (uint16_t)new_duty;		//turn everything back to unsigned

		   static uint8_t pwm_started = 0;
		   if (!pwm_started) { HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); pwm_started = 1; }
		   __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, duty);
			HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 1000);
		}
	  }


  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  htim16.Init.Prescaler = 159;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 699;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){
	if (huart -> Instance == USART2)
	{
		uint8_t byte = rx_byte;		//copy the newly stored rx_byte into a local variable to store

		//HAL_UART_Transmit(&huart2, &rx_byte, 1, 1000);		//echo the transmit
		if (byte == ':')
		{
			rx_index = 0;
		    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
			return;
		}
		byte_ready = 1;			//sets if statement to true
		if (byte_ready)
		{
			if (received_LF)	//check if \n is received
			{
			  received_LF = 0;
			  if (byte == '\n')
				{
				  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);		//swallowed the \n which means there's no more data to store
				  return;			//ignores the rest of the callback and go back to while loop
				}
			}
			if (byte == '\r')		//if \r is detected
			{
				rx_buffer[rx_index] = '\0';		//stores the null value to mark the end of the string
				msg_length = rx_index;
				rx_index = 0;					//reset the index
				received_LF = 1;				//Received one of the LF, so look out for more
				ready = 1;						//the ready state is set to true to run the while loop
			}
			else if (rx_index < sizeof(rx_buffer) - 1)	//keep storing the bytes
			{
				rx_buffer[rx_index++] = byte;   //accumulates what you're typing
			}
		}
	}
		HAL_UART_Receive_IT(&huart2, &rx_byte, 1);		//rearms UART for next byte
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == MOTOR_PIN)
  {
	  motor_count++;			//accumulates motor speed count based on rpm
  }

}


//definitions for hexadecimal
uint8_t hex_char_to_val(uint8_t c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

uint8_t hex_pair_to_byte(uint8_t high, uint8_t low)
{
    return (hex_char_to_val(high) << 4) | hex_char_to_val(low);
}

uint8_t hex_val_to_char(uint8_t c)
{
	if (c >= 0 && c <= 9) return c + '0';
	if (c >= 10 && c <= 15) return c + 'A' - 10;
    return 0;
}

static void process_message(void)
{
    if (msg_length < 12) return;		//needs the register to be complete, or else the function won't run

    uint8_t address  = hex_pair_to_byte(rx_buffer[0], rx_buffer[1]);
    uint8_t function = hex_pair_to_byte(rx_buffer[2], rx_buffer[3]);

    uint8_t LRC = 0;

    if (address == 0x01 && function == 0x06)
    {


        uint16_t reg_address = (hex_pair_to_byte(rx_buffer[4], rx_buffer[5]) << 8)
                        |  hex_pair_to_byte(rx_buffer[6], rx_buffer[7]);

        uint16_t value = (hex_pair_to_byte(rx_buffer[8], rx_buffer[9]) << 8)
                                |  hex_pair_to_byte(rx_buffer[10], rx_buffer[11]);

        if (reg_address == 0x0000)
        {
        	target_rpm = value;
        }
        if (reg_address == 0x0001)
        {
        	Kp = value;
        }
        if (reg_address == 0x0002)
        {
        	Ki = value;
        }

        //Buffer for outgoing reply
		uint8_t reply[32];
		//Position will be incrementally updated after each element is written into the buffer
		uint8_t pos = 0;
		uint8_t LRC = 0;

        reply[pos++] = ':';

		//Write position of address to master so master knows it's the right one that it queried
		uint8_t high_address = (address >> 4) & 0x0F;   // top 4 bits of address
		uint8_t low_address  = address & 0x0F;          // bottom 4 bits of address
		LRC = LRC + address;
		reply[pos++] = hex_val_to_char(high_address);
		reply[pos++] = hex_val_to_char(low_address);

		uint8_t high_function = (function >> 4) & 0x0F;
		uint8_t low_function = function & 0x0F;
		LRC = LRC + function;
		reply[pos++] = hex_val_to_char(high_function);
		reply[pos++] = hex_val_to_char(low_function);

		//Write position of register address (2 bytes, 16 bits)
		uint16_t high_reg = (reg_address >> 8) & 0x0FF;		//split data into 8 and 8
		uint16_t high_reg_hi = (high_reg >> 4) & 0x0F;	//split high bit in half again
		uint16_t high_reg_lo = high_reg & 0x0F;
		uint16_t low_reg = reg_address & 0x0FF;
		uint16_t low_reg_hi = (low_reg >> 4) & 0x0F;
		uint16_t low_reg_lo = low_reg & 0x0F;
		LRC = LRC + high_reg + low_reg;
		reply[pos++] = hex_val_to_char(high_reg_hi);
		reply[pos++] = hex_val_to_char(high_reg_lo);
		reply[pos++] = hex_val_to_char(low_reg_hi);
		reply[pos++] = hex_val_to_char(low_reg_lo);

		//Write position of value (2 bytes, 16 bits)
		uint16_t high_val = (value >> 8) & 0x0FF;		//split data into 8 and 8
		uint16_t high_val_hi = (high_val >> 4) & 0x0F;	//split high bit in half again
		uint16_t high_val_lo = high_val & 0x0F;
		uint16_t low_val = value & 0x0FF;
		uint16_t low_val_hi = (low_val >> 4) & 0x0F;
		uint16_t low_val_lo = low_val & 0x0F;
		LRC = LRC + high_val + low_val;
		reply[pos++] = hex_val_to_char(high_val_hi);
		reply[pos++] = hex_val_to_char(high_val_lo);
		reply[pos++] = hex_val_to_char(low_val_hi);
		reply[pos++] = hex_val_to_char(low_val_lo);

		//Handle LRC (2's complement)
		uint8_t lrc_comp = (~LRC) + 1;
		uint8_t high_lrc_comp = (lrc_comp >> 4) & 0x0F;
		uint8_t low_lrc_comp = lrc_comp & 0x0F;
		reply[pos++] = hex_val_to_char(high_lrc_comp);
		reply[pos++] = hex_val_to_char(low_lrc_comp);

		//Finish with \r\n
		reply[pos++] = '\r';
		reply[pos++] = '\n';

		//Transmit it back to console
		HAL_UART_Transmit(&huart2,reply, pos, 1000);

    }

    if (address == 0x01 && function == 0x03)
    {
        /*uint16_t reg_address = (hex_pair_to_byte(rx_buffer[4], rx_buffer[5]) << 8)
                        |  hex_pair_to_byte(rx_buffer[6], rx_buffer[7]);

        uint16_t quantity_field= (hex_pair_to_byte(rx_buffer[8], rx_buffer[9]) << 8)
                        |  hex_pair_to_byte(rx_buffer[10], rx_buffer[11]); */

        //Buffer for outgoing reply
        uint8_t reply[32];
        //Position will be incrementally updated after each element is written into the buffer
        uint8_t pos = 0;

        reply[pos++] = ':';

        //Write position of address to master so master knows it's the right one that it queried
        uint8_t high_address = (address >> 4) & 0x0F;   // top 4 bits of address
        uint8_t low_address  = address & 0x0F;          // bottom 4 bits of address
        LRC = LRC + address;
        reply[pos++] = hex_val_to_char(high_address);
        reply[pos++] = hex_val_to_char(low_address);

        uint8_t high_function = (function >> 4) & 0x0F;
        uint8_t low_function = function & 0x0F;
        LRC = LRC + function;
        reply[pos++] = hex_val_to_char(high_function);
        reply[pos++] = hex_val_to_char(low_function);

        //Tells master how many data bytes will follow so it knows when to stop reading for data
        uint8_t byte_count = 2;
        LRC = LRC + byte_count;
        uint8_t high_bc = (byte_count >> 4) & 0x0F;
        uint8_t low_bc  = byte_count & 0x0F;
        reply[pos++] = hex_val_to_char(high_bc);
        reply[pos++] = hex_val_to_char(low_bc);

        //Write position of data (2 bytes, 16 bits)
        uint16_t high_remember = (remember >> 8) & 0x0FF;		//split data into 8 and 8
        uint16_t high_remember_hi = (high_remember >> 4) & 0x0F;	//split high bit in half again
        uint16_t high_remember_lo = high_remember & 0x0F;
        uint16_t low_remember = remember & 0x0FF;
        uint16_t low_remember_hi = (low_remember >> 4) & 0x0F;
        uint16_t low_remember_lo = low_remember & 0x0F;
        LRC = LRC + high_remember + low_remember;
        reply[pos++] = hex_val_to_char(high_remember_hi);
        reply[pos++] = hex_val_to_char(high_remember_lo);
        reply[pos++] = hex_val_to_char(low_remember_hi);
        reply[pos++] = hex_val_to_char(low_remember_lo);

        uint8_t lrc_comp = (~LRC) + 1;
        uint8_t high_lrc_comp = (lrc_comp >> 4) & 0x0F;
        uint8_t low_lrc_comp = lrc_comp & 0x0F;
        reply[pos++] = hex_val_to_char(high_lrc_comp);
        reply[pos++] = hex_val_to_char(low_lrc_comp);

        //Finish with \r\n
        reply[pos++] = '\r';
        reply[pos++] = '\n';

        //Transmit it back to console
        HAL_UART_Transmit(&huart2,reply, pos, 1000);

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
