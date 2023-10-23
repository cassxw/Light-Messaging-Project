/**
******************************************************************************
* @file           : Transmitter Code for EEE3095S
* @brief          : Light-of-Things CS Project
******************************************************************************
* @attention
* Group Members:
* -> Tumi Mokoka (MKKBOI005)
* -> Matome Mbowene (MBWMAT002)
* -> Cassandra Wallace (WLLCAS004)
*
* Copyright (c) 2023 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim3;

uint32_t prev_millis = 0;
uint32_t curr_millis = 0;
uint32_t delay_t = 500; // Initialise delay to 500ms

// 16-bit for efficiency, because we know that the maximum ADC value that
// the Transmitter could send to the Receiver is 4095.
uint16_t adc_val;

// Counter for Transmitted Data Messages
uint16_t transmitted_counter = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

void EXTI0_1_IRQHandler(void);
void triggerCheckpoint(void);
void transmitMessage(uint16_t data, uint8_t val);
uint8_t calculateParity(uint16_t data);
uint32_t pollADC(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM3_Init();

  init_LCD();

  // PWM setup
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Start PWM on TIM3 Channel 3

  /* Infinite loop */
  while (1)
  {
	// Listening for Checkpoint Message Transmission
	if (!(GPIOA->IDR & GPIO_IDR_1)) {
		triggerCheckpoint();
	}

	// Read ADC Value from POT1
	adc_val = pollADC();

	// Convert ADC Value to a String for LCD display
	char adc_val_str[20];
	sprintf(adc_val_str, "ADC Value: %d", adc_val);

	// Display ADC Value on the LCD
	lcd_command(CLEAR);
	lcd_putstring(adc_val_str);

	// Wait for delay ms
	HAL_Delay(delay_t);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  ADC1->CR |= ADC_CR_ADCAL;
  while(ADC1->CR & ADC_CR_ADCAL);			// Calibrate the ADC
  ADC1->CR |= (1 << 0);						// Enable ADC
  while((ADC1->ISR & (1 << 0)) == 0);		// Wait for ADC ready
  /* USER CODE END ADC_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 47999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP); //@

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); //@

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief   Interrupt handler for EXTI0_1 (PA0 button press).
 * @note    This function is called every time PA0 is pressed.
 */
void EXTI0_1_IRQHandler(void)
{
	// Getting the current time
	curr_millis = HAL_GetTick();

	// Debouncing to prevent false triggers
	if ((curr_millis - prev_millis) >= 240000) {

		//*****************************************************
		// When user presses PA0, this function initiates the
		// transmission process:
		// 1) Polls the current ADC value
		// 2) Writes the current ADC value to LCD
		// 3) Converts to binary and pulses LED0
		//*****************************************************

		// Read ADC Value from POT1
		adc_val = pollADC();

		// Display the current ADC value on the LCD
		char lcd_message[20];
		sprintf(lcd_message, "ADC Value: %d", adc_val);
		lcd_command(CLEAR);
		lcd_putstring(lcd_message);
		lcd_command(LINE_TWO);
		lcd_putstring("Sending...");

		// Transmit the Data Message
		transmitMessage(adc_val, 0);

		// Indicate successful sending
		lcd_command(LINE_TWO);
		lcd_putstring("Sent to CN!");
		transmitted_counter++;

		// Add a delay for visual feedback
		delay(200000);

		// Update previous time for debouncing
		prev_millis = curr_millis;
	}

	HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
}

/**
 * @brief   Trigger a checkpoint transmission.
 *
 * This function is responsible for transmitting a Checkpoint Message to a destination.
 * It includes the counter value in the message and updates the LCD display accordingly.
 *
 * The function performs the following steps:
 * 1. Construct a message containing the current counter value.
 * 2. Display the message on the first line of the LCD.
 * 3. Display a "Sending..." message on the second line of the LCD.
 * 4. Transmit the checkpoint message to the destination.
 * 5. Display a "Sent to CN!" message on the second line of the LCD.
 * 6. Add a delay for visual feedback.
 */
void triggerCheckpoint(void) {

	// Construct a message with the current counter value
	char checkpoint[20];
	sprintf(checkpoint, "Counter: %d", transmitted_counter);

	// Display the Checkpoint Message on the LCD
	lcd_command(CLEAR);
	lcd_putstring(checkpoint);
	lcd_command(LINE_TWO);
	lcd_putstring("Sending...");

	// Transmit the Checkpoint Message
	transmitMessage(transmitted_counter, 1);

	// Indicate successful sending
	lcd_command(LINE_TWO);
	lcd_putstring("Sent to CN!");

	// Add a delay for visual feedback
	delay(200000);
}

/**
 * @brief   Transmit data over a communication channel, including control bits.
 *
 * This function is responsible for transmitting data, along with control bits,
 * over a communication channel represented by an LED. The data can be a Data Message
 * or a Checkpoint Message, and control bits are represented by LED states.
 *
 * @param   data The 16-bit data to be transmitted.
 * @param   val A control variable to determine the message type:
 *            - If val is 1, the message is a Checkpoint Message (LED on).
 *            - If val is 0, the message is a Data Message (LED off).
 */
void transmitMessage(uint16_t data, uint8_t val) {

    // Start bit (SOT) - LED on for 1 second
    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
    delay(200000);

    // Message Identifier
	if (val == 1) {
		// Checkpoint Message - LED on for 1 second
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
	} else if (val == 0) {
		// Data Message - LED off for 1 second
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
	}
	delay(200000);

    // Send the data in Little Endian binary format
    for (int i = 0; i < 16; i++) {
        // Extract each bit from the data
        uint8_t bit = (data >> i) & 0x01;

        // LED on for 500 ms for '1', LED off for 500 ms for '0'
        HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, (bit == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        delay(100000);
    }

    // Calculate the parity bit
    uint8_t parity = calculateParity(data);

    // LED on or off for 1 second based on parity bit
    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, (parity == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay(200000);

    // Stop (EOT) - Toggling in 100ms intervals for 1 second
    for (int i = 0; i < 10; i++) {
    	HAL_GPIO_TogglePin(LED7_GPIO_Port, LED7_Pin);
    	delay(20000);
    }

    // Ensure Back to Idle State (OFF)
    HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
}

/**
 * @brief   Calculate the parity bit for a 16-bit data value.
 *
 * This function calculates the parity bit for a 16-bit data value by
 * performing an XOR operation on all the individual bits. Parity is used
 * to check for data integrity and to ensure that the number of set bits in
 * the data is even or odd.
 *
 * @param   data The 16-bit data for which to calculate the parity.
 * @return  The calculated parity bit (0 or 1) representing data integrity.
 */
uint8_t calculateParity(uint16_t data) {
    uint8_t parity = 0;
    for (int i = 0; i < 16; i++) {
    	// XOR each bit to calculate the parity
        parity ^= (data >> i) & 0x01;
    }
    return parity;
}

/**
 * @brief   Polls the ADC to get the current ADC value.
 *
 * This function initiates an ADC conversion and blocks until the conversion is
 * completed, obtaining the current ADC value. If the ADC conversion is successful,
 * it returns the ADC value; otherwise, it returns an error value (0xFFFFFFFF).
 *
 * @return  The current ADC value if the conversion is successful,
 * 			or 0xFFFFFFFF in case of an error.
 */
uint32_t pollADC(void){

	// Start ADC Conversion
	HAL_ADC_Start(&hadc);

	uint32_t val = 0;

	// Wait for ADC to finish
	// Using HAL_MAX_DELAY ensures enough blocking until ADC Conversion has finished
	if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) == HAL_OK) {

		// Successful ADC Conversion
		// Read the ADC value
		val = HAL_ADC_GetValue(&hadc);

		// Stop ADC Conversion
		HAL_ADC_Stop(&hadc);

	// If something went wrong in ADC Conversion
	} else {
		val = 0xFFFFFFFF;
	}

	return val;
}

/**
 * @brief   ADC1 and COMP1 (Analog Comparator) interrupt handler.
 *
 * This function is an interrupt handler for ADC1 and COMP1 interrupts.
 * It reads the ADC value and clears associated interrupt flags.
 */
void ADC1_COMP_IRQHandler(void)
{
	// Read the ADC value
	adc_val = HAL_ADC_GetValue(&hadc);

	// Clear associated interrupt flags
	HAL_ADC_IRQHandler(&hadc);
}

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
