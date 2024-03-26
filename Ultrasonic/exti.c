
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
//#include "stm32f4xx.h"
//#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void GPIO_Init(void);
void Trigger_Sensor(void);
void Interrupt_Config (void);
void delay_us(uint32_t microseconds);
void UART_Print(UART_HandleTypeDef *huart, char *str);
void TIM5_Init(void);

volatile uint32_t pulseWidth = 0;
volatile uint32_t startTime = 0;
volatile uint8_t edgeDetected = 0; // 0 for rising edge, 1 for falling edge

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();

	GPIO_Init();
	Interrupt_Config();
	TIM5_Init();
	HAL_Delay(100);

	while (1)
	{
		HAL_Delay(50);
		Trigger_Sensor();
		HAL_Delay(5); // Wait for sensor to settle

		// Assuming pulseWidth is defined and represents the time in microseconds
		int distance_cm = (int)((pulseWidth / 100) * 0.034 / 2); // Integer part of the distance
		int distance_fraction = (int)(((pulseWidth) * 0.034 / 2) * 100) % 100; // Fractional part after multiplying by 100

		char buffer[32]; // Buffer to hold the formatted string
		// Format and print the distance with two decimal places, without using floating point in sprintf
		sprintf(buffer, "Distance: %d.%02d\r\n", distance_cm, distance_fraction);
		UART_Print(&huart2, buffer); // Send the string over USART2

		// Reset all LEDs first
		// PB15, 14, 13
		GPIOB->BRR = (1 << 15) | (1 << 14) | (1 << 13);

		// Conditionally set LEDs based on the measured distance
		if (distance_cm < 10) {
			// RED LED
			GPIOB->BSRR = (1 << 15);
		}
		else if (distance_cm >= 10 && distance_cm < 20) {
			// YELLOW LED
			GPIOB->BSRR = (1 << 14);
		}
		else if (distance_cm >= 20 && distance_cm < 35) {
			// WHITE LED
			GPIOB->BSRR = (1 << 13);
		}
		else {
			// Max distance it detects is around 40cm
			continue;
		}
		// If distance >= 15, LEDs remain reset from the initial reset command
	}
}

void UART_Print(UART_HandleTypeDef *huart, char *str) {
    HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void GPIO_Init(void) {

	// Initialize GPIOB Clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN);

	// Initialize B2 (ECHO) and B1 (TRIGGER)
	// Clear both with 1111 to B2 and B1
	GPIOB->MODER &= ~(0xF<<2);

	// Echo already at input, Trigger to Output
	GPIOB->MODER |= (0x1<<2);

	GPIOB->PUPDR |= (0x1<<5);

	// Initialize B15, B14, B13 to Red, Yellow, and White LEDs
	// 111111 to B15, B14, B13
	GPIOB->MODER &= ~(0x3F<<26);

	// 010101 to B15, B14, B13
	GPIOB->MODER |= (0x15<<26);
}

void TIM5_Init(void) {
    __enable_irq();
    RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM5EN);	// turn on TIM5
    TIM5->PSC = 3;
    NVIC->ISER[0] = (1 << (TIM5_IRQn & 0x1F));
    TIM5->CR1 |= TIM_CR1_CEN;			// start timer

}

void Trigger_Sensor(void) {
    // Set trigger pin high
    GPIOB->ODR |= (1<<1); // Adjust GPIOx, GPIO_PIN_y as necessary
    delay_us(10); // 10us delay

    // Set trigger pin low
    GPIOB->ODR &= ~(1<<1);
}

void EXTI2_IRQHandler(void)
{
    // Clear the EXTI line pending bit first to prevent missing interrupts

    	EXTI->PR1 |= (1<<2); // Set flag

        // Check the current state of the pin to determine the edge
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) {
            // Rising edge detected
            if (edgeDetected == 0) {
                startTime = TIM5->CNT; // Use a hardware timer for microsecond resolution
                edgeDetected = 1; // Next expected edge is falling
            }
        } else {
            // Falling edge detected
            if (edgeDetected == 1) {
                pulseWidth = TIM5->CNT - startTime; // Calculate the pulse width
                edgeDetected = 0; // Reset for the next cycle, expecting a rising edge
            }
        }

}

void Interrupt_Config (void)
{
	/*************>>>>>>> STEPS FOLLOWED <<<<<<<<************

	1. Enable the SYSCNFG bit in RCC register
	2. Configure the EXTI configuration Regiter in the SYSCNFG
	3. Enable the EXTI using Interrupt Mask Register (IMR)
	4. Configure the Rising Edge / Falling Edge Trigger
	5. Set the Interrupt Priority
	6. Enable the interrupt

	********************************************************/

	RCC->APB2ENR |= (1<<14);  // Enable SYSCNFG

	SYSCFG->EXTICR[0] &= ~(0xf<<8);  // Bits[11:10:9:8] = (0:0:0:1)  -> configure EXTICR1 to access EXTI2
	SYSCFG->EXTICR[0] |= 0x1 << 8;

	EXTI->IMR1 |= (1<<2);  // Bit[1] = 1  --> Enable the Mask on EXTI 1

	EXTI->RTSR1 |= (1<<2);  // Enable Rising Edge Trigger for PB2

	EXTI->FTSR1 |= (1<<2);  // Enable Falling Edge Trigger for PB2

	NVIC_SetPriority (EXTI2_IRQn, 0);  // Set Priority

	NVIC_EnableIRQ (EXTI2_IRQn);  // Enable Interrupt

}

void delay_us(uint32_t microseconds) {
    // Simple loop for delay
    // Each loop takes 4 cycles (this is an approximation and depends on the compiler and MCU architecture)
    uint32_t loops = (microseconds * (4000000 / 1000000)); // 4MHz / 4 cycles per loop

    for(uint32_t i = 0; i < loops; i++) {
        // Do nothing, just wait
        __asm("NOP"); // Assembly instruction for No Operation, ensures the loop does not get optimized out
    }
}


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//    static uint32_t startTime = 0;
//
//    if (GPIOB->IDR & (0x1<<2)) { // Check if the interrupt comes from the echo pin
//        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) { // Rising edge
//            startTime = HAL_GetTick(); // Record start time
//        } else { // Falling edge
//            pulseWidth = HAL_GetTick() - startTime; // Calculate pulse width
//            // Now pulseWidth holds the duration of the pulse in milliseconds
//        }
//    }
//}

//int flag = 0;

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
