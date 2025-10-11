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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//DEFINES
#define LED_PIN     LL_GPIO_PIN_5   // LED PIN
#define LED_PORT    GPIOA
#define BUTTON_PIN  LL_GPIO_PIN_13 // Input Pin
#define BUTTON_PORT GPIOC

//Variablen

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	while (!LL_USART_IsActiveFlag_TXE(USART2)) {
	}
	LL_USART_TransmitData8(USART2, ch);
	return ch;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* System interrupt init*/
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	//LL_SYSTICK_EnableIT();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // channel aktivieren
	LL_TIM_EnableAllOutputs(TIM2); // outputs aktivieren
	LL_TIM_EnableCounter(TIM2);    // counter starten
	//printf("Hallo Echtzeitsysteme!\r\n"); // hier weil USART2 erst in Zeile 99 initialisiert wird
	//uint32_t blinkDelay = 750; // halbe Periode in ms T = 2 * blinkDelay -> 1500ms
	uint32_t arr = LL_TIM_GetAutoReload(TIM2);
	int32_t helligkeit = 0; // (CCR/ARR) * 100 --> Arr = 999 CCR = 500 -> helligkeit = 50%
	int8_t schritte = 20; // wie schnell wird Helligkeit stärker
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/*
		 * Aufgabe aus dem Setup
		 if (LL_GPIO_IsInputPinSet(BUTTON_PORT, BUTTON_PIN)) {
		 blinkDelay = 250;
		 } else {
		 blinkDelay = 750;
		 }

		 //LED anschalten
		 LL_GPIO_TogglePin(LED_PORT, LED_PIN);
		 // delay einstellen
		 LL_mDelay(blinkDelay);

		 printf("[%lu ms] LED toggled\n", SysTicks);
		 */

		LL_TIM_OC_SetCompareCH1(TIM2, (uint32_t) helligkeit); //CCR wird gesetzt
		LL_mDelay(10);

		helligkeit += schritte; // helligkeit hochschrauben

		//wenn TOP erreicht dann soll helligkeit wieder runtergehen
		if (helligkeit >= (int32_t) arr) {
			helligkeit = arr;
			schritte = -schritte; //schritte negativ machen
		}
		//wenn 0 erreicht soll es wieder nach oben gehen
		if (helligkeit <= 0) {
			helligkeit = 0;
			schritte = -schritte; //schritte wieder zurücksetzen
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
	LL_PWR_DisableOverDriveMode();
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1) {

	}
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336,
			LL_RCC_PLLP_DIV_4);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	while (LL_PWR_IsActiveFlag_VOS() == 0) {
	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_Init1msTick(84000000);
	LL_SetSystemCoreClock(84000000);
	LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
