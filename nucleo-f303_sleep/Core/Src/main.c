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
  *
  * STM32F303-prosessorin virransäästötilojen demo.
  * F303RE osaa 3 virransäästötilaa:
  * - Sleep Mode ( I/O-pinnien tilat säilyvät)
  * - Stop Mode ( I/O-pinnien tilat säilyvät, HSI ja HSE-kellosignaalit pysähtyvät, muisti ja rekisterit säilyvät)
  * - Standby Mode
  * Tilojen virrankulutus alenee progressiivisesti samalla kuin erilaisia resursseja suljetaan
  * Standby-tila ei enää ylläpidä muistin eikä prosessorin rekisterien tilaa. Ainoastaan
  * reaaliaikakellon tilataltion RAM-muisti säilyy
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include "menu.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

int _read(int fd, char *ptr, int len) {
	if (fd == STDIN_FILENO ) {
		HAL_UART_Receive(&huart2, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
	    HAL_UART_Transmit(&huart2, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
	}
	return 1;
}

int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    hstatus = HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

void showMenu() {
	printf(" \r\nSleep demo menu----------------------\r\n\r\n");
	printf("1: Enter sleep mode. Wakeup with EXTI interrupt by blue button\r\n");
	printf("2: Enter stop mode.  Wakeup with EXTI interrupt by blue button\r\n");
	printf("3: Enter Standby mode. Wakeup by reset\r\n");
	printf("4: Enter Standby mode. Wakeup by RTC\r\n");
	printf("5: \r\n");
	printf("6: \r\n");
}
uint8_t getCommandFromUSART() {
	char cmd;
	while ( 1 ) {
		printf("Enter sleep mode - 1: sleep; 2: stop; 3: standby; 4: sleep+auto wakeup\r\n");
		_read(0, &cmd, 1);
		if ( cmd == '1' ) return 1;
		if ( cmd == '2' ) return 2;
		if ( cmd == '3' ) return 3;
		if ( cmd == '4' ) return 4;
		printf("%c did not compute\r\n", cmd);
	}
	return 0;	// jotta kääntyri on onnellinen, ei tähän koskaan oikeasti tulla
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t command;
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  printf("\r\nARM Low Power demo\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  command = getCommandFromUSART();

	  switch ( command ) {
		  case 1: {
			  printf("Entering Sleep mode now\r\n");
			  break;
		  }
		  case 2: {
			  printf("Entering Stop mode now\r\n");
			  break;
		  }
		  case 3: {
			  printf("Entering Standby mode now\r\n");
			  break;
		  }
		  case 4: {
			  printf("Entering Sleep+auto wakeup mode now\r\n");
			  break;
		  }
		  default: {
			  printf("That was not a sleep mode!\r\n");
		  }
	  }

	  printf("Good Night!\r\n");
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	  // Keskeytykset herättävät prosessorin virransäästötilasta
	  // sen takia Tick-keskeytys on pysäytettävä, muuten prosessori herää
	  // heti millisekunnin sisällä.
	  HAL_SuspendTick();

	  // Siirrytään valittuun virransäästötilaan
	  if ( command == 1) {
		  // Sleep Mode: CPU core pysähtyy. Muisti ja rekisterit säilyvät
		  // Osa oheislaitteista säilyy aktiivisina ja ne voivat herättää systeemin
		  // Low Power regulator: oheislaitteet toimivat alennetulla kellotaajuudella
		  HAL_PWR_EnterSLEEPMode( PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI );
	  }
	  else if ( command == 2 ) {
		  // Stop Mode: CPU core pysähtyy. Muisti ja rekisterit säilyvät
		  // Useimmat oheislaitteet pysähtyvät
		  // Reaaliaikakello on mahdollista pysäyttää virran säästämiseksi
		  HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI );
		  SystemClock_Config();
	  }
	  else if ( command == 3 ) {
		  HAL_PWR_EnterSTANDBYMode();
	  }
	  else if ( command == 4) {
		  // Sleep Mode: CPU core pysähtyy. Muisti ja rekisterit säilyvät
		  // Osa oheislaitteista säilyy aktiivisina ja ne voivat herättää systeemin
		  // Low Power regulator: oheislaitteet toimivat alennetulla kellotaajuudella
		  // Systeemi herää automaattisesti reaaliaikakellon keskeytyksestä.
		  // Muuttuja sleepTime määrää nukkumisajan millisekunteina
#define RTC_CLOCK_FREQ 32768
#define RTC_CLOCK_DIVIDER 16
		  uint32_t sleepTime = ( ( (uint32_t) 5000 ) * ( RTC_CLOCK_FREQ / RTC_CLOCK_DIVIDER ) ) / 1000;
		  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleepTime, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

		  HAL_PWR_EnterSLEEPMode( PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI );
	  }

	  printf("Good Morning!\r\n");


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
