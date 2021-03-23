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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include "menu.h"
#include "hbridge.h"
#include "stm32f303xe.h"

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// extern "C" kertoo kääntäjälle, että sen rajaamassa lohkossa esitellyt jutut
// on käännetty C-kääntäjällä. Tässä meillä on C++ -kääntäjä, jonka
// funktioiden kutsukonventio on erilainen kuin perus-C kielessä.
// Tällä saadaan kääntäjä kutsumaan C-kielisiä binäärejä C-kielen säännöillä
extern "C" {
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

} // extern "C"

using namespace CLIMenu;
using namespace H_bridge;

#define MAXSPEED 1000
uint16_t setSpeed = 100;
uint16_t currentSpeed = 0;

#define MAXACC 5000
#define MINACC 1
uint16_t setAcc = 10;
uint16_t accIterator = 0;

hbridge pwm(&htim1, TIM_CHANNEL_3, IN1_A_GPIO_Port, IN1_A_Pin, IN1_B_GPIO_Port, IN1_B_Pin );
menu mainMenu("Sleep demo menu----------------------");

void speedHandler( char _selector );
void accHandler( char _selector );
void moveHandler( char _selector );

// Päävalikko
#define NUM_MAINMENUITEMS 7
menuItem mainMenuItems[] =
{	{'1', "Set shaft speed ( 0-1000 )", speedHandler},
	{'2', "Set acceleration ( %% per second )", accHandler},
	{'3', "Move forward", moveHandler},
	{'4', "Move backward", moveHandler},
	{'5', "STOP by ramping down", moveHandler},
	{'6', "STOP by bridge cutoff", moveHandler},
	{'7', "STOP by bridge short circuit", moveHandler}
};

void speedHandler( char _selector ) {
	uint8_t numArgs;
	uint32_t tmpSpeed;
	while (1) {
		printf("Set speed: ");
		fflush(stdout);
		numArgs = scanf("%d", &tmpSpeed );
		if ( numArgs == 1 ) {
			if ( tmpSpeed <= MAXSPEED ) {
				pwm.setSpeedReference( tmpSpeed );
				return;
			}
		}
		else {
			printf("That did not compute!\r\n");
			return;
		}
	}
}

void accHandler( char _selector ) {
	uint8_t numArgs;
	uint32_t tmpAcc;
	while (1) {
		printf("Set acceleration: ");
		fflush(stdout);
		numArgs = scanf("%d", &tmpAcc );
		if ( numArgs == 1 ) {
			if ( tmpAcc <= MAXACC && tmpAcc >= MINACC ) {
				pwm.setAccReference( tmpAcc );
				return;
			}
		}
		else {
			printf("That did not compute!\r\n");
			return;
		}
	}

}

void moveHandler( char _selector ) {
	if ( _selector == '3' ) {
		pwm.forward();
	}
	else if ( _selector == '4' ) {
		pwm.reverse();
	}
	else if ( _selector == '5' ) {
		pwm.stop();
	}
	else if ( _selector == '6' ) {
		pwm.cutoff();
	}
	else {
		pwm.brake();
	}
}

void runStateMachine() {
	pwm.stateMachine();
}

int main(void) {


	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();

	printf("Simple DC motor control using PWM voltage\r\n");
	for ( uint8_t i=0; i<NUM_MAINMENUITEMS; i++ ) {
		mainMenu.addItem( &mainMenuItems[i] );
	}
	mainMenu.run( true );

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

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
