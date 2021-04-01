#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include "menu.h"
#include "setpoint.h"
#include "pid.h"
#include "pwm.h"
#include "speedctrl.h"
#include "stm32f303xe.h"

UART_HandleTypeDef huart2;

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
using namespace setpoint;
using namespace actual;
using namespace pid;
using namespace pwm;
using namespace speedcontrol;

#define MAXSPEED 1500.0	// RPM
#define MINSPEED -1500.0	// RPM

#define MAXACC 2000	// RPM/s^2
#define MINACC 10

#define MAXPGAIN 10000.0
#define MAXIGAIN 100.0
#define MAXDGAIN 100.0

menu mainMenu("Speed control demo menu---------------");

// Säätäjän toiminnalliset lohkot (luokat)
Setpoint speedSetpoint;
AvgActual actualSpeed(&htim2, TIM_CHANNEL_ALL);
PID pidController( 1000, true, 50, 0.1, 0 );
PWM Hbridge(&htim1, TIM_CHANNEL_3, IN1_A_GPIO_Port, IN1_A_Pin, IN1_B_GPIO_Port, IN1_B_Pin);
Speedcontrol ctrlLoop( &speedSetpoint, &actualSpeed, &pidController, &Hbridge );

void speedHandler( char _selector );
void accHandler( char _selector );
void moveHandler( char _selector );
void actualMvmtHandler( char _selector );
void KpHandler( char _selector );
void KiHandler( char _selector );
void KdHandler( char _selector );

// Päävalikko
#define NUM_MAINMENUITEMS 8
menuItem mainMenuItems[] =
{	{'1', "Set shaft speed RPM ( -1500...0...+1500 )", speedHandler},
	{'2', "Set acceleration RPM/s^2 ( 10...1000 )", accHandler},
	{'3', "Run", moveHandler},
	{'4', "STOP", moveHandler},
	{'a', "Display motor movement", actualMvmtHandler},
	{'p', "Set proportional gain Kp", KpHandler},
	{'i', "Set integrating gain  Ki", KiHandler},
	{'d', "Set derivating gain   Kd", KdHandler},
};

void speedHandler( char _selector ) {
	float sRef;
	uint8_t numArgs;
	while (1) {
		printf("Set speed: ");
		fflush(stdout);
		numArgs = scanf("%f", &sRef );
		if ( numArgs == 1 ) {
			if ( ( sRef <= MAXSPEED )  && ( sRef >= MINSPEED )) {
				speedSetpoint.setTargetRPM( sRef );
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
				speedSetpoint.setIncrementRPM( tmpAcc );
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
		ctrlLoop.run();
	}
	else ctrlLoop.stop();
}

void actualMvmtHandler( char _selector ) {
	float instantVelocity;
	float avgRPM;
	char dummy;
	while ( 1 ) {
		// tulostetaan raaka-arvot
		// tulostetaan skaalatut liikearvot
		instantVelocity = actualSpeed.getSpeed();
		avgRPM = actualSpeed.getAvgSpeedRPM();
		printf("Inst: %3d, RPM: %5.1f Pos: %-12d\r",
				actualSpeed.getSpeed(),
				actualSpeed.getAvgSpeedRPM(),
				actualSpeed.getPosition() );
		fflush(stdout);
		if ( USART2->ISR & UART_FLAG_RXNE) {
			dummy = USART2->RDR;
			printf("\r\n");
			return;
		}
		HAL_Delay(100);
	}
}

void KpHandler( char _selector ) {
	uint8_t numArgs;
	float tmpGain;
	while ( 1 ) {
		printf("Set P gain: ");
		fflush(stdout);
		numArgs = scanf("%f", &tmpGain );
		if ( numArgs == 1 ) {
			if ( tmpGain <= MAXPGAIN && tmpGain >= EPSILON ) {
				pidController.setKp( tmpGain );
				return;
			}
		}
	}
}

void KiHandler( char _selector ) {
	uint8_t numArgs;
	float tmpGain;
	while ( 1 ) {
		printf("Set I gain: ");
		fflush(stdout);
		numArgs = scanf("%f", &tmpGain );
		if ( numArgs == 1 ) {
			if ( tmpGain <= MAXIGAIN && tmpGain >= EPSILON ) {
				pidController.setKi( tmpGain );
				return;
			}
		}
	}
}

void KdHandler( char _selector ) {
	uint8_t numArgs;
	float tmpGain;
	while ( 1 ) {
		printf("Set D gain: ");
		fflush(stdout);
		numArgs = scanf("%f", &tmpGain );
		if ( numArgs == 1 ) {
			if ( tmpGain <= MAXDGAIN && tmpGain >= EPSILON ) {
				pidController.setKd( tmpGain );
				return;
			}
		}
	}
}

void runStateMachine() { ctrlLoop.stateMachine(); }

int main(void) {


	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART2_UART_Init();

	// H-bridge PWM timer
	MX_TIM1_Init();

	// Quadrature encoder counter timer
	MX_TIM2_Init();

	// Velocity actual update timer
	MX_TIM3_Init();

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
