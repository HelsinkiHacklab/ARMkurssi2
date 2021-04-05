/*
 * ARMkurssi - yksinkertainen moottorin pyörimisnopeuden mittaus pulssivälin ajastuksella
 * Author: Kremmen 2021-04-05
 *
 */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <errno.h>

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
//---------------------

/* Kriittisen sektion vahtifunktiot ----------------------------------------------*/
volatile uint8_t SR_reg;               /* FAULTMASK rekisterin hetkellinen arvo */
volatile uint8_t SR_lock = 0x00U;      /* lukko */

/* Tallennetaan coren status ja kielletään keskeytykset */
#define EnterCritical() \
  do {\
    if (++SR_lock == 1u) {\
      /*lint -save  -e586 -e950 Disable MISRA rule (2.1,1.1) checking. */\
      asm ( \
      "MRS R0, PRIMASK\n\t" \
      "CPSID i\n\t"            \
      "STRB R0, %[output]"  \
      : [output] "=m" (SR_reg)\
      :: "r0");\
      /*lint -restore Enable MISRA rule (2.1,1.1) checking. */\
  }\
} while(0)

/* Palautetaan prosessorin status  */
#define ExitCritical() \
  do {\
    if (--SR_lock == 0u) { \
      /*lint -save  -e586 -e950 Disable MISRA rule (2.1,1.1) checking. */\
      asm (                 \
      "ldrb r0, %[input]\n\t"\
      "msr PRIMASK,r0;\n\t" \
      ::[input] "m" (SR_reg)  \
      : "r0");                \
      /*lint -restore Enable MISRA rule (2.1,1.1) checking. */\
    }\
  } while(0)

/*-------------------------------------------------------------------------------*/

enum timingState { waitingStart, waitingEnd };

volatile timingState state = waitingStart;
volatile uint32_t startTimeStamp;
volatile uint32_t endTimeStamp;
volatile uint32_t time;
volatile uint32_t frequency;
volatile uint32_t timerOverrunCount;
volatile uint32_t rotationFrequency;
uint32_t timerPeriod;
uint32_t sysClockFrequency;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
	if ( htim->Instance == TIM2 ) {
		if ( state == waitingStart ) {
			startTimeStamp = htim->Instance->CCR1;
			timerOverrunCount = 0;
			state = waitingEnd;
		}
		else { // waitingEnd
			endTimeStamp = htim->Instance->CCR1;
			time = endTimeStamp + timerOverrunCount * timerPeriod - startTimeStamp;
			frequency = sysClockFrequency / time;
			state = waitingStart;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	timerOverrunCount++;
}


int main(void) {

  HAL_Init();

  SystemClock_Config();

  sysClockFrequency =  HAL_RCC_GetSysClockFreq();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  timerPeriod = htim2.Instance->ARR +1;

  while (1) {

  }

}

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }

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
