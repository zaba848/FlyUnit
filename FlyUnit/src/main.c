#include <dma.h>
#include <gpio.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32_hal_legacy.h>
#include <stm32f401xe.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_cortex.h>
#include <stm32f4xx_hal_flash_ex.h>
#include <stm32f4xx_hal_pwr_ex.h>
#include <stm32f4xx_hal_rcc.h>
#include <stm32f4xx_hal_tim.h>
#include <tim.h>
#include <usart.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void SystemClock_Config(void);


int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();

	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();



  char printBuffer[10];
  char rededCommand[5];
//  uint8_t redBuffer[3];
//  uint8_t redBufferBack[3];
  char readed[3];
  uint8_t readedBack[3];
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_ALL);
//	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_ALL);


	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

//	uint8_t angleLeft = 5;
//	uint8_t angleRight = 5;


	  sprintf(printBuffer,"setart reciving\n");
	  send_USB(printBuffer);

//	  serwoAngle(SS_LEWE, 140);
//	  int itera = 0;

	  readed[0] = 0;
	  readed[1] = 0;
	  readedBack[0] = 0;
	  readedBack[1] = 0;


  while (1)
  {


		recive(rededCommand);
		send_USB(rededCommand);
//	  send_USB(&rededCommand[0]);



	  switch(rededCommand[0])
	  {
	  case 127:	// X
	  {
		  while((rededCommand[0] == 127))
		  {


			  recive(rededCommand);
			if(rededCommand[0] == 126)
			{

				 readed[0] = readedBack[0];
				 readed[1] = readedBack[1];
				break;
			}
		  }
		  if(rededCommand[0] != 126)
		  readed[0] = rededCommand[0];

	  }break;
	  case 126:	// Y
	  {
		  while(rededCommand[0] == 126)
		  {
			  recive(rededCommand);

				if(rededCommand[0] == 127)
					{

						 readed[0] = readedBack[0];
						 readed[1] = readedBack[1];
						break;
					}
		  }
		  if(rededCommand[0] != 127)
		  readed[1] = rededCommand[0];

	  }break;
	  default:
		  break;
	  }


	  send_USB(readed);


	  if ((readed[1] != readedBack[1]) || (readed[0] != readedBack[0])) {

			readedBack[1] = readed[1];
			readedBack[0] = readed[0];
			if ((readed[1] != 0) && (readed[0] != 0))
				serwoControl((int8_t) readed[0], (int8_t) readed[1]);
		}

//	  if(printBuffer != " ")
//	  {
//		  send_USB(printBuffer);
//		  sprintf(printBuffer," ");
//	  }
  }

}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 74;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
printf("Wrong parameters value: file %s on line %d\r\n", file, line) ;


}

#endif
