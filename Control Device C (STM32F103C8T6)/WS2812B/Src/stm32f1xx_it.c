/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "ws2812.h"
#include "main.h"

volatile unsigned char state;			
volatile unsigned char nextState;	
volatile uint8_t stairsCleared = 0;

extern uint16_t BUF_DMA [ARRAY_LEN];
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim4_ch3;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern DMA_HandleTypeDef hdma_tim4_ch1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	HAL_TIM_PWM_Stop_DMA(&htim4,TIM_CHANNEL_1);
  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
	
	if(stairsCleared)
	{
		HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_2,(uint32_t*)&BUF_DMA,ARRAY_LEN);
	}
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel4 global interrupt.
*/
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
	HAL_TIM_PWM_Stop_DMA(&htim4,TIM_CHANNEL_2);
  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch2);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
	if(stairsCleared)
	{
		HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_3,(uint32_t*)&BUF_DMA,ARRAY_LEN);
		
		stairsCleared = 0;
	}
  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
	HAL_TIM_PWM_Stop_DMA(&htim4,TIM_CHANNEL_3);
  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim4_ch3);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	state = FIRST_STEP_ON;

  HAL_TIM_Base_Start(&htim3);	
	HAL_TIM_Base_Start_IT(&htim3);	

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	switch(state)
	{
		case FIRST_STEP_ON:
			
			nextState = SECOND_STEP_ON;
		
			for(int i = 0; i< LED_COUNT; i++)
			{
				ws2812_pixel_rgb_to_buf_dma(0,0,255,i);
			}
		
			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_1,(uint32_t*)&BUF_DMA,ARRAY_LEN);
			
		break;
		
		case SECOND_STEP_ON:
			
			nextState = THIRD_STEP_ON;
	
			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_2,(uint32_t*)&BUF_DMA,ARRAY_LEN);
			
			break;
				
		case THIRD_STEP_ON:
			
			nextState = FIRST_STEP_OFF;
		
			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_3,(uint32_t*)&BUF_DMA,ARRAY_LEN);
		
			break;
						
		case FIRST_STEP_OFF:
			
			nextState = SECOND_STEP_OFF;
		
			for(int i = 0; i< LED_COUNT; i++)
			{
				ws2812_pixel_rgb_to_buf_dma(255,0,0,i);
			}
			
			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_1,(uint32_t*)&BUF_DMA,ARRAY_LEN);
		
			break;
								
		case SECOND_STEP_OFF:
			
			nextState = THIRD_STEP_OFF;

			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_2,(uint32_t*)&BUF_DMA,ARRAY_LEN);
		
			break;
		
		case THIRD_STEP_OFF:
			
			nextState = ALL_OFF;
			
			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_3,(uint32_t*)&BUF_DMA,ARRAY_LEN);
		
			break;
												
		case ALL_OFF:
			
			//nextState = FIRST_STEP_ON;
		
			ws2812_clear();
			
			stairsCleared = 1;
		
			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_1,(uint32_t*)&BUF_DMA,ARRAY_LEN);
		
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_Base_Stop(&htim3);
		break;
														
		default:
			ws2812_clear();
			break;
		
	}

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
	
  /* USER CODE BEGIN TIM3_IRQn 1 */
	state = nextState;
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
