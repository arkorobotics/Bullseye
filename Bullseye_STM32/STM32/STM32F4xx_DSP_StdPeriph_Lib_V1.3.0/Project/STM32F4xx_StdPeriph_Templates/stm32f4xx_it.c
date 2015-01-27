/**
  ******************************************************************************
  * @file    TIM/TIM_PWMInput/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.3.0
  * @date    13-November-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup TIM_PWMInput
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t IC4Value = 0;
__IO uint16_t DutyCycle4 = 0;
__IO uint32_t Frequency4 = 0;

__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle2 = 0;
__IO uint32_t Frequency2 = 0;

__IO uint16_t IC5Value = 0;
__IO uint16_t DutyCycle5 = 0;
__IO uint32_t Frequency5 = 0;

__IO uint16_t uhIC3ReadValue1 = 0;
__IO uint16_t uhIC3ReadValue2 = 0;
__IO uint16_t uhCaptureNumber = 0;
__IO uint32_t uwCapture = 0;
__IO uint32_t uwTIM1Freq = 0;


int error5 = 0;
int integral5 = 0;
int diff5 = 0;
int last_error5 = 0;
	int Kp = 1;
	double Ki = 0.1;
	double Kd = 0.1;
unsigned char PWM_Out5;
int feedback = 0;
uint32_t freq5;
int CMD = 1650;

extern void SetLeftFrontWheelPwm(int);
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void TimingDelay_Decrement(void);

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
	
  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC4Value = TIM_GetCapture2(TIM4);

  if (IC4Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle4 = (TIM_GetCapture1(TIM4) * 100) / IC4Value;

    /* Frequency computation 
       TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

    Frequency4 = ((RCC_Clocks.HCLK_Frequency)/6.25) / IC4Value;
  }
  else
  {
    DutyCycle4 = 0;
    Frequency4 = 0;
  }
}

void TIM2_IRQHandler(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC2Value = TIM_GetCapture2(TIM2);

  if (IC2Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle2 = (TIM_GetCapture1(TIM2) * 100) / IC2Value;

    /* Frequency computation 
       TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

    Frequency2 = ((RCC_Clocks.HCLK_Frequency)/6.25) / IC2Value;
  }
  else
  {
    DutyCycle2 = 0;
    Frequency2 = 0;
  }
}

void TIM1_CC_IRQHandler(void)
{ 
  if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) 
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    if(uhCaptureNumber == 0)
    {
      /* Get the Input Capture value */
      uhIC3ReadValue1 = TIM_GetCapture2(TIM1);
      uhCaptureNumber = 1;
    }
    else if(uhCaptureNumber == 1)
    {
      /* Get the Input Capture value */
      uhIC3ReadValue2 = TIM_GetCapture2(TIM1); 
      
      /* Capture computation */
      if (uhIC3ReadValue2 > uhIC3ReadValue1)
      {
        uwCapture = (uhIC3ReadValue2 - uhIC3ReadValue1); 
      }
      else if (uhIC3ReadValue2 < uhIC3ReadValue1)
      {
        uwCapture = ((0xFFFF - uhIC3ReadValue1) + uhIC3ReadValue2); 
      }
      else
      {
        uwCapture = 0;
      }
      /* Frequency computation */ 
      uwTIM1Freq = (uint32_t) SystemCoreClock/3.124 / uwCapture;
      uhCaptureNumber = 0;
    }
  }
}

void TIM5_IRQHandler(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);

  /* Clear TIM4 Capture compare interrupt pending bit */
  TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);

  /* Get the Input Capture value */
  IC5Value = TIM_GetCapture2(TIM5);

  if (IC5Value != 0)
  {
    /* Duty cycle computation */
    DutyCycle5 = (TIM_GetCapture1(TIM5) * 100) / IC5Value;

    /* Frequency computation 
       TIM4 counter clock = (RCC_Clocks.HCLK_Frequency)/2 */

    Frequency5 = ((RCC_Clocks.HCLK_Frequency)/6.25) / IC5Value;
  }
  else
  {
    DutyCycle5 = 0;
    Frequency5 = 0;
  }
}

void TIM7_IRQHandler(void)
{ 
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
{
TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  if(Frequency5>3300)
	{freq5 = 3300;}
	else {freq5 = Frequency5;}
	
	error5 = CMD - freq5; //-1050
	integral5 = integral5 + error5;//-1050
	if (integral5> 3300)
	{integral5 = 3300;}
		if (integral5<0)
	{integral5 = 0;}
	diff5 = (error5 - last_error5);//-1050
	PWM_Out5 = (((0.05)*error5)+((0.01)*integral5) + ((0)*diff5));
	if (PWM_Out5>100)
		{PWM_Out5=100;}
	if (PWM_Out5<0)
		{PWM_Out5=0;}
	last_error5 = error5;
SetLeftFrontWheelPwm(PWM_Out5);
	}
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
