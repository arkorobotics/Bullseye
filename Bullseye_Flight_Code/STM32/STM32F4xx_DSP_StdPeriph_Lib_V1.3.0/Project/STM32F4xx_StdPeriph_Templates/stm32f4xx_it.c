// --------------------------------------------------------------------
// Bullseye - FLIGHT CODE
// --------------------------------------------------------------------
//
// Authors:
// -> Ara Kourchians
// -> Sabrina Kaefer
// -> David Ramirez
//
// --------------------------------------------------------------------
// STM32F4XX_IT.C
// --------------------------------------------------------------------

#include "stm32f4xx_it.h"

/////////////////////////////////////////////////////////////////////////////
// Left Front Wheel - Variables
/////////////////////////////////////////////////////////////////////////////
__IO uint16_t IC5Value = 0;
__IO uint16_t DutyCycle5 = 0;
__IO uint32_t LeftFront_Frequency_Raw = 0;
double LeftFront_Kp = 0.278;
double LeftFront_Ki = 0.003;
double LeftFront_Kd = 0.0008;
int LeftFront_Error = 0;
int LeftFront_Integral = 0;
int LeftFront_Diff = 0;
int LeftFront_PID = 0;
int Last_LeftFront_Error = 0;
unsigned char LeftFront_PWM;
uint32_t LeftFront_Frequency;


/////////////////////////////////////////////////////////////////////////////
// Left Back Wheel - Variables
/////////////////////////////////////////////////////////////////////////////
__IO uint16_t uhIC3ReadValue1 = 0;
__IO uint16_t uhIC3ReadValue2 = 0;
__IO uint16_t uhCaptureNumber = 0;
__IO uint32_t uwCapture = 0;
__IO uint32_t LeftBack_Frequency_Raw = 0;
double LeftBack_Kp = 0.2782;
double LeftBack_Ki = 0.00365;
double LeftBack_Kd = 0.00097;
int LeftBack_Error = 0;
int LeftBack_Integral = 0;
int LeftBack_Diff = 0;
int LeftBack_PID = 0;
int Last_LeftBack_Error = 0;
unsigned char LeftBack_PWM;
uint32_t LeftBack_Frequency;


/////////////////////////////////////////////////////////////////////////////
// Right Front Wheel - Variables
/////////////////////////////////////////////////////////////////////////////
__IO uint16_t IC4Value = 0;
__IO uint16_t DutyCycle4 = 0;
__IO uint32_t RightFront_Frequency_Raw = 0;
double RightFront_Kp = 0.278;
double RightFront_Ki = 0.0037;
double RightFront_Kd = 0.00087;
int RightFront_Error = 0;
int RightFront_Integral = 0;
int RightFront_Diff = 0;
int RightFront_PID = 0;
int Last_RightFront_Error = 0;
unsigned char RightFront_PWM;
uint32_t RightFront_Frequency;


/////////////////////////////////////////////////////////////////////////////
// Right Back Wheel - Variables
/////////////////////////////////////////////////////////////////////////////
__IO uint16_t IC2Value = 0;
__IO uint16_t DutyCycle2 = 0;
__IO uint32_t RightBack_Frequency_Raw = 0;
double RightBack_Kp = 0.278;
double RightBack_Ki = 0.0035;
double RightBack_Kd = 0.0008;
int RightBack_Error = 0;
int RightBack_Integral = 0;
int RightBack_Diff = 0;
int RightBack_PID = 0;
int Last_RightBack_Error = 0;
unsigned char RightBack_PWM;
uint32_t RightBack_Frequency;



/////////////////////////////////////////////////////////////////////////////
// Wheel Command - Variables
/////////////////////////////////////////////////////////////////////////////
int CMD_Left = 1500;
int CMD_Right = 1500;


extern void SetLeftFrontWheelPwm(int);
extern void SetLeftBackWheelPwm(int);
extern void SetRightFrontWheelPwm(int);
extern void SetRightBackWheelPwm(int);

void TimingDelay_Decrement(void);



/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/


/////////////////////////////////////////////////////////////////////////////
// Left Front Wheel - Encoder Interrupt - Timer 5
/////////////////////////////////////////////////////////////////////////////
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

		/* Frequency computation */
		LeftFront_Frequency_Raw = ((RCC_Clocks.HCLK_Frequency)/6.25) / IC5Value;
	}
	else
	{
		DutyCycle5 = 0;
		LeftFront_Frequency_Raw = 0;
	}
}


/////////////////////////////////////////////////////////////////////////////
// Left Back Wheel - Encoder Interrupt - Timer 1
/////////////////////////////////////////////////////////////////////////////
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

			if (uwCapture != 0)
			{
				/* Frequency computation */ 
				LeftBack_Frequency_Raw = (uint32_t) SystemCoreClock/3.124 / uwCapture;
				uhCaptureNumber = 0;
			}
			else
			{
				/* Frequency computation */ 
				LeftBack_Frequency_Raw = 0;
				uhCaptureNumber = 0;
			}
		}
	}
}


/////////////////////////////////////////////////////////////////////////////
// Right Front Wheel - Encoder Interrupt - Timer 4
/////////////////////////////////////////////////////////////////////////////
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

		/* Frequency computation  */
		RightFront_Frequency_Raw = ((RCC_Clocks.HCLK_Frequency)/6.25) / IC4Value;
	}
	else
	{
		DutyCycle4 = 0;
		RightFront_Frequency_Raw = 0;
	}
}


/////////////////////////////////////////////////////////////////////////////
// Right Back Wheel - Encoder Interrupt - Timer 2
/////////////////////////////////////////////////////////////////////////////
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

		/* Frequency computation */
		RightBack_Frequency_Raw = ((RCC_Clocks.HCLK_Frequency)/6.25) / IC2Value;
	}
	else
	{
		DutyCycle2 = 0;
		RightBack_Frequency_Raw = 0;
	}
}



/////////////////////////////////////////////////////////////////////////////
// PID Loop Interrupt
/////////////////////////////////////////////////////////////////////////////
void TIM7_IRQHandler(void)
{ 
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);


		/////////////////////////////////////////////////////////////////////////////
		// Left Front Wheel - PID
		/////////////////////////////////////////////////////////////////////////////
		if (LeftFront_Frequency_Raw > 3750)
		{
			LeftFront_Frequency = 3750;
		}
		else 
		{
			LeftFront_Frequency = LeftFront_Frequency_Raw;
		}

		LeftFront_Error = CMD_Left - LeftFront_Frequency;
		LeftFront_Integral = LeftFront_Integral + LeftFront_Error;
		
		if (LeftFront_Integral > 3750)
		{
			LeftFront_Integral = 3750;
		}

		if (LeftFront_Integral < 0)
		{
			LeftFront_Integral = 0;
		}
		
		LeftFront_Diff = (LeftFront_Error - Last_LeftFront_Error);
		LeftFront_PID = (((LeftFront_Kp)*LeftFront_Error) + ((LeftFront_Ki)*LeftFront_Integral) + ((LeftFront_Kd)*LeftFront_Diff));

		if (LeftFront_PID > 100)
		{
			LeftFront_PWM = 100;
		}
		else if (LeftFront_PID < 0)
		{
			LeftFront_PWM = 0;
		}
		else 
		{
			LeftFront_PWM = (unsigned char)LeftFront_PID;
		}
		
		Last_LeftFront_Error = LeftFront_Error;
		SetLeftFrontWheelPwm(LeftFront_PWM);


		/////////////////////////////////////////////////////////////////////////////
		// Left Back Wheel - PID
		/////////////////////////////////////////////////////////////////////////////
		if (LeftBack_Frequency_Raw > 3750)
		{
			LeftBack_Frequency = 3750;
		}
		else 
		{
			LeftBack_Frequency = LeftBack_Frequency_Raw;
		}

		LeftBack_Error = CMD_Left - LeftBack_Frequency;
		LeftBack_Integral = LeftBack_Integral + LeftBack_Error;

		if (LeftBack_Integral > 3750)
		{
			LeftBack_Integral = 3750;
		}
		if (LeftBack_Integral < 0)
		{
			LeftBack_Integral = 0;
		}

		LeftBack_Diff = (LeftBack_Error - Last_LeftBack_Error);
		LeftBack_PID = (((LeftBack_Kp)*LeftBack_Error) + ((LeftBack_Ki)*LeftBack_Integral) + ((LeftBack_Kd)*LeftBack_Diff));
		
		if (LeftBack_PID > 100)
		{
			LeftBack_PWM = 100;
		}
		else if (LeftBack_PID < 0)
		{
			LeftBack_PWM = 0;
		}
		else 
		{
			LeftBack_PWM = (unsigned char)LeftBack_PID;
		}

		Last_LeftBack_Error = LeftBack_Error;
		SetLeftBackWheelPwm(LeftBack_PWM);

		
		/////////////////////////////////////////////////////////////////////////////
		// Right Front Wheel - PID
		/////////////////////////////////////////////////////////////////////////////
		if(RightFront_Frequency_Raw > 3750)
		{
			RightFront_Frequency = 3750;
		}
		else 
		{
			RightFront_Frequency = RightFront_Frequency_Raw;
		}

		RightFront_Error = CMD_Right - RightFront_Frequency;
		RightFront_Integral = RightFront_Integral + RightFront_Error;
		
		if (RightFront_Integral > 3750)
		{
			RightFront_Integral = 3750;
		}
		if (RightFront_Integral < 0)
		{
			RightFront_Integral = 0;
		}

		RightFront_Diff = (RightFront_Error - Last_RightFront_Error);
		RightFront_PID = (((RightFront_Kp)*RightFront_Error) + ((RightFront_Ki)*RightFront_Integral) + ((RightFront_Kd)*RightFront_Diff));
		
		if (RightFront_PID > 100)
		{
			RightFront_PWM = 100;
		}
		else if (RightFront_PID < 0)
		{
			RightFront_PWM = 0;
		}
		else 
		{
			RightFront_PWM = (unsigned char)RightFront_PID;
		}

		Last_RightFront_Error = RightFront_Error;
		SetRightFrontWheelPwm(RightFront_PWM);


		/////////////////////////////////////////////////////////////////////////////
		// Right Back Wheel - PID
		/////////////////////////////////////////////////////////////////////////////
		if (RightBack_Frequency_Raw > 3750)
		{
			RightBack_Frequency = 3750;
		}
		else
		{
			RightBack_Frequency = RightBack_Frequency_Raw;
		}

		RightBack_Error = CMD_Right - RightBack_Frequency;
		RightBack_Integral = RightBack_Integral + RightBack_Error;
		
		if (RightBack_Integral > 3750)
		{
			RightBack_Integral = 3750;
		}
		if (RightBack_Integral < 0)
		{
			RightBack_Integral = 0;
		}
		
		RightBack_Diff = (RightBack_Error - Last_RightBack_Error);
		RightBack_PID = (((RightBack_Kp)*RightBack_Error) + ((RightBack_Ki)*RightBack_Integral) + ((RightBack_Kd)*RightBack_Diff));
		
		if (RightBack_PID > 100)
		{
			RightBack_PWM = 100;
		}
		else if (RightBack_PID < 0)
		{
			RightBack_PWM = 0;
		}
		else 
		{
			RightBack_PWM = (unsigned char)RightBack_PID;
		}
		
		Last_RightBack_Error = RightBack_Error;
		SetRightBackWheelPwm(RightBack_PWM);


		/////////////////////////////////////////////////////////////////////////////
	}
}

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
