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
#include "imu.h"
#include "sonar.h"
#include "i2c.h"
#include "speaker.h"
#include "run.h"

/////////////////////////////////////////////////////////////////////////////
// Theta - Variables
/////////////////////////////////////////////////////////////////////////////
double Theta_Kp = 10.000;
double Theta_Ki = 0.01;
double Theta_Kd = 0.000;
double Theta_Error = 0;
double Theta_Integral = 0;
double Theta_Diff = 0;
double Theta_PID = 0;
double Last_Theta_Error = 0;


/////////////////////////////////////////////////////////////////////////////
// Left Front Wheel - Variables
/////////////////////////////////////////////////////////////////////////////
__IO uint16_t IC5Value = 0;
__IO uint16_t DutyCycle5 = 0;
__IO uint32_t LeftFront_Frequency_Raw = 0;
double LeftFront_Kp = 1.0000;
double LeftFront_Ki = 0.0001;
double LeftFront_Kd = 0.0001;
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
double LeftBack_Kp = 1.0000;
double LeftBack_Ki = 0.0005;
double LeftBack_Kd = 0.0001;
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
double RightFront_Kp = 1.5000;
double RightFront_Ki = 0.0005;
double RightFront_Kd = 0.0001;
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
double RightBack_Kp = 1.5000;
double RightBack_Ki = 0.0005;
double RightBack_Kd = 0.0001;
int RightBack_Error = 0;
int RightBack_Integral = 0;
int RightBack_Diff = 0;
int RightBack_PID = 0;
int Last_RightBack_Error = 0;
unsigned char RightBack_PWM;
uint32_t RightBack_Frequency;



/////////////////////////////////////////////////////////////////////////////
// Wheel - Variables
/////////////////////////////////////////////////////////////////////////////
int CMD_Left = 0;
int CMD_Right = 0;

int CMD_Left_Target = 0;
int CMD_Right_Target = 0;

double CMD_Theta = 0.000;

double CMD_Distance = 0;

double CMD_DistanceLeft = 0;
double CMD_DistanceRight = 0;
double CMD_ODR = 0;
double CMD_ODL = 0;

double LF_DeltaDistance = 0;
double LF_Distance = 0;
double RF_DeltaDistance = 0;
double RF_Distance = 0;
double LB_DeltaDistance = 0;
double LB_Distance = 0;
double RB_DeltaDistance = 0;
double RB_Distance = 0;

double AverageDistance_Left = 0; 
double AverageDistance_Right = 0;
double AverageDistanceTravelled = 0;

double r = 0.05; 					// Wheel radius: 0.1 meters
double dt = 0.1;					// dt: 1kHz => 1ms for timer7

int heart_count = 0;
int heart_led = 0;
#define Set_LED				GPIOD->BSRRL = (1<<15)
#define Clear_LED			GPIOD->BSRRH = (1<<15)


extern void SetLeftFrontWheelPwm(int);
extern void SetLeftBackWheelPwm(int);
extern void SetRightFrontWheelPwm(int);
extern void SetRightBackWheelPwm(int);


/////////////////////////////////////////////////////////////////////////////
// Sonar - Variables
/////////////////////////////////////////////////////////////////////////////
int sonar_read_delay = 20;
int sonar_read_count = 0;

/////////////////////////////////////////////////////////////////////////////
// IMU - Variables
/////////////////////////////////////////////////////////////////////////////
int imu_read_delay = 10;
int imu_read_count = 0;

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

		Theta_Error = CMD_Theta - heading;
		Theta_Integral = Theta_Integral + Theta_Error;
		Theta_Diff = Theta_Error - Last_Theta_Error;
		Theta_PID = (Theta_Kp * Theta_Error) + (Theta_Ki * Theta_Integral) + (Theta_Kd * Theta_Diff);

		if(run == 0)
		{
			Theta_PID = 0;
		}

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

		LeftFront_Error = CMD_Left - LeftFront_Frequency - (signed int)(Theta_PID);
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

		LeftBack_Error = CMD_Left - LeftBack_Frequency - (signed int)(Theta_PID);
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

		RightFront_Error = CMD_Right - RightFront_Frequency + (signed int)(Theta_PID);
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

		RightBack_Error = CMD_Right - RightBack_Frequency + (signed int)(Theta_PID);
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
		// Odometry
		/////////////////////////////////////////////////////////////////////////////
		//Distance Calculations for all Motors
		// Note: 1632 divide the velocity by 48*34	
		LF_DeltaDistance = 2*(3.141592657)*r*(LeftFront_Frequency/(48*34))*dt;		//Left Front
		LF_Distance = LF_Distance + LF_DeltaDistance;
		RF_DeltaDistance = 2*(3.141592657)*r*(LeftBack_Frequency/(48*34))*dt;		//Left Back
		RF_Distance = RF_Distance + RF_DeltaDistance;
		AverageDistance_Left = (LF_Distance + RF_Distance)/2;
		
		LB_DeltaDistance = 2*(3.141592657)*r*(RightFront_Frequency/(48*34))*dt;		//Right Front
		LB_Distance = LB_Distance + LB_DeltaDistance;
		RB_DeltaDistance = 2*(3.141592657)*r*(RightBack_Frequency/(48*34))*dt;		//Right Back
		RB_Distance = RB_Distance + RB_DeltaDistance;
		AverageDistance_Right = (LB_Distance + RB_Distance)/2;

		AverageDistanceTravelled = (AverageDistance_Left + AverageDistance_Right)/2;
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
	// System Tick Handler fires at 100Hz
	TimingDelay_Decrement();
	

	/////////////////////////////////////////////////////////////////////////////
	// Update IMU Accel, Gyro, Mag values
	/////////////////////////////////////////////////////////////////////////////
	if(imu_read_count < imu_read_delay)
	{
		imu_read_count++;
	}
	else
	{
		IMU_Update();
		heading = heading + (gyro[2] - gyro_z_cal)*dt;
		imu_read_count = 0;
	}


	/////////////////////////////////////////////////////////////////////////////
	// Update Sonar
	/////////////////////////////////////////////////////////////////////////////
	if(sonar_read_count < sonar_read_delay)
	{
		sonar_read_count++;
	}
	else
	{
		I2cSonarRead(0x52, sonarBuffer, 2);
		sonar_read_count = 0;
	}
	

	/////////////////////////////////////////////////////////////////////////////
	// Heartbeat
	/////////////////////////////////////////////////////////////////////////////
	heart_count = heart_count + 1;
	if(heart_count == 1000)
	{
		heart_count = 0; 
		heart_led = !heart_led;
	}
	if(heart_led==1)
	{
		Set_LED;
	} 
	else if(heart_led==0)
	{
		Clear_LED;
	}
	
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f40xx.s/startup_stm32f427x.s).                         */
/******************************************************************************/

/**
  * @brief  This function handles External line 15 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  { 
    /* Change the wave */
    ubKeyPressed = 1;

    /* Change the selected waves forms */
    ubSelectedWavesForm = !ubSelectedWavesForm;

    /* Clear the Right Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}
