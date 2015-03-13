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
#include "math.h"
#include "imu.h"
#include "i2c.h"

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
	double Kp = 10;
	double Ki = 0.01;
	double Kd = 0;
unsigned char PWM_Out5;
uint32_t freq5;
double d5, od5 = 0;

int error1 = 0;
int integral1 = 0;
int diff1 = 0;
int last_error1 = 0;
unsigned char PWM_Out1;
uint32_t freq1;
double d1, od1 = 0;

int error2 = 0;
int integral2 = 0;
int diff2 = 0;
int last_error2 = 0;
unsigned char PWM_Out2;
uint32_t freq2;
double d2, od2 = 0;

int error4 = 0;
int integral4 = 0;
int diff4 = 0;
int last_error4 = 0;
unsigned char PWM_Out4;
uint32_t freq4;
double d4, od4 = 0;

double error_theta =0;
double integral_theta = 0;
double diff_theta = 0;
double last_error_theta = 0;
double CMDtheta = 0.0;
double thetaIN = 0;
double theta_calc = 0;
double old_heading = 0.0;

int errorRight = 0;
int integralRight = 0;
int diffRight = 0;
int last_errorRight = 0;

int errorLeft = 0;
int integralLeft = 0;
int diffLeft = 0;
int last_errorLeft = 0;

int pid_out1 = 0;
int pid_out2 = 0;
int pid_out5 = 0;
int pid_out4 = 0;

double cmdDistanceLeft, cmdDistanceRight, cmdODR, cmdODL = 0;
double averageLeft, averageRight;
double r = 0.05; //diameter is 0.1 meters
double t = 0.01;//1kHz => 1ms for timer7 
double mag_heading;

int CMDL = 1500;
int CMDR = 1500;
int CMDLSave = 1500;
int CMDRSave = 1500;
//for testing only below
int count = 0;
int heart = 0;
int pause = 101;
int BACK = 0;
int FORWARD = 1;
int STOP= 0;
int RAMPUP = 0;
int RAMPDOWN = 0;
int i = 0;
int i2c_count = 0;
int i2c_start_count = 0;

#define Set					GPIOD->BSRRL = (1<<15)
#define Clear				GPIOD->BSRRH = (1<<15)

extern void Move_Backward();
extern void Move_Forward();
extern int straight;
extern int go;
extern double start_heading;
extern double start_mag_heading;
double start_mag_heading = 0;
extern void SetLeftFrontWheelPwm(int);
extern void SetLeftBackWheelPwm(int);
extern void SetRightFrontWheelPwm(int);
extern void SetRightBackWheelPwm(int);
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

void Clear_All(void)
{
			CMDL=0; CMDR=0;
			Frequency4 = 0; freq4 = 0;
			Frequency5 = 0; freq5 = 0;
			Frequency2 = 0; freq2 = 0;
			uwTIM1Freq = 0; freq1 = 0;
			od5 = 0; od4 = 0; od2 = 0; od1 = 1; d1 = 0; d2 = 0; d4 = 0; d5 = 0;
			cmdODL = 0; cmdODR = 0; cmdDistanceLeft = 0; cmdDistanceRight = 0;
			averageLeft = 0; averageRight = 0;
}
void Clear_CMD_Freq()
	{
			CMDL=0; CMDR=0;
			Frequency4 = 0; freq4 = 0;
			Frequency5 = 0; freq5 = 0;
			Frequency2 = 0; freq2 = 0;
			uwTIM1Freq = 0; freq1 = 0;
	}
void Clear_Distance()
	{
			od5 = 0; od4 = 0; od2 = 0; od1 = 1; d1 = 0; d2 = 0; d4 = 0; d5 = 0;
			cmdODL = 0; cmdODR = 0; cmdDistanceLeft = 0; cmdDistanceRight = 0;
			averageLeft = 0; averageRight = 0;
	}
void HeartBeat()
	{
		count = count + 1;
		if(count == 100){
		count = 0; heart = !heart;
		if(heart==1){Set;} 
		else if(heart==0){Clear;}
		}
	}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */	
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
	//Distance Calculations for all Motors
	cmdDistanceLeft = (2*(3.141592657)*r*CMDL*t)/(48*34); //1632 divide the velocity by 48*34
	cmdODL = cmdODL + cmdDistanceLeft;
	cmdDistanceRight = (2*(3.141592657)*r*CMDR*t)/(48*34);
	cmdODR = cmdODR + cmdDistanceRight;

	d5 = (2*(3.141592657)*r*freq5*t)/(1632);//Front Left
	od5 = od5 +d5;
	d1 = (2*(3.141592657)*r*freq1*t)/(1632);//Back Left
	od1 = od1 +d1;
	averageLeft = (od5+od1)/2;

	d2 = (2*(3.141592657)*r*freq2*t)/(1632);//Back Right
	od2 = od2 +d2;
	d4 = (2*(3.141592657)*r*freq4*t)/(1632);//Front Right
	od4 = od4 +d4;
	averageRight = (od4+od2)/2;

//	if(FORWARD)
//	{
//	if ((averageLeft>15)&&(averageLeft<15.1))
//		{
//			RAMPDOWN=1;
//		}
//	}
//	pause++;
//	if(pause==20)
//		{
//			FORWARD=0;
//			BACK=1;
//			Clear_All();
//			Move_Backward();
//			RAMPUP=1;
//			CMDL = 1000;
//			CMDR = 1000;
//			Clear_Distance();
//		}
//		if(BACK)
//			{
//				if ((averageLeft>15.5)&&(averageLeft<15.6))
//					{
//						RAMPDOWN = 1;
//						STOP = 1;
//						BACK = 0;
//					}
//			}
//		if(RAMPUP)
//				{
//					CMDL = CMDL + 2;
//					CMDR = CMDR + 2;
//					if(CMDR==CMDRSave)
//					{
//						RAMPUP=0;
//					}
//				}
//		if(RAMPDOWN)
//				{
//					CMDL = CMDL - 5;
//					CMDR = CMDR - 5;
//					if(CMDR==0)
//					{
//						Clear_All();
//						RAMPDOWN=0;
//						pause = 0;
//						if(STOP){pause=101;}
//					}
//				}
	HeartBeat();
	if(i<1000)
	{
		if(i2c_start_count < 100)
		{
			i2c_start_count++;
		}
		else
		{
			i2c_start_count = 0;
			IMU_Update();
			CMDtheta = ((CMDtheta + (gyro[2]*(0.01))) / 2);
			start_heading = ((start_heading + gyro[2])/2);
			start_mag_heading = ((start_mag_heading + atan2(mag[0]/100,mag[1]/100)*(180/3.141592654))/2);
		}
			i++;
	}
	if(go==0){
		i2c_count++;
		if((gyro[2]>-3.0)&&(gyro[2]<-1.9))
		{thetaIN = 0;}
		else {thetaIN = gyro[2]- start_heading;}
	if(i2c_count==100)
		{
			IMU_Update();
			//mag_heading = atan2(mag[0],mag[1])*(180/3.141592654);
			mag_heading = atan2(mag[0]/100,mag[1]/100)*(180/3.141592654) - start_mag_heading;
			
			heading = (heading + (thetaIN*0.1));
			i2c_count = 0;
	}
}
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
	
//////////////////////////////// IMU PID /////////////////////////////////////
	error_theta = CMDtheta - heading;
	integral_theta = integral_theta + error_theta;
	diff_theta = error_theta - last_error_theta;
	theta_calc = (Kp * error_theta) + (Ki * integral_theta) + (Kd * diff_theta);
		CMDL = CMDLSave - theta_calc;
		CMDR = CMDRSave + theta_calc;
		if(CMDL>2000){CMDL = 2000;}
		if(CMDR>2000){CMDR = 2000;}
		if(CMDL<0){CMDL = 0;}
		if(CMDR<0){CMDR = 0;}
		last_error_theta = error_theta;
	
////////////////////////////////////////////////////////////////////
  if(Frequency5>3750)
	{freq5 = 3750;}
	else {freq5 = Frequency5;}
	
	error5 = CMDL - freq5; //-1050
	integral5 = integral5 + error5;//-1050
	if (integral5> 3750)
	{integral5 = 3750;}
		if (integral5<0)
	{integral5 = 0;}
	diff5 = (error5 - last_error5);//-1050
	pid_out5 = (((1.5)*error5)+((0.0005)*integral5) + ((0.0005)*diff5));
	if (pid_out5>100)
		{PWM_Out5=100;}
	else if (pid_out5<0)
		{PWM_Out5=0;}
		else {PWM_Out5=(unsigned char)pid_out5;}
	last_error5 = error5;
SetLeftFrontWheelPwm(PWM_Out5);
		
/////////////////////////////////////////////////////////////////////////////
		
  if(uwTIM1Freq>3750)
	{freq1 = 3750;}
	else {freq1 = uwTIM1Freq;}
	
	error1 = CMDL - freq1; //-1050
	integral1 = integral1 + error1;//-1050
	if (integral1> 3750)
	{integral1 = 3750;}
		if (integral1<0)
	{integral1 = 0;}
	diff1 = (error1 - last_error1);//-1050
	pid_out1 = (((1)*error1)+((0.0005)*integral1) + ((0.0001)*diff1));
	//pid_out1 = (((0.2782)*error1)+((0.00365)*integral1) + ((0.00097)*diff1));
	if (pid_out1>100)
		{PWM_Out1=100;}
	else if (pid_out1<0)
		{PWM_Out1=0;}
		else {PWM_Out1 = (unsigned char)pid_out1;}
	last_error1 = error1;
SetLeftBackWheelPwm(PWM_Out1);
		
/////////////////////////////////////////////////////////////////////////////
		
  if(Frequency2>3750)
	{freq2 = 3750;}
	else {freq2 = Frequency2;}
	
	error2 = CMDR - freq2; //-1050
	integral2 = integral2 + error2;//-1050
	if (integral2> 3750)
	{integral2 = 3750;}
		if (integral2<0)
	{integral2 = 0;}
	diff2 = (error2 - last_error2);//-1050
	pid_out2 = (((1.5)*error2)+((0.0005)*integral2) + ((0.0001)*diff2));
	if (pid_out2>100)
		{PWM_Out2=100;}
	else if (pid_out2<0)
		{PWM_Out2=0;}
		else {PWM_Out2 = (unsigned char)pid_out2;}
	last_error2 = error2;
SetRightBackWheelPwm(PWM_Out2);
		
/////////////////////////////////////////////////////////////////////////////
		
  if(Frequency4>3750)
	{freq4 = 3750;}
	else {freq4 = Frequency4;}
	
	error4 = CMDR - freq4; //-1050
	integral4 = integral4 + error4;//-1050
	if (integral4> 3750)
	{integral4 = 3750;}
		if (integral4<0)
	{integral4 = 0;}
	diff4 = (error4 - last_error4);//-1050
	pid_out4 = (((1.5)*error4)+((0.0005)*integral4) + ((0.0001)*diff4));
	if (pid_out4>100)
		{PWM_Out4=100;}
	else if (pid_out4<0)
		{PWM_Out4=0;}
		else {PWM_Out4 = (unsigned char)pid_out4;}
	last_error4 = error4;
SetRightFrontWheelPwm(PWM_Out4);
	}
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
