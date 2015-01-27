/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

TIM_ICInitTypeDef  TIM_ICInitStructure;
static unsigned char duty = 0;
static unsigned char rightFrontDuty = 0;
static unsigned char rightBackDuty = 0;
static unsigned char leftFrontDuty = 0;
static unsigned char leftBackDuty = 0;
static unsigned char currDuty = 0;
void init(void);
void PreScale_TIME_Init(void);
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t PrescalerValue = 0;
void TIM_Config(void);
void TMR3_PWM_Init(void);
void mainInit(void);
extern uint32_t Frequency4;
extern uint32_t Frequency2;
extern uint32_t Frequency5;
extern uint32_t uwTIM1Freq;

#define Set_IN2					GPIOE->BSRRL = (1<<4)
#define Clear_IN2				GPIOE->BSRRH = (1<<4)
#define Set_IN1					GPIOE->BSRRL = (1<<2)
#define Clear_IN1				GPIOE->BSRRH = (1<<2)

static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;
static void Delay(__IO uint32_t nTime);
void TIM3_Config(void);
///////////////////////////////////////////////////////////////////////////////
/*-----------------------------Main Init-------------------------------------*/
void mainInit(){
TMR3_PWM_Init();
TIM3_Config();
PreScale_TIME_Init();
}
///////////////////////////////////////////////////////////////////////////////
/*-------------------------------PWM-----------------------------------------*/
/*******************************PWM Code*********************************/
int setpulse(int k){
		uint16_t p=0;
		return (p+100);
	}

int setpwm(){
		uint16_t p=100;
		return (p*6.65);
	}
	
/******************************************************************************
* TIM_PWMOutput code from template examples, specs given in folder
******************************************************************************/

void TMR3_PWM_Init(void){
	 /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 21000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 4450;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 333;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 333;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 333;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 333;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}


/******************************************************************************
* TIM_PWMOutput code from template examples, specs given in folder
******************************************************************************/
void TIM3_Config(void){
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  /* GPIOC Configuration: TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8) and TIM3 CH4 (PC9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3); 
}

void SetRightFrontWheelPwm(unsigned char PwmDutyCycle){//only sets Oc1[Pc6]                        currently using C6 need to add C7 C8
	TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);																
	}
	
	void SetRightBackWheelPwm(unsigned char PwmDutyCycle){
  TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	}
	
		void SetLeftFrontWheelPwm(unsigned char PwmDutyCycle){
  TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	}
	
		void SetLeftBackWheelPwm(unsigned char PwmDutyCycle){
  TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	}
	//A15, B3, B10, B11	
	void PreScale_TIME_Init(void){
		/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 21000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 4450;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;					//try this
  TIM_OCInitStructure.TIM_Pulse = 333;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = setpwm()/2;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = setpwm()/4;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = setpwm()/10;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void DriveInit(){
				GPIO_InitTypeDef GPIO_InitStructure; //this
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); //power up
			RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOEEN,ENABLE); //enable
			
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_2; //4: IN2  2:IN1
			GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void Move_Forward(){ //Clockwise Control
//IN2 must be a logic low
	Clear_IN2;
//IN1 must be a logic high
	Set_IN1;
}
void Move_Backward(){//Counter Clockwise Control
//IN1 must be a logic low
	Clear_IN1;
//IN2 must be a logic high
	Set_IN2;
}

void Forward_Straight(unsigned char forwardDuty){
Move_Forward(); 
  SetRightFrontWheelPwm(forwardDuty);
	SetRightBackWheelPwm(forwardDuty);
	SetLeftFrontWheelPwm(forwardDuty);
	SetLeftBackWheelPwm(forwardDuty);
}
void Backward_Straight(unsigned char backwardDuty){
Move_Backward(); 
  SetRightFrontWheelPwm(backwardDuty);
	SetRightBackWheelPwm(backwardDuty);
	SetLeftFrontWheelPwm(backwardDuty);
	SetLeftBackWheelPwm(backwardDuty);
}
	
void FrontRightErrorControl(){  //Timer 4
uint32_t error4;
uint32_t integral4;
uint32_t diff4;
uint32_t last_error4;
unsigned char PWM_Out4;

//	error4 = ? - Frequency4;
//	integral4 = integral4 + error4;
//	if (integral4> ?)
//		integral4 = ?;
//	diff4 = (error4 - last_error4);
//	PWM_Out4 = [((Kp)*error4)+((Ki)*integral4) + ((Kd)*diff4)];
//	last_error4 = error4;
	
rightFrontDuty = PWM_Out4;
}
void FrontLeftErrorControl(){  //Timer 5
uint32_t error5;
uint32_t integral5;
uint32_t diff5;
uint32_t last_error5;
unsigned char PWM_Out5;

//	error5 = ? - Frequency5;
//	integral5 = integral5 + error5;
//	if (integral5> ?)
//		integral5 = ?;
//	diff5 = (error5 - last_error5);
//	PWM_Out5 = [((Kp)*error5)+((Ki)*integral5) + ((Kd)*diff5)];
//	last_error5 = error5;
	
leftFrontDuty = PWM_Out5;
}
void BackRightErrorControl(){  //Timer 2
uint32_t error2;
uint32_t integral2;
uint32_t diff2;
uint32_t last_error2;
unsigned char PWM_Out2;

//	error2 = ? - Frequency2;
//	integral2 = integral2 + error2;
//	if (integral2> ?)
//		integral2 = ?;
//	diff2 = (error2 - last_error2);
//	PWM_Out2 = [((Kp)*error2)+((Ki)*integral2) + ((Kd)*diff2)];
//	last_error2 = error2;
rightBackDuty = PWM_Out2;
}
void BackLeftErrorControl(){  //Timer 1
uint32_t error1;
uint32_t integral1;
uint32_t diff1;
uint32_t last_error1;
unsigned char PWM_Out1;

//	error1 = ? - uwTIM1Freq;
//	integral1 = integral1 + error1;
//	if (integral1> ?)
//		integral1 = ?;
//	diff1 = (error1 - last_error1);
//	PWM_Out1 = [((Kp)*error1)+((Ki)*integral1) + ((Kd)*diff1)];
//	last_error1 = error1;
leftBackDuty=PWM_Out1;
}

int main(void)
{
		mainInit();
	  DriveInit();

//try a couple test functions 
	duty=50;
	Forward_Straight(duty);
	Backward_Straight(duty);

TIM_Config();	
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM5, &TIM_ICInitStructure);
	TIM_PWMIConfig(TIM2, &TIM_ICInitStructure);
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure);

  /* Select the TIM4 Input Trigger: TI2FP2 */
  TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2);
	TIM_SelectInputTrigger(TIM5, TIM_TS_TI2FP2);
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM1, &TIM_ICInitStructure);

  /* Select the slave Mode: Reset Mode */
  TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM4,TIM_MasterSlaveMode_Enable);
	TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM5,TIM_MasterSlaveMode_Enable);
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM2,TIM_MasterSlaveMode_Enable);

  /* TIM enable counter */
  TIM_Cmd(TIM4, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

  /* Enable the CC2 Interrupt Request */
   TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	 TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);	
   TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
   TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);	
	 
/*Error control must be constant*/
  while (1){
		FrontLeftErrorControl();
		SetLeftFrontWheelPwm(leftFrontDuty);
		FrontRightErrorControl();
		SetRightFrontWheelPwm(rightFrontDuty);
		BackLeftErrorControl();
		SetRightBackWheelPwm(rightBackDuty);
		BackRightErrorControl();
		SetLeftBackWheelPwm(leftBackDuty);
	}
}

void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
  /* TIM4 chennel2 configuration : PB.07 */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  /* Connect TIM pin to AF2 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

  /* Enable the TIM4 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
}

void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
