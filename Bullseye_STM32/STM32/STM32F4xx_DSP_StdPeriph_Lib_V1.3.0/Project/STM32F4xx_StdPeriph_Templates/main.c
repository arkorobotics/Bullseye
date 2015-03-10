/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <math.h>

static unsigned char duty = 0;
static unsigned char rightFrontDuty = 0;
static unsigned char rightBackDuty = 0;
static unsigned char leftFrontDuty = 0;
static unsigned char leftBackDuty = 0;
static unsigned char currDuty = 0;
uint16_t PrescalerValue = 0;
static __IO uint32_t uwTimingDelay;
int straight;
int go = 1;

void init(void);
void PreScale_TIME_Init(void);
void TIM_Config(void);
void TMR3_PWM_Init(void);
void mainInit(void);
static void Delay(__IO uint32_t nTime);
void TIM3_Config(void);

TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
RCC_ClocksTypeDef RCC_Clocks;

extern uint32_t Frequency4;
extern uint32_t Frequency2;
extern uint32_t Frequency5;
extern uint32_t uwTIM1Freq;
extern int RAMPUP;

#define Set_IN2_Left					GPIOE->BSRRL = (1<<4)
#define Clear_IN2_Left				GPIOE->BSRRH = (1<<4)
#define Set_IN1_Left					GPIOE->BSRRL = (1<<2)
#define Clear_IN1_Left				GPIOE->BSRRH = (1<<2)

#define Set_IN2_Right					GPIOE->BSRRL = (1<<3)
#define Clear_IN2_Right				GPIOE->BSRRH = (1<<3)
#define Set_IN1_Right					GPIOE->BSRRL = (1<<5)
#define Clear_IN1_Right				GPIOE->BSRRH = (1<<5)

void mainInit(){
TMR3_PWM_Init();
TIM3_Config();
PreScale_TIME_Init();
}
int setpulse(int k){
		uint16_t p=0;
		return (p+100);
	}

int setpwm(){
		uint16_t p=100;
		return (p*6.65);
	}
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

void SetRightFrontWheelPwm(unsigned char PwmDutyCycle){
//	TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
//	TIM_OC1Init(TIM3, &TIM_OCInitStructure);	
	TIM3->CCR1 = PwmDutyCycle * 4450/100;	
	}
	
void SetRightBackWheelPwm(unsigned char PwmDutyCycle){
//  TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
//	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		TIM3->CCR2 = PwmDutyCycle * 4450/100;
	}
	
void SetLeftFrontWheelPwm(unsigned char PwmDutyCycle){
//  TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
//	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM3->CCR3 = PwmDutyCycle * 4450/100;
	}
	
void SetLeftBackWheelPwm(unsigned char PwmDutyCycle){
//  TIM_OCInitStructure.TIM_Pulse = PwmDutyCycle * TIM_TimeBaseStructure.TIM_Period/100;
//	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
		TIM3->CCR4 = PwmDutyCycle * 4450/100;
	}
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
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_3; //4: IN2  2:IN1
			GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void Test(){
			GPIO_InitTypeDef GPIO_InitStructure; //this
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //power up
			RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIODEN,ENABLE); //enable
	
			GPIO_InitTypeDef  GPIO_InitStruct;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; 
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //4: IN2  2:IN1
			GPIO_Init(GPIOD, &GPIO_InitStructure);
					
			GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
			GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
			GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void Move_Forward(){ //Clockwise Control
//IN2 must be a logic low
	Clear_IN2_Left;
	Clear_IN2_Right;
//IN1 must be a logic high
	Set_IN1_Left;
	Set_IN1_Right;
}
void Move_Backward(){//Counter Clockwise Control
//IN1 must be a logic low
	Clear_IN1_Left;
	Clear_IN1_Right;
//IN2 must be a logic high
	Set_IN2_Left;
	Set_IN2_Right;
}

	void TIM7_Config(){
NVIC_InitTypeDef NVIC_InitStructure;
/* Enable the TIM7 gloabal Interrupt */
NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
 
/* TIM7 clock enable */
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
/* Time base configuration */
TIM_TimeBaseStructure.TIM_Period = 157952; // 1 MHz down to 1 KHz (1 ms)
TIM_TimeBaseStructure.TIM_Prescaler = 0; 
TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
/* TIM IT enable */
TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
/* TIM2 enable counter */
TIM_Cmd(TIM7, ENABLE); 
	}
int main(void){
		// System Tick Handler configured to 100Hz
	SysTick_Config(SystemCoreClock/100);
Test();
while(go){
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){
		RAMPUP=1;
		go = 0;
	straight=1;
		mainInit();
	  DriveInit();
	Move_Forward();
TIM7_Config();
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
  while (1){
	}}}
}

void TIM_Config(void){
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

void Delay(__IO uint32_t nTime){ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

void TimingDelay_Decrement(void){
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
