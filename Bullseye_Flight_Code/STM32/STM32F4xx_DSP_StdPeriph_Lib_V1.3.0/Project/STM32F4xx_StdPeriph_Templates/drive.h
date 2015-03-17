/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_H
#define DRIVE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/////////////////////////////////////////////////////////////////////////////
// Drive - Variables, Defines, & Functions
/////////////////////////////////////////////////////////////////////////////

extern TIM_ICInitTypeDef					TIM_ICInitStructure;
extern TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
extern TIM_OCInitTypeDef					TIM_OCInitStructure;
extern RCC_ClocksTypeDef					RCC_Clocks;

extern uint32_t LeftFront_Frequency_Raw;
extern uint32_t LeftBack_Frequency_Raw;
extern uint32_t RightFront_Frequency_Raw;
extern uint32_t RightBack_Frequency_Raw;

extern int CMD_Left;
extern int CMD_Right;

extern int CMD_Left_Target;
extern int CMD_Right_Target;

extern double CMD_Distance;
extern double AverageDistanceTravelled;

extern uint8_t run;

#define Set_IN1_Left				GPIOE->BSRRL = (1<<2)
#define Clear_IN1_Left				GPIOE->BSRRH = (1<<2)
#define Set_IN2_Left				GPIOE->BSRRL = (1<<4)
#define Clear_IN2_Left				GPIOE->BSRRH = (1<<4)

#define Set_IN1_Right				GPIOE->BSRRL = (1<<5)
#define Clear_IN1_Right				GPIOE->BSRRH = (1<<5)
#define Set_IN2_Right				GPIOE->BSRRL = (1<<3)
#define Clear_IN2_Right				GPIOE->BSRRH = (1<<3)

void TIM3_Config(void);
void TMR3_PWM_Init(void);
void TIM3_Pin_Init(void);
void PreScale_TIME_Init(void);
void TIM7_Config(void);
void TIM_Config(void);

void DriveInit(void);
void SetDirection_Forward(void);
void SetDirection_Backward(void);
void SetRotation_Left(void);
void SetRotation_Right(void);
void SetRightFrontWheelPwm(unsigned char);
void SetRightBackWheelPwm(unsigned char);
void SetLeftFrontWheelPwm(unsigned char);
void SetLeftBackWheelPwm(unsigned char);
/////////////////////////////////////////////////////////////////////////////

#endif /* DRIVE_H */

/*****************************END OF FILE**************************************/
