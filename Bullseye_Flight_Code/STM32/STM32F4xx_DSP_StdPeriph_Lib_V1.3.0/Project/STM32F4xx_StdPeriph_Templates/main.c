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
// MAIN
// --------------------------------------------------------------------


// --------------------------------------------------------------------
// Timer Allocation
// --------------------------------------------------------------------
//
//	Timer 1: Left Back Wheel Encoder Interrupt
//	Timer 2: Right Back Wheel Encoder Interrupt
//	Timer 3: PWM Output Timer
//	Timer 4: Right Front Wheel Encoder Interrupt
//	Timer 5: Left Front Wheel Encoder Interrupt
//	Timer 6:
//	Timer 7: PID Loop Interrupt
//
// --------------------------------------------------------------------

#include "stm32f4xx.h"
#include "drive.h"
#include "imu.h"
#include "i2c.h"
#include "sonar.h"
#include "speaker.h"
#include "run.h"

#include <math.h>
#include <string.h>


static __IO uint32_t uwTimingDelay;
static void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);


uint8_t run = 0;
uint8_t gyro_enable = 0; 
uint8_t flip_status = 0;

typedef enum { TUG_OF_WAR, RELAY_RACE, OBSTACLE_RACE, MOO, ROUND_UP, WAYPOINT } state_mode;

/////////////////////////////////////////////////////////////////////////////
// CURRENT MODE
/////////////////////////////////////////////////////////////////////////////
state_mode current_state = RELAY_RACE;
/////////////////////////////////////////////////////////////////////////////

int main(void)
{	
	RunSwitchInit();

	I2CInit();

	IMU_Init();

	CalibrateIMU();

	Speaker_Config();

	// System Tick Handler configured to 100Hz
	SysTick_Config(SystemCoreClock/100);

	DriveInit();				// Initialize GPIO Pins
	SetDirection_Forward();		// Set initial direction to forward

	TIM3_Config();			// Configure Timer 3: PWM 
	TIM7_Config();			// Configure Timer 7: PID Timer Loop
	TIM_Config();			// Configure Timers 1,2,4,5: Frequency input from encoders


	// Initialize target motor speed to 0
	CMD_Left = 0;
	CMD_Right = 0;

	// Initialize sonar buffer to smallest possible reading
	sonarBuffer[0] = 1;
	sonarBuffer[1] = 1;

	// Staging to start, awaits "run" switch
	while (run == 0)
	{
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1)
		{
			run = 1;
			gyro_enable = 1;
		}
	}

	while (run)
	{
		// Run the state machine
		switch(current_state)
		{
			case TUG_OF_WAR:
				SetDirection_Forward();
				Delay(10);
				break;
			case RELAY_RACE:
				
				
				if(flip_status == 0)
				{
					if(0.0 <= AverageDistance_Left && AverageDistance_Left < 10.0)
					{
						SetDirection_Forward();
						SetLeftFrontWheelPwm(60);
						SetLeftBackWheelPwm(60);
						SetRightFrontWheelPwm(60);
						SetRightBackWheelPwm(60);
						CMD_Left = 2400;
						CMD_Right = 2400;
						CMD_Theta = 0.00;
					}					
					else if(10.0 <= AverageDistance_Left && AverageDistance_Left < 60.0)
					{
						SetDirection_Forward();
						CMD_Left = 1510;
						CMD_Right = 1500;
						//gyro_enable = 1;
						CMD_Theta = 0.00;
					}
					else if(60.0 <= AverageDistance_Left && AverageDistance_Left < 120.0)
					{
						//gyro_enable = 0;
						CMD_Theta = 0.00;
					}
					else if(120.0 <= AverageDistance_Left && AverageDistance_Left < 435.0)
					{
						//gyro_enable = 1;
						CMD_Theta = 0.50; 
					}
					else if(435.0 <= AverageDistance_Left)
					{
						//gyro_enable = 1;
						CMD_Left = 0;
						CMD_Right = 0;
						SetDirection_Backward();
						Delay(20);					
						CMD_Left = 1510;
						CMD_Right = 1500;
						CMD_Theta = 0.00; 
						flip_status = 1;
					}
				}
				else if(flip_status == 1)
				{
					if(AverageDistance_Left < 1200.0)
					{
						//gyro_enable = 1;
						SetDirection_Backward();
						CMD_Theta = 1.00; 
					}
					else if(1200.0 <= AverageDistance_Left)
					{
						//gyro_enable = 1;
						SetDirection_Forward();
						CMD_Left = 0;
						CMD_Right = 0;
						CMD_Theta = 1.00; 
					}
				}
				
				break;
			case OBSTACLE_RACE:
				SetDirection_Forward();
				/*
				if(sonarBuffer[0] < 100)
				{
					CMD_Left = 1500*(sonarBuffer[0]/100);
					CMD_Right = 1500*(sonarBuffer[0]/100);
				}
				else
				{
					CMD_Left = 1500;
					CMD_Right = 1500;
				}
				*/
				break;
			case MOO:
				Moo();
				break;
			case ROUND_UP:

				break;
			case WAYPOINT:

				break;
			default:
				// Turn off motors
				CMD_Left = 0;
				CMD_Right = 0;
				break;
		}
	}
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

