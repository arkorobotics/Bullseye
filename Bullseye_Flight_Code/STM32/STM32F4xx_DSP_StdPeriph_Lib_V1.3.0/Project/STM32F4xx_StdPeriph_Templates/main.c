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

uint32_t flip_count = 0;
uint8_t flip_status = 0;

typedef enum { TUG_OF_WAR, RELAY_RACE, OBSTACLE_RACE, MOO, ROUND_UP, WAYPOINT } state_mode;

/////////////////////////////////////////////////////////////////////////////
// CURRENT MODE
/////////////////////////////////////////////////////////////////////////////
state_mode current_state = ROUND_UP;  // Comment out sonar when not used
/////////////////////////////////////////////////////////////////////////////

int main(void)
{	
	RunSwitchInit();

	I2CInit();

	IMU_Init();

	CalibrateIMU();

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
				CMD_Left = 3600;
				CMD_Right = 3600;
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
						CMD_Left = 2400;
						CMD_Right = 2400;
						//gyro_enable = 1;
						CMD_Theta = 0.00;
					}
					else if(60.0 <= AverageDistance_Left && AverageDistance_Left < 260.0)
					{
						//gyro_enable = 0;
						CMD_Theta = 0.00;
					}
					else if(260.0 <= AverageDistance_Left && AverageDistance_Left < 635.0)
					{
						//gyro_enable = 1;
						CMD_Theta = 0.00; 
					}
					else if(635.0 <= AverageDistance_Left)
					{
						//gyro_enable = 1;
						CMD_Left = 0;
						CMD_Right = 0;
						SetDirection_Backward();
						
						// Delay 1
						flip_count = 0;
						while(flip_count < 1000000)
						{
							flip_count++;
						}
						
						SetLeftFrontWheelPwm(60);
						SetLeftBackWheelPwm(60);
						SetRightFrontWheelPwm(60);
						SetRightBackWheelPwm(60);				
						
						// Delay 1
						flip_count = 0;
						while(flip_count < 1000000)
						{
							flip_count++;
						}
						
						CMD_Left = 2400;
						CMD_Right = 2400;
						CMD_Theta = 0.00; 
						flip_status = 1;
					}
				}
				else if(flip_status == 1)
				{
					if(AverageDistance_Left < 1400.0)
					{
						//gyro_enable = 1;
						SetDirection_Backward();
						CMD_Theta = 0.00; 
					}
					else if(1400.0 <= AverageDistance_Left)
					{
						//gyro_enable = 1;
						SetDirection_Forward();
						CMD_Left = 0;
						CMD_Right = 0;
						CMD_Theta = 0.00; 
					}
				}
				break;
			case OBSTACLE_RACE:
				SetDirection_Forward();
				// Use Sabrina's Code
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
				while(rodeo_count < 100)
				{
						obstacle_front_flag = 0;
						SetDirection_Forward();
						CMD_Left = 1800;
						CMD_Right = 1500;
					
				}
				CMD_Left = 0;
				CMD_Right = 0;
				Speaker_Config();
				Moo_ON();
				Delay(66);
				Moo_OFF();
				Delay(33);
				break;
			case ROUND_UP:
				// Go in spirals and do 180's around things in front
				CMD_Left = 2600;
				CMD_Right = 2600;
				
				// Delay 1
				flip_count = 0;
				while(flip_count < 4000000)
				{
					flip_count++;
				}
				
				CMD_Left = 2600;
				CMD_Right = 1500;
				
				// Delay 2
				flip_count = 0;
				while(flip_count < 150000)
				{
					flip_count++;
				}
			
				break;
			case WAYPOINT:
				SetDirection_Forward();
				if(sonarBuffer[0] <= 55)
				{
					CMD_Left = 3600;
					CMD_Right = 1700;
				}
				else if(sonarBuffer[0] <= 20)
				{
					CMD_Left = 0;
					CMD_Right = 0;
				}
				else
				{
					CMD_Left = 3400;
					CMD_Right = 1700;
				}
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
