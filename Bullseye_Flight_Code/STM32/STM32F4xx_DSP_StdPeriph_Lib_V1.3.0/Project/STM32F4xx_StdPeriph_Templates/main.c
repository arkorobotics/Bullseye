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
#include "speaker.h"

#include <math.h>
#include <string.h>


static __IO uint32_t uwTimingDelay;
static void Delay(__IO uint32_t nTime);
void TimingDelay_Decrement(void);

uint8_t sonarBuffer[2];

typedef enum { TUG_OF_WAR, RELAY_RACE, OBSTACLE_RACE, MOO, ROUND_UP, WAYPOINT } state_mode;

/////////////////////////////////////////////////////////////////////////////
// CURRENT MODE
/////////////////////////////////////////////////////////////////////////////
state_mode current_state = OBSTACLE_RACE;
/////////////////////////////////////////////////////////////////////////////

int main(void)
{
	DriveInit();				// Initialize GPIO Pins
	SetDirection_Forward();		// Set initial direction to forward

	TIM3_Config();			// Configure Timer 3: PWM 
	TIM7_Config();			// Configure Timer 7: PID Timer Loop
	TIM_Config();			// Configure Timers 1,2,4,5: Frequency input from encoders

	// System Tick Handler configured to 100Hz
	SysTick_Config(SystemCoreClock/100);

	//IMU_Init();
	//Speaker_Config();
	I2CInit();
	
	CMD_Left = 0;
	CMD_Right = 0;

	while (1)
	{
		// Run the state machine
		switch(current_state)
		{
			case TUG_OF_WAR:
				SetDirection_Forward();
				Delay(10);
				break;
			case RELAY_RACE:

				break;
			case OBSTACLE_RACE:
				SetDirection_Forward();
			
				//CMD_Left = 0;
				//CMD_Right = 0;
			
				I2cSonarRead(0x52, sonarBuffer, 2);
			
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
				
				Delay(20);
				
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

