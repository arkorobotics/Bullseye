#include "stm32f4xx.h"

struct ErrorControl5
	{
	int error5 = 0;
int integral5 = 0;
int diff5 = 0;
int last_error5 = 0;
double Kp = 0.01;
double Ki = 0.05;
double Kd = 0.1;
unsigned char PWM_Out5;
int feedback = 0;
uint32_t freq5;
};