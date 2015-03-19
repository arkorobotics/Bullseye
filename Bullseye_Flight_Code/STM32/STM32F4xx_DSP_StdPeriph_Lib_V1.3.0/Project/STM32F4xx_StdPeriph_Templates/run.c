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
// RUN.C
// --------------------------------------------------------------------
#include "stm32f4xx.h"
#include "run.h"

uint32_t rodeo_count = 0;

void RunSwitchInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
