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
// SPEAKER.C
// --------------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include <string.h>
#include "speaker.h"

/////////////////////////////////////////////////////////////////////////////
// Audio - Variables, Defines, & Functions
/////////////////////////////////////////////////////////////////////////////
DAC_InitTypeDef  			DAC_InitStructure;

uint16_t aSine12bit[32] = {
                       0, 0, 0, 0, 0, 0, 0, 0, 
										   0, 0, 0, 0, 0, 0, 0, 0,
										   0, 0, 0, 0, 0, 0, 0, 0, 
										   0, 0, 0, 0, 0, 0, 0, 0};
											
__IO uint8_t ubSelectedWavesForm = 1;
__IO uint8_t ubKeyPressed = SET; 

uint8_t obstacle_front_flag = 0;
											 
void Speaker_Config(void)
{
	SpeakerGPIO_Config();
	TIM6_Config();  
	DAC_DeInit(); 
	DAC_Ch2_SineWaveConfig();
}

static void SpeakerGPIO_Config(void)
{
	/* Preconfiguration before using DAC----------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/* DMA1 clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	/* GPIOA clock enable (to be used with DAC) */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                         
	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	/* DAC channel 1 (DAC_OUT2 = PA.5) configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**             
  * @brief  TIM6 Configuration
  * @note   TIM6 configuration is based on APB1 frequency
  * @note   TIM6 Update event occurs each TIM6CLK/256   
  * @param  None
  * @retval None
  */
static void TIM6_Config(void)
{
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	/* TIM6 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* --------------------------------------------------------
	TIM3 input clock (TIM6CLK) is set to 2 * APB1 clock (PCLK1), 
	since APB1 prescaler is different from 1.   
	TIM6CLK = 2 * PCLK1  
	TIM6CLK = HCLK / 2 = SystemCoreClock /2 

	TIM6 Update event occurs each TIM6CLK/256 

	Note: 
	SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	function to update SystemCoreClock variable value. Otherwise, any configuration
	based on this variable will be incorrect.    

	----------------------------------------------------------- */
	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Period = 1604;          
	TIM_TimeBaseStructure.TIM_Prescaler = 0;       
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	/* TIM6 TRGO selection */
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	//TIM_SelectOutputTrigger(TIM6,TIM_TRGOSource_Reset);

	/* TIM6 enable counter */
	TIM_Cmd(TIM6, ENABLE);
}

/**
  * @brief  DAC  Channel2 SineWave Configuration
  * @param  None
  * @retval None
  */
static void DAC_Ch2_SineWaveConfig(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	/* DAC channel2 Configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;//DAC_Trigger_None;//
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	/* DMA1_Stream6 channel7 configuration **************************************/
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R2_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&aSine12bit;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 32;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);

	/* Enable DMA1_Stream6 */
	DMA_Cmd(DMA1_Stream6, ENABLE);

	/* Enable DAC Channel2 */
	DAC_Cmd(DAC_Channel_2, ENABLE);

	/* Enable DMA for DAC Channel2 */
	DAC_DMACmd(DAC_Channel_2, ENABLE);
}

void Moo_OFF(void)
{
	memcpy(aSine12bit, (uint16_t [32]){0, 0, 0, 0, 0, 0, 0, 0, 
										   0, 0, 0, 0, 0, 0, 0, 0,
										   0, 0, 0, 0, 0, 0, 0, 0, 
										   0, 0, 0, 0, 0, 0, 0, 0}, 
										   32*sizeof(uint16_t));
}

void Moo_ON(void)
{											 
	memcpy(aSine12bit, (uint16_t [32]){2047, 2447, 2831, 3185, 3498, 
										   3750, 3939, 4056, 4095, 4056,
										   3939, 3750, 3495, 3185, 2831, 
										   2447, 2047, 1647, 1263, 909, 
										   599, 344, 155, 38, 0, 38, 
										   155, 344, 599, 909, 1263, 1647}, 
										   32*sizeof(uint16_t));
}
/////////////////////////////////////////////////////////////////////////////
