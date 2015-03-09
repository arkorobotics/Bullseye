/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPEAKER_H
#define SPEAKER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern int moo_count;
extern __IO uint8_t ubSelectedWavesForm, ubKeyPressed;
extern uint16_t aSine12bit[32];

/* Exported macro ------------------------------------------------------------*/
#if defined (USE_STM324xG_EVAL)
  #define DAC_DHR12R2_ADDRESS    0x40007414
  #define DAC_DHR8R1_ADDRESS     0x40007410

#else /* defined (USE_STM324x7I_EVAL)*/ 
  #define DAC_DHR12R2_ADDRESS    0x40007414
  #define DAC_DHR8R1_ADDRESS     0x40007410

#endif

/* Exported functions ------------------------------------------------------- */

void Speaker_Config(void);
static void SpeakerGPIO_Config(void);
static void TIM6_Config(void);
static void DAC_Ch2_SineWaveConfig(void);
void Moo(void);

#endif /* SPEAKER_H */

/*****************************END OF FILE**************************************/
