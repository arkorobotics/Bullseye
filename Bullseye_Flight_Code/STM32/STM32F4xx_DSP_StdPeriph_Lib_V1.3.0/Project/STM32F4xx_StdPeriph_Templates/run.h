/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RUN_H
#define RUN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

extern uint8_t run;
extern uint8_t gyro_enable;
extern uint32_t rodeo_count;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void RunSwitchInit(void);

#endif /* RUN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
