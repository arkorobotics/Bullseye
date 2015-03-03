/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef I2C_H
#define I2C_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

unsigned char I2cRead(unsigned char I2cAddress, unsigned char Register, uint8_t *pData, short unsigned int Length);
unsigned char I2cWrite(unsigned char I2cAddress, unsigned char Register, signed char Data[], short unsigned int Length);
unsigned char I2cWriteConfig(unsigned char I2cAddress, unsigned char Register, unsigned char Data);
unsigned char I2cWriteMag(unsigned char I2cAddress,unsigned char DeviceID, unsigned char Register, signed char Data);
unsigned char I2cReadMag(unsigned char I2cAddress, unsigned char DeviceID, unsigned char Register, signed char *pData, short unsigned int Length);
void I2CInit(void);



#endif /* I2C_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
