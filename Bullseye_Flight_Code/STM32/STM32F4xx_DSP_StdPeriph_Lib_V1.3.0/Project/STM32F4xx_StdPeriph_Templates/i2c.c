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
// IMU.C
// --------------------------------------------------------------------
#include "stm32f4xx.h"
#include "i2c.h"

/////////////////////////////////////////////////////////////////////////////
// I2C - Variables
/////////////////////////////////////////////////////////////////////////////
#define LONG_TIMEOUT				((uint32_t)0x12C000)
#define LONGER_TIMEOUT				((uint32_t)0x5A00000)

/////////////////////////////////////////////////////////////////////////////
// I2C - Functions
/////////////////////////////////////////////////////////////////////////////
unsigned char I2cRead(unsigned char I2cAddress, unsigned char Register, uint8_t *pData, short unsigned int Length)
{
	__IO uint32_t Timeout = LONG_TIMEOUT;  

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0)
		{ 
			I2CInit();
			return 1;
		}
	}

	I2C_GenerateSTART(I2C1, ENABLE);// Start the config sequence 

	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))// Test on EV5 and clear it 
	{
		if((Timeout--) == 0) 
		{ 
			I2CInit();
			return 1;
		}
	}

	I2C_Send7bitAddress(I2C1, I2cAddress, I2C_Direction_Transmitter); //chaeckthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
		{ 
			I2CInit();
			return 1;
		}
	}

	I2C_SendData(I2C1, Register); //checkthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))/* Test on EV8_2 and clear it */
	{
		if((Timeout--) == 0) 
		{ 
			I2CInit();
			return 1;
		}
	}

	I2C_GenerateSTART(I2C1, ENABLE); /* Start the config sequence */

	/* Test on EV5 and clear it */
	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((Timeout--) == 0) 
		{ 
			I2CInit();
			return 1;
		}
	}

	I2C_Send7bitAddress(I2C1, I2cAddress , I2C_Direction_Receiver);/* Transmit the slave address and enable writing operation */

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
		{ 
			I2CInit();
			return 1;
		}
	}

	(void)I2C1->SR2;

	while(Length > 0)
	{
		Timeout = LONG_TIMEOUT;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) /* Test on EV7 and clear it */
		{
			if((Timeout--) == 0) 
			{ 
				I2CInit();
				return 1;
			}
		}	

		if(Length == 1)
			I2C_AcknowledgeConfig(I2C1, DISABLE);

		*pData++ = I2C_ReceiveData(I2C1);
		Length--;
	}

	I2C_GenerateSTOP(I2C1, ENABLE); /* End the configuration sequence */ 

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
		{ 
			I2CInit();
			return 1;
		}
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;  /* Return the verifying value: 0 (Passed) or 1 (Failed) */		
}

unsigned char I2cWrite(unsigned char I2cAddress, unsigned char Register, signed char Data[], short unsigned int Length)
{
	__IO uint32_t Timeout = LONG_TIMEOUT;  

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTART(I2C1, ENABLE);// Start the config sequence 

	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))// Test on EV5 and clear it 
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_Send7bitAddress(I2C1, I2cAddress, I2C_Direction_Transmitter); 

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, Register); //checkthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))/* Test on EV8_2 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	for(int i = 0; i<Length; i++)
	{
		Timeout = LONG_TIMEOUT;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))/* Test on EV8_2 and clear it */
		{
			if((Timeout--) == 0) 
				return 1;
		}
		I2C_SendData(I2C1, Data[i]);
	}

	I2C_GenerateSTOP(I2C1, ENABLE); /* End the configuration sequence */ 

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
		return 1;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;  /* Return the verifying value: 0 (Passed) or 1 (Failed) */		
}

unsigned char I2cWriteConfig(unsigned char I2cAddress, unsigned char Register, unsigned char Data)
{
	__IO uint32_t Timeout = LONG_TIMEOUT;  

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTART(I2C1, ENABLE);// Start the config sequence 

	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))// Test on EV5 and clear it 
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_Send7bitAddress(I2C1, I2cAddress, I2C_Direction_Transmitter); 

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, Register); //checkthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))/* Test on EV8_2 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, Data);

	/* Test on EV5 and clear it */
	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTOP(I2C1, ENABLE); /* End the configuration sequence */ 

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;  /* Return the verifying value: 0 (Passed) or 1 (Failed) */		
}

unsigned char I2cWriteMag(unsigned char I2cAddress,unsigned char DeviceID, unsigned char Register, signed char Data)
{
	__IO uint32_t Timeout = LONG_TIMEOUT;  

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTART(I2C1, ENABLE);// Start the config sequence 

	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))// Test on EV5 and clear it 
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_Send7bitAddress(I2C1, I2cAddress, I2C_Direction_Transmitter); 

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, DeviceID); //checkthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))/* Test on EV8_2 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, Register);

	/* Test on EV5 and clear it */
	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, Data);

	/* Test on EV5 and clear it */
	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTOP(I2C1, ENABLE); /* End the configuration sequence */ 

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;  /* Return the verifying value: 0 (Passed) or 1 (Failed) */		
}

unsigned char I2cReadMag(unsigned char I2cAddress, unsigned char DeviceID, unsigned char Register, signed char *pData, short unsigned int Length)
{
	__IO uint32_t Timeout = LONG_TIMEOUT;  

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTART(I2C1, ENABLE);// Start the config sequence 

	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))// Test on EV5 and clear it 
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_Send7bitAddress(I2C1, I2cAddress, I2C_Direction_Transmitter); //chaeckthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, DeviceID); //checkthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))/* Test on EV8_2 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_SendData(I2C1, Register); //checkthis

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))/* Test on EV8_2 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTART(I2C1, ENABLE); /* Start the config sequence */

	/* Test on EV5 and clear it */
	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_Send7bitAddress(I2C1, I2cAddress , I2C_Direction_Receiver);/* Transmit the slave address and enable writing operation */

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	(void)I2C1->SR2;

	while(Length > 0)
	{
		Timeout = LONG_TIMEOUT;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) /* Test on EV7 and clear it */
		{
			if((Timeout--) == 0) 
				return 1;
		}	

		if(Length == 1)
			I2C_AcknowledgeConfig(I2C1, DISABLE);

		*pData++ = I2C_ReceiveData(I2C1);
		Length--;
	}

	I2C_GenerateSTOP(I2C1, ENABLE); /* End the configuration sequence */ 

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;  /* Return the verifying value: 0 (Passed) or 1 (Failed) */		
}

unsigned char I2cSonarRead(unsigned char I2cAddress, uint8_t *pData, short unsigned int Length){
	__IO uint32_t Timeout = LONGER_TIMEOUT;  
	
	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_GenerateSTART(I2C1, ENABLE);// Start the config sequence 

	Timeout = LONG_TIMEOUT;
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))// Test on EV5 and clear it 
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_Send7bitAddress(I2C1, I2cAddress , I2C_Direction_Receiver);/* Transmit the slave address and enable writing operation */

	Timeout = LONG_TIMEOUT;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))/* Test on EV6 and clear it */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	(void)I2C1->SR2;

	//Length--;		//needed to receive correct number of bytes
	while(Length > 0)
	{
		Length--;
		Timeout = LONG_TIMEOUT;
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) /* Test on EV7 and clear it */
		{
			if((Timeout--) == 0) 
				return 1;
		}	

		if(Length == 1)
			I2C_AcknowledgeConfig(I2C1, DISABLE);

		*pData++ = I2C_ReceiveData(I2C1);
		//Length--;
	}

	I2C_GenerateSTOP(I2C1, ENABLE); /* End the configuration sequence */ 

	Timeout = LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /*!< While the bus is busy */
	{
		if((Timeout--) == 0) 
			return 1;
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0;  /* Return the verifying value: 0 (Passed) or 1 (Failed) */		
}

void I2CInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOBEN,ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE); /* Release reset signal of I2C1 IP */	

	/* CODEC_I2C SCL and SDA pins configuration -------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //8 : SCL
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);     

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //9 : SDA
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); 

	//I2c1 Init 
	I2C_SoftwareResetCmd(I2C1, ENABLE);
	I2C_SoftwareResetCmd(I2C1, DISABLE);

	I2C_DeInit(I2C1);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x40;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &I2C_InitStructure); 

	//Board Led's
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}
