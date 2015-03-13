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
#include "imu.h"
#include "i2c.h"

/////////////////////////////////////////////////////////////////////////////
// IMU - Variables
/////////////////////////////////////////////////////////////////////////////
uint8_t GYRO_X_HIGH = 0;
uint8_t GYRO_X_LOW = 0;
uint8_t GYRO_Y_HIGH = 0;
uint8_t GYRO_Y_LOW = 0;
uint8_t GYRO_Z_HIGH = 0;
uint8_t GYRO_Z_LOW = 0;

uint8_t ACCEL_X_HIGH = 0;
uint8_t ACCEL_X_LOW = 0;
uint8_t ACCEL_Y_HIGH = 0;
uint8_t ACCEL_Y_LOW = 0;
uint8_t ACCEL_Z_HIGH = 0;
uint8_t ACCEL_Z_LOW = 0;

uint8_t MAG_X_HIGH = 0;
uint8_t MAG_X_LOW = 0;
uint8_t MAG_Y_HIGH = 0;
uint8_t MAG_Y_LOW = 0;
uint8_t MAG_Z_HIGH = 0;
uint8_t MAG_Z_LOW = 0;

int16_t GYROX = 0;
int16_t GYROY = 0;
int16_t GYROZ = 0;

int16_t ACCELX = 0;
int16_t ACCELY = 0;
int16_t ACCELZ = 0;

int16_t MAGX = 0;
int16_t MAGY = 0;
int16_t MAGZ = 0;


double gyro[3] = {0.000, 0.000, 0.000};

double accel[3] = {0.000, 0.000, 0.000};

double mag[3] = {0.000, 0.000, 0.000};

double heading = 0.000;

int magstart[3] = {-5,-95,-260};

double magCal[3] = {0.000, 0.000, 0.000};

uint8_t buffer[14] = {0, 0, 0, 0,
					  0, 0, 0, 0,
					  0, 0, 0, 0,
					  0, 0};

uint8_t mag8[6] = {0, 0, 0, 0, 0, 0};


void initAK8975A(double *destination)
{
	uint8_t rawData[3];  										// x/y/z gyro register data stored here
	I2cWriteConfig(0x18, 0x0A, 0x00); 			// Power down
	I2cWriteConfig(0x18, 0x0A, 0x0F); 			// Enter Fuse ROM access mode
	I2cRead(0x18, 0x10, rawData, 0x03);  		// Read the x-, y-, and z-axis calibration values
	destination[0] =  ((double)(rawData[0] - 128))/256. + 1.; // Return x-axis sensitivity adjustment values
	destination[1] =  ((double)(rawData[1] - 128))/256. + 1.;  
	destination[2] =  ((double)(rawData[2] - 128))/256. + 1.; 
}

void IMU_Init(void)
{
	/* Insert 50 ms delay Delay(5)*/
	// I2CInit(); // Removed because it is called in main.c

	//Configuration
	I2cWriteConfig(0xD0, 0x6B, 0x01);
	I2cWriteConfig(0xD0, 0x1B, 0x00);
	I2cWriteConfig(0xD0, 0x1C, 0x00);
	I2cWriteConfig(0xD0, 0x1A, 0x00);
	I2cWriteConfig(0xD0, 0x19, 0x00);
	I2cWriteConfig(0xD0, 0x37, 0x02);
	I2cWriteConfig(0xD0, 0x6A, 0x00);
	I2cWriteConfig(0xD0, 0x6C, 0x00);
	initAK8975A(magCal);
	I2cWriteConfig(0x18, 0x0A, 0x01); 
}

void IMU_Update(void)	
{
	//Accel Readings
	I2cRead(0xD0, 0x3B, buffer, 0x0E);
	ACCELX = ((int16_t)buffer[0] << 8)| buffer[1];
	accel[0] = ((double)ACCELX)*(0.000061);
	ACCELY = ((int16_t)buffer[2] << 8) | buffer[3];
	accel[1] = ((double)ACCELY)*(0.000061);
	ACCELZ = ((int16_t)buffer[4] << 8) | buffer[5];
	accel[2] = ((double)ACCELZ)*(0.000061);

	//Gyro Readings degree/sec
	GYROX = ((int16_t)buffer[8] << 8) | buffer[9];
	gyro[0] = ((double)GYROX)*(0.007629);
	GYROY = ((int16_t)buffer[10] << 8) | buffer[11];
	gyro[1] = ((double)GYROY)*(0.007629);
	GYROZ = ((int16_t)buffer[12] << 8) | buffer[13];
	gyro[2] = ((double)GYROZ)*(0.007629);
		
	//MAG Readings degree/sec
	I2cRead(0x18, 0x03, mag8, 0x06);
	MAGX = ((int16_t)mag8[0] << 8) | mag8[1];
	mag[0] = ((double)MAGX)*(3.00049)*(magCal[0]) - magstart[0];
	MAGY = ((int16_t)mag8[2] << 8) | mag8[3];
	mag[1] = ((double)MAGY)*(3.00049)*(magCal[1]) - magstart[1];
	MAGZ = ((int16_t)mag8[4] << 8) | mag8[5];
	mag[2] = ((double)MAGZ)*(3.00049)*(magCal[2]) - magstart[2];
	I2cWriteConfig(0x18, 0x0A, 0x01); 
}
