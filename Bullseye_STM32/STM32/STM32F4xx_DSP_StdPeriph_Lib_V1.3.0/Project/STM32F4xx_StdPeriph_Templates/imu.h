/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IMU_H
#define IMU_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern double accel[3];
extern double gyro[3];
extern double mag[3];
extern double heading;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void initAK8975A(double *destination);
void IMU_Init(void);
void IMU_Update(void); 

#endif /* IMU_H */

/*****************************END OF FILE**************************************/
