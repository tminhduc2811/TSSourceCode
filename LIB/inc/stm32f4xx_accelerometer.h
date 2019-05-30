#include "stm32f4xx.h"

/* Define */
#define 								Accelerometer_Read 											 0x33
#define									Accelerometer_Write		 									 0x32
#define         				LSM303DLHC_Ctrl_Reg_1                    0x20
#define         				LSM303DLHC_Mode_Normal_50Hz_3Axis        0x47
#define 								Accelerometer_TimeOut										 20000
/* Export Types */
typedef enum{
	Not_OK = 0,
	OK,
}CheckEVStatus;
/* Export variables */
/* Export function */
void I2C_Config(void);
CheckEVStatus WriteToSensor(uint8_t register_addr, uint8_t data);
CheckEVStatus ReadFromSensor(uint8_t register_addr, uint8_t *data);
CheckEVStatus Read3AxisAccelerometer(uint8_t *register_addr, double *pBuffer);
void Accelerometer_Init(void);





