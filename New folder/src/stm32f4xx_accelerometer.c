/********      I2C function to read/write sensor LSM303DLHC  accelerometer******/
#include "stm32f4xx_accelerometer.h"
GPIO_InitTypeDef 				Acc_GPIO_Struct;
I2C_InitTypeDef					Acc_I2C_Struct;
/* ------- I2C configuration (datasheet about connecting stm32 vs peripherals*/
void I2C_Config()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	/*----Config I2C pin-----*/
	Acc_GPIO_Struct.GPIO_Pin   							= GPIO_Pin_6|GPIO_Pin_9;
	Acc_GPIO_Struct.GPIO_Mode								= GPIO_Mode_AF;
	Acc_GPIO_Struct.GPIO_OType							= GPIO_OType_OD;
	Acc_GPIO_Struct.GPIO_PuPd								= GPIO_PuPd_UP;
	Acc_GPIO_Struct.GPIO_Speed							= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&Acc_GPIO_Struct);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_I2C1);
	/*----I2C config-----*/
	Acc_I2C_Struct.I2C_Mode                 = I2C_Mode_I2C;
	Acc_I2C_Struct.I2C_DutyCycle						= I2C_DutyCycle_2;
	Acc_I2C_Struct.I2C_OwnAddress1					= 0x00;
	Acc_I2C_Struct.I2C_Ack									= I2C_Ack_Enable;
	Acc_I2C_Struct.I2C_ClockSpeed						= 100000;    //100KHz
	Acc_I2C_Struct.I2C_AcknowledgedAddress	= I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1,&Acc_I2C_Struct);
	I2C_Cmd(I2C1,ENABLE);
}
/*------- I2C start transmit ------*/
CheckEVStatus I2C_Start_Transmit()
{
	uint16_t time_out;
	
	I2C_GenerateSTART(I2C1,ENABLE);
	
	time_out        = Accelerometer_TimeOut;
	//Check on EV5
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
	{
		if ((time_out--) == 0)
		{return Not_OK;}
	}
	//Send slave address
	I2C_Send7bitAddress(I2C1,Accelerometer_Write,I2C_Direction_Transmitter);
	
	//Check on EV6
	time_out  			= Accelerometer_TimeOut;
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		if((time_out--) == 0)
		{return Not_OK;}
	}
	return OK;
}

/*------------I2C Start receive----------------*/
CheckEVStatus I2C_Start_Receive()
{
	uint16_t time_out;
	
	I2C_GenerateSTART(I2C1,ENABLE);
	
	//Check on EV5
	time_out = Accelerometer_TimeOut;
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
	{
		if (time_out-- == 0)
			return Not_OK;
	}
	
	I2C_Send7bitAddress(I2C1,Accelerometer_Read,I2C_Direction_Receiver);
	
	//Check on EV6
	time_out = Accelerometer_TimeOut;
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if (time_out-- == 0)
			return Not_OK;
	}
	return OK;
}
/*-------- Write To Accelerometer --------------*/
CheckEVStatus WriteToAccelerometer(uint8_t register_addr, uint8_t data)
{
	//Check bus busy or not
	uint16_t time_out;
	time_out = Accelerometer_TimeOut;
	while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY))
	{
		if (time_out-- == 0)
			return Not_OK;
	}
	if (I2C_Start_Transmit())
	{
		I2C_SendData(I2C1,register_addr);
		time_out = Accelerometer_TimeOut;
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			if (time_out-- == 0)
				return Not_OK;
		}
		I2C_SendData(I2C1,data);
		time_out = Accelerometer_TimeOut;
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			if (time_out-- == 0)
				return Not_OK;
		}
		I2C_GenerateSTOP(I2C1,ENABLE);
	}
	else return Not_OK;
	return OK;
}
/*---------- Read from accelerometer ---------------*/
CheckEVStatus ReadFromAccelerometer(uint8_t register_addr, uint8_t *data)
{
	uint16_t time_out;
	time_out = Accelerometer_TimeOut;
	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY))
	{
		if(time_out-- == 0)
			return Not_OK;
	}
	if(I2C_Start_Transmit())
	{
		I2C_SendData(I2C1,register_addr);
		time_out = Accelerometer_TimeOut;
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			if(time_out-- == 0)
				return Not_OK;
		}
		if(I2C_Start_Receive())
		{
			I2C_AcknowledgeConfig(I2C1,ENABLE);
			time_out = Accelerometer_TimeOut;
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
			{
				if(time_out-- == 0)
					return Not_OK;
			}
			*data = I2C_ReceiveData(I2C1);
		}
		else return Not_OK;
	}
	else return Not_OK;
	return OK;
}
/*----------Read 3 axis value from accelerometer-----------------*/
CheckEVStatus Read3AxisAccelerometer(uint8_t *register_addr, double *pBuffer)
{
	uint16_t time_out;
	uint8_t  TempBuffer[6];
	int16_t  Temp[3];
	uint8_t  start_addr = 0x28;
	for (int i = 0; i < 6; i++)
	{
		time_out = Accelerometer_TimeOut;
		while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY))
		{
			if(time_out-- == 0)
				return Not_OK;
		}
		if (I2C_Start_Transmit())
		{
			I2C_SendData(I2C1,start_addr + i);
			time_out = Accelerometer_TimeOut;
			while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			{
				if (time_out-- == 0)
					return Not_OK;
			}
			if (I2C_Start_Receive())
			{
				I2C_AcknowledgeConfig(I2C1,ENABLE);
				time_out = Accelerometer_TimeOut;
				while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
				{
					if (time_out-- == 0)
						return Not_OK;
				}
				TempBuffer[i] = I2C_ReceiveData(I2C1);
			}
			else return Not_OK;
		}
		else return Not_OK;
		I2C_AcknowledgeConfig(I2C1,DISABLE);
		I2C_GenerateSTOP(I2C1,ENABLE);
	}
	Temp[0] = TempBuffer[1] * 256 + TempBuffer[0];
	pBuffer[0] = (float)(4 * Temp[0]) / (float)65535;
	Temp[1] = TempBuffer[3] * 256 + TempBuffer[2];
	pBuffer[1] = (float)(4 * Temp[1]) / (float)65535;
	Temp[2] = TempBuffer[5] * 256 + TempBuffer[4];
	pBuffer[2] = (float)(4 * Temp[2]) / (float)65535;
	return OK;
}
/*--------------Accelerometer Init-----------------*/
void Accelerometer_Init()
{
	// ----Normal/ Low power mode (50Hz) (Ctrl Reg 1: 20h)
	WriteToAccelerometer(LSM303DLHC_Ctrl_Reg_1,LSM303DLHC_Mode_Normal_50Hz_3Axis);
}




