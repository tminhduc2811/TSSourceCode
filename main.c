#include "main.h"

/** @Control @Characters **/
/** Motor Control
** @A  : 
** @W  :
** @S  :
** @D  : 
** @C  : Config Peripheral.
** Format: C , v , angle , M1kp , M1kd , M1ki , M2kp , M2kd , M2ki
** Send PC format: VelM1,VelM2,angle,Y/N,xcor,ycor
**/
/* Used peripherals */
// TIM3, TIM4, TIM9
/* Global Variables */
TIM_TimeBaseInitTypeDef			Main_TIM_Struct;
NVIC_InitTypeDef						Main_NVIC_Struct;
Srf05_InitTypeDef						Srf05_Struct;
/* Function */
#define	 							Declare_Var(type,name)				type SPIDMA_##name
#define								Declare_PVar(type,name)				type *SPIDMA_##name
#define								GetVar(name)									SPIDMA_##name
Declare_Var(SPI_InitTypeDef,SPI_Struct);
Declare_Var(GPIO_InitTypeDef, GPIO_Struct);
Declare_Var(DMA_InitTypeDef, DMA_Struct);
Declare_Var(NVIC_InitTypeDef, NVIC_Struct);
uint8_t SPI4_RxBuffer[9];
void SPI4_DMA_Config(void)
{
	GetSPIxPeriphClock(SPI4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	/* GPIO config */
	GetVar(GPIO_Struct).GPIO_Mode = GPIO_Mode_AF;
	GetVar(GPIO_Struct).GPIO_Pin  = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GetVar(GPIO_Struct).GPIO_OType = GPIO_OType_PP;
	GetVar(GPIO_Struct).GPIO_PuPd = GPIO_PuPd_UP;
	GetVar(GPIO_Struct).GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GetVar(GPIO_Struct));
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_SPI4);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource12,GPIO_AF_SPI4);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_SPI4);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_SPI4);
	
	/*SPI config */
	GetVar(SPI_Struct).SPI_NSS							= SPI_NSS_Soft;
	GetVar(SPI_Struct).SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	GetVar(SPI_Struct).SPI_DataSize  				= SPI_DataSize_8b;
	GetVar(SPI_Struct).SPI_CPHA							= SPI_CPHA_1Edge;
	GetVar(SPI_Struct).SPI_CPOL     				= SPI_CPOL_Low;
	GetVar(SPI_Struct).SPI_Direction 				= SPI_Direction_2Lines_FullDuplex;
	GetVar(SPI_Struct).SPI_FirstBit 				= SPI_FirstBit_MSB;
	GetVar(SPI_Struct).SPI_Mode    					= SPI_Mode_Slave;
	SPI_Init(SPI4,&GetVar(SPI_Struct));
	SPI_Cmd(SPI4,ENABLE);
	SPI_I2S_DMACmd(SPI4,SPI_I2S_DMAReq_Rx,ENABLE);
	
	//-----------Config DMA SPI Rx------------------
	GetVar(DMA_Struct).DMA_Channel									= DMA_Channel_4;
	GetVar(DMA_Struct).DMA_BufferSize								= 9;
	GetVar(DMA_Struct).DMA_Mode 										= DMA_Mode_Normal;
	GetVar(DMA_Struct).DMA_DIR											= DMA_DIR_PeripheralToMemory;
	GetVar(DMA_Struct).DMA_Memory0BaseAddr					= (uint32_t)&SPI4_RxBuffer;
	GetVar(DMA_Struct).DMA_MemoryBurst 							= DMA_MemoryBurst_Single;
	GetVar(DMA_Struct).DMA_MemoryDataSize						= DMA_MemoryDataSize_Byte;
	GetVar(DMA_Struct).DMA_MemoryInc								= DMA_MemoryInc_Enable;
	GetVar(DMA_Struct).DMA_PeripheralBaseAddr 			= (uint32_t)&SPI4->DR;
	GetVar(DMA_Struct).DMA_PeripheralBurst					=	DMA_PeripheralBurst_Single;
	GetVar(DMA_Struct).DMA_PeripheralDataSize				= DMA_PeripheralDataSize_Byte;
	GetVar(DMA_Struct).DMA_PeripheralInc						= DMA_PeripheralInc_Disable;
	GetVar(DMA_Struct).DMA_FIFOMode									= DMA_FIFOMode_Disable;
	GetVar(DMA_Struct).DMA_FIFOThreshold						= DMA_FIFOThreshold_HalfFull;
	GetVar(DMA_Struct).DMA_Priority 								= DMA_Priority_Medium;
	DMA_Init(DMA2_Stream0,&GetVar(DMA_Struct));
	DMA_Cmd(DMA2_Stream0,ENABLE);
	//------------NVIC Config----------
	GetVar(NVIC_Struct).NVIC_IRQChannel 										= DMA2_Stream0_IRQn;
	GetVar(NVIC_Struct).NVIC_IRQChannelPreemptionPriority 	= 2;
	GetVar(NVIC_Struct).NVIC_IRQChannelSubPriority					= 0;
	GetVar(NVIC_Struct).NVIC_IRQChannelCmd									= ENABLE;
	NVIC_Init(&GetVar(NVIC_Struct));
	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
}

/** @brief  : TIM1 interrupt count config
**  @agr    : void
**  @retval : void
**/
void TimerDelayConfig(TIM_TypeDef *TIMx)   // ms
{
	GetTIMxClockCmd(TIMx,ENABLE);
	
	Main_TIM_Struct.TIM_Prescaler  				=  50000 - 1; //0.5ms
	Main_TIM_Struct.TIM_Period     				=  0;
	Main_TIM_Struct.TIM_ClockDivision 		=  TIM_CKD_DIV1;
	Main_TIM_Struct.TIM_CounterMode  			=  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMx,&Main_TIM_Struct);
	
	Main_NVIC_Struct.NVIC_IRQChannel  		=  GetIRQHandlerFromTIMxCC(TIMx);
	Main_NVIC_Struct.NVIC_IRQChannelCmd 	=  ENABLE;
	Main_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 3;
	Main_NVIC_Struct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&Main_NVIC_Struct); 
}

void StartTimer(TIM_TypeDef *TIMx, uint32_t DelayTime)
{
	Main_TIM_Struct.TIM_Period						= DelayTime * 2 - 1;
	TIM_TimeBaseInit(TIMx,&Main_TIM_Struct);
	Status_UpdateStatus(&VehStt.Veh_Timer_Finish,Check_NOK);
	TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIMx, ENABLE);
}


void StopTimer(TIM_TypeDef *TIMx)
{
	TIM_Cmd(TIMx,DISABLE);
	TIM_ITConfig(TIMx,TIM_IT_Update,DISABLE);
}

void TIM5_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
	if(!Status_CheckStatus(&VehStt.Veh_Timer_Start))
	{
		Status_UpdateStatus(&VehStt.Veh_Timer_Start,Check_OK);
	}
	else
	{
		StopTimer(TIM5);
		Status_UpdateStatus(&VehStt.Veh_Timer_Start,Check_NOK);
		Status_UpdateStatus(&VehStt.Veh_Timer_Finish,Check_OK);
	}
}

/** @brief  : Peripheral config
**  @agr    : void
**  @retval : void
**/
void Peripheral_Config(void)
{
	TimerDelayConfig(TIM5);
	USART1_Config(115200);
	USART2_Config(9600); 			//GPS USART first priority 
	USART6_Config(9600); 			//Sending and controling USART1
	Encoder_Config();				
	SPI4_DMA_Config();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/** @brief  : Delay core clock 100MHZ
**  @agr    : time
**  @retval : void
**/
void Delay(uint32_t time)
{ 	
	while(time--)
	{};
}

void GPS_ReadParametersFromFlash(FlashMemory *pflash, GPS *pgps)
{
	ReadFromFlash(pflash, FLASH_GPSPara_BaseAddr);
	GetMessageInfo((char*)pflash->ReadOutBuffer, Flash.Message, ',');
	pgps->NbOfWayPoints = GetValueFromString(&Flash.Message[0][0]);
	for(int i = 0; i < pgps->NbOfWayPoints; i++)
	{
		pgps->Latitude 			= GetValueFromString(&Flash.Message[i * 2 + 1][0]);
		pgps->Longitude			= GetValueFromString(&Flash.Message[i * 2 + 2][0]);
		GPS_LatLonToUTM(pgps);
		pgps->Path_X[i]			= pgps->CorX;
		pgps->Path_Y[i]			= pgps->CorY;
	}
	GPS_UpdatePathYaw(pgps);
}

void GPS_SaveManual(void)
{
	Flash.Length = 0;
	Flash.Length += ToChar(2,&Flash.WriteInBuffer[Flash.Length],2);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(10.2565,&Flash.WriteInBuffer[Flash.Length],4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(106.3256,&Flash.WriteInBuffer[Flash.Length],4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(10.3536,&Flash.WriteInBuffer[Flash.Length],4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(106.1156,&Flash.WriteInBuffer[Flash.Length],4);
	EraseMemory(FLASH_Sector_6);
	WriteToFlash(&Flash,FLASH_Sector_6,FLASH_GPSPara_BaseAddr);
}
/** @brief  : Read PID parameters from internal flash memory
**  @agr    : void
**  @retval : void
**/
void PID_ReadParametersFromFlash(void)
{
	ReadFromFlash(&Flash,FLASH_PIDPara_BaseAddr);
	GetMessageInfo((char*)Flash.ReadOutBuffer,Flash.Message,',');
	PID_ParametersUpdate(&M1,GetValueFromString(&Flash.Message[0][0]),GetValueFromString(&Flash.Message[1][0]),GetValueFromString(&Flash.Message[2][0]));
	PID_ParametersUpdate(&M2,GetValueFromString(&Flash.Message[3][0]),GetValueFromString(&Flash.Message[4][0]),GetValueFromString(&Flash.Message[5][0]));
}

void PID_SaveManual(void)
{
	Flash.Length		= 0;
	Flash.Length    += ToChar(0.060000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.080000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.050000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.100000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length],6);
	EraseMemory(FLASH_Sector_7);
	WriteToFlash(&Flash,FLASH_Sector_7,FLASH_PIDPara_BaseAddr);
}
void SendStatusData(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'E';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'H';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'S';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'T';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Veh.Mode,&U6_TxBuffer[Veh.SendData_Ind],1); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(GPS_NEO.NbOfWayPoints,&U6_TxBuffer[Veh.SendData_Ind],1); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(GPS_NEO.K,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Veh.Max_Velocity,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Kp,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Ki,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Kd,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M2.Kp,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M2.Ki,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M2.Kd,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Mag.Ke,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Mag.Kedot,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Mag.Ku,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void SendData_0(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'I';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'F';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'O';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'0';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Set_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M1 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(M2.Set_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M2 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(M1.Current_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M1 Velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind  		+= ToChar(M2.Current_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M2 velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(Mag.Set_Angle,&U6_TxBuffer[Veh.SendData_Ind],3); // Set angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   		+= ToChar(Mag.Angle,&U6_TxBuffer[Veh.SendData_Ind],3); // Current angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   		+= ToChar(Veh.Sensor_Angle,&U6_TxBuffer[Veh.SendData_Ind],3); // Current angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void SendData_1(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'I';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'F';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'O';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'1';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	if(Status_CheckStatus(&VehStt.GPS_ValidGPS))
	{
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'Y';
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	}
	else
	{
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	}
	Veh.SendData_Ind   			+= ToChar(GPS_NEO.GPS_Quality,&U6_TxBuffer[Veh.SendData_Ind],1);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   			+= ToChar(GPS_NEO.Latitude,&U6_TxBuffer[Veh.SendData_Ind],13);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   			+= ToChar(GPS_NEO.Longitude,&U6_TxBuffer[Veh.SendData_Ind],13);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void SendData_2(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'I';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'F';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'O';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'2';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Thetae * (180/pi),&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Thetad * (180/pi),&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Delta_Angle,&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.dmin,&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.efa,&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void GPS_StanleyCompute(void)
{
	GPS_StanleyControl(&GPS_NEO, Timer.T, M1.Current_Vel, M2.Current_Vel);
	
	
	IMU_UpdateSetAngle(&Mag,GPS_NEO.Delta_Angle);
	IMU_UpdateFuzzyInput(&Mag,&Timer.T);
	Defuzzification_Max_Min(&Mag);
	if(Mag.Fuzzy_Out >= 0)
	{
		PID_UpdateSetVel(&M1,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
		PID_UpdateSetVel(&M2,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
	}
	else
	{
		PID_UpdateSetVel(&M1,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
		PID_UpdateSetVel(&M2,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
	}
	if(Status_CheckStatus(&GPS_NEO.Goal_Flag))
	{
		PID_UpdateSetVel(&M1,0);
		PID_UpdateSetVel(&M2,0);
	}
}

/** @brief  : Initial parameters for input
**  @agr    : void
**  @retval : void
**/
void Parameters_Init(void)
{
	/*-----------------Error Init ---------------*/
	Error_ResetIndex(&Veh_Error);
	/*-----------------Timer Init ---------------*/
	Time_ParametersInit(&Timer,50000,1000000);
	Time_GetSampleTime(&Timer);
	Status_ParametersInit(&VehStt);
	/*------------PID Parameter Init-------------*/
	PID_ReadParametersFromFlash();
	PID_ParametersInitial(&M1);
	PID_ParametersInitial(&M2);
	/*------------Fuzzy parametes Init ----------*/
	Fuzzy_ParametersInit();
	/*------------------AngleControl-------------*/
	IMU_ParametesInit(&Mag);
	IMU_UpdateFuzzyCoefficients(&Mag,(double)1/180,(double)1/30,(double)1);
	/*------------------StanleyParameter---------*/
	//GPS_ReadParametersFromFlash(&Flash,&GPS_NEO);
	//EraseMemory(FLASH_Sector_6);
	GPS_ParametersInit(&GPS_NEO);
	/*------------------Vehicleinit--------------*/
	Veh_ParametersInit(&Veh);
	/*----------------- Robot two wheel Init-----*/
	Self_ParametersInit(&selfPosition);
}
int main(void)
{
	Parameters_Init();
	Peripheral_Config();
	//PID_SaveManual();
	//GPS_SaveManual();

	SysTick_Config(SystemCoreClock/1000000); // (us)	
	while(1)
	{
		/*-----------------------------------------------------------------*/
		/*-----------------------------------------------------------------*/
		/*-----------------------------------------------------------------*/
		/*------------ Algorithm section ----------------------------------*/
		if(Status_CheckStatus(&VehStt.Veh_Sample_Time))
		{
			PID_UpdateEnc(&M1,TIM_GetCounter(TIM3));
			PID_UpdateEnc(&M2,TIM_GetCounter(TIM4));
			GetVehicleVelocity();
			switch((int)Veh.Mode)
			{
				/*-------------- Auto mode section ------------------*/
				/*---------------------------------------------------*/
				case Auto_Mode:
					if(Status_CheckStatus(&VehStt.Veh_Auto_Flag))
					{
						if(Status_CheckStatus(&VehStt.GPS_Coordinate_Reveived))
						{
							Status_UpdateStatus(&VehStt.GPS_Coordinate_Reveived,Check_NOK);
							if((GPS_NEO.GPS_Quality == RTK_Fixed) || (GPS_NEO.GPS_Quality == RTK_Float))
							{
								OverWritePosition(&selfPosition,GPS_NEO.CorX,GPS_NEO.CorY);
								GPS_StanleyCompute();
							}
							else
							{
								if(Status_CheckStatus(&VehStt.GPS_SelfUpdatePosition_Flag) && Status_CheckStatus(&VehStt.GPS_FirstGetPosition))
								{
									SelfPositionUpdateParams(&selfPosition,M2.Current_Vel,M1.Current_Vel,Mag.Angle,Timer.T);
									GPS_UpdateCoordinateXY(&GPS_NEO,selfPosition.x,selfPosition.y);
									GPS_StanleyCompute();
								}
								else
								{
									PID_UpdateSetVel(&M1,0);
									PID_UpdateSetVel(&M2,0);
								}
							}
						}
						else if(Status_CheckStatus(&VehStt.GPS_FirstGetPosition) && Status_CheckStatus(&VehStt.GPS_SelfUpdatePosition_Flag))
						{
							SelfPositionUpdateParams(&selfPosition,M2.Current_Vel,M1.Current_Vel,Mag.Angle,Timer.T);
							GPS_UpdateCoordinateXY(&GPS_NEO,selfPosition.x,selfPosition.y);
							GPS_StanleyCompute();
						}
						else if((GPS_NEO.GPS_Quality == RTK_Fixed) || (GPS_NEO.GPS_Quality == RTK_Float))
						{
							GPS_UpdateNewCoordinates(&GPS_NEO, Timer.T);
							GPS_StanleyCompute();
						}
						
						// Case 1
						/*else if((GPS_NEO.GPS_Quality == RTK_Fixed) || (GPS_NEO.GPS_Quality == RTK_Float))
						{
							if(Mag.Fuzzy_Out >= 0)
							{
								PID_UpdateSetVel(&M1,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
								PID_UpdateSetVel(&M2,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
							}
							else
							{
								PID_UpdateSetVel(&M1,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
								PID_UpdateSetVel(&M2,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
							}
							if(Status_CheckStatus(&GPS_NEO.Goal_Flag))
							{
								PID_UpdateSetVel(&M1,0);
								PID_UpdateSetVel(&M2,0);
							}
						}*/
						else
						{
							PID_UpdateSetVel(&M1,0);
							PID_UpdateSetVel(&M2,0);
						}
					}
					else
					{
						PID_UpdateSetVel(&M1,0);
						PID_UpdateSetVel(&M2,0);
					}
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Run(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
					IMU_UpdatePreAngle(&Mag);
					GPS_NEO.Pre_CorX = GPS_NEO.CorX;
					GPS_NEO.Pre_CorY = GPS_NEO.CorY;
					break;
				
				/*--------------- Manual mode section --------------- -*/
				/*----------------------------------------------------*/
				/* Notes: This mode is uses for angle control test purposes */
				case Manual_Mode: 
					Veh_UpdateVehicleFromKey(&Veh);
					IMU_UpdateFuzzyInput(&Mag,&Timer.T);
					Defuzzification_Max_Min(&Mag);
					if(Mag.Fuzzy_Out >= 0)
					{
						PID_UpdateSetVel(&M1,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
						PID_UpdateSetVel(&M2,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					}
					else
					{
						PID_UpdateSetVel(&M1,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
						PID_UpdateSetVel(&M2,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					}
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Run(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
					IMU_UpdatePreAngle(&Mag);
					Veh.ManualCtrlKey = 0;
					break;
				
				/*--------------- Calibration section ------------------*/
				/*------------------------------------------------------*/
				case Calib_Mode:
					if(Status_CheckStatus(&VehStt.Veh_Timer_Finish))
					{
						if(Status_CheckStatus(&VehStt.Veh_Calib_Flag))
						{
							if(Veh.Distance < 39400)
							{
								Veh.Distance += (M2.Enc + (M2.OverFlow - 1) * 65535) - M2.PreEnc;
							}
							else
							{
								Veh.Distance = 0;
								if(Veh.TotalDistance < 20)
								{
									Veh.TotalDistance++;
								}
								else
								{
									Veh.TotalDistance = 0;
									PID_UpdateSetVel(&M1,0);
									PID_UpdateSetVel(&M2,0);
									Status_UpdateStatus(&VehStt.Veh_Calib_Flag,Check_NOK);
									Status_UpdateStatus(&VehStt.IMU_Calib_Finish,Check_OK);
								}
							}
						}
						if(Status_CheckStatus(&VehStt.IMU_Calib_Finish))
						{
							if((M1.Current_Vel == 0) && (M2.Current_Vel == 0))
							{
								U1_SendData(FeedBack(U1_TxBuffer,"$MAGST"));
								U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
								Status_UpdateStatus(&VehStt.IMU_Calib_Finish,Check_NOK);
							}
						}
						PID_Compute(&M1);
						PID_Compute(&M2);
						Robot_Run(M1.PID_Out,M2.PID_Out);
					}
					break;
				
				/*-------------- Test section --------------------------*/
				/*------------------------------------------------------*/
				case KeyBoard_Mode:
					//SelfPositionUpdateParams(&selfPosition,M2.Current_Vel/60,M1.Current_Vel/60,Timer.T);
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Run(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
					break;
				
				case Soft_Reset_Mode:
					if(Status_CheckStatus(&VehStt.Veh_Timer_Finish))
					{
						NVIC_SystemReset();
					}
					break;
			}
			PID_ResetEncoder(&M1);
			PID_ResetEncoder(&M2);
			Status_UpdateStatus(&VehStt.Veh_Sample_Time,Check_NOK);
		}
		
		
		/*-----------------------------------------------------------------*/
		/*-----------------------------------------------------------------*/
		/*-----------------------------------------------------------------*/
		/*--------- Send Data section -------------------------------------*/
		if(Status_CheckStatus(&VehStt.Veh_Send_Data))
		{
			if(Status_CheckStatus(&VehStt.Veh_Send_Parameters))
			{
				SendStatusData();
				Status_UpdateStatus(&VehStt.Veh_Send_Parameters,Check_NOK);
			}
			else
			{
				if(Status_CheckStatus(&VehStt.Veh_SendData_Flag))
				{
					SendData_0();
					while(!IsDataTransferCompleted());
					SendData_1();
					while(!IsDataTransferCompleted());
					SendData_2();
				}
			}
			Status_UpdateStatus(&VehStt.Veh_Send_Data,Check_NOK);
		}
	}
}




















