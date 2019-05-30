#include "Srf05.h"

GPIO_InitTypeDef						Srf05_GPIO_Struct;
TIM_TimeBaseInitTypeDef			Srf05_TIM_TimeBaseStruct;
TIM_ICInitTypeDef						Srf05_TIM_ICStruct;
NVIC_InitTypeDef						Srf05_NVIC_Struct;
void Core_Delay_Us(uint16_t count)
{
	count *= 25;
	while(count--);
}

/** @Brief: Parameters init for sensor
**	@Args : Sensor variables structure, GPIOx, TIMx, Trigger input pin to start the sensor and Output pin trigger of sensor
**	@Ret	: None
**/
void	Srf05_Initial(Srf05_InitTypeDef *psrf05)
{
	GetGPIOxClockCmd(psrf05->Srf05_GPIO_Trigger,ENABLE);
	GetGPIOxClockCmd(psrf05->Srf05_GPIO_Echo,ENABLE);
	/* Config GPIO pin */
	Srf05_GPIO_Struct.GPIO_Pin									= psrf05->Srf05_Trigger_Pin;
	Srf05_GPIO_Struct.GPIO_Mode									= GPIO_Mode_OUT;
	Srf05_GPIO_Struct.GPIO_OType								= GPIO_OType_PP;
	Srf05_GPIO_Struct.GPIO_PuPd									= GPIO_PuPd_NOPULL;
	Srf05_GPIO_Struct.GPIO_Speed								= GPIO_Speed_50MHz;
	GPIO_Init(psrf05->Srf05_GPIO_Trigger,&Srf05_GPIO_Struct);
	
	Srf05_GPIO_Struct.GPIO_Mode									= GPIO_Mode_AF;
	Srf05_GPIO_Struct.GPIO_Pin									= psrf05->Srf05_Echo_Pin;
	Srf05_GPIO_Struct.GPIO_Speed								= GPIO_Speed_50MHz;
	GPIO_Init(psrf05->Srf05_GPIO_Echo,&Srf05_GPIO_Struct);
	
	GPIO_PinAFConfig(psrf05->Srf05_GPIO_Echo,GetPinSourceFromGPIOPin(psrf05->Srf05_Echo_Pin),GetAFFromTIM(psrf05->Srf05_ICTIM));
	
	GetTIMxClockCmd(psrf05->Srf05_ICTIM,ENABLE);
	/* Config time base struct */
	Srf05_TIM_TimeBaseStruct.TIM_Prescaler			= 100 - 1;
	Srf05_TIM_TimeBaseStruct.TIM_Period					= 0xFFFFFFFF;
	Srf05_TIM_TimeBaseStruct.TIM_ClockDivision 	= TIM_CKD_DIV1;
	Srf05_TIM_TimeBaseStruct.TIM_CounterMode 	  = TIM_CounterMode_Up;
	TIM_TimeBaseInit(psrf05->Srf05_ICTIM,&Srf05_TIM_TimeBaseStruct);
	
	Srf05_TIM_ICStruct.TIM_Channel							= psrf05->Srf05_IC_Channel;
	Srf05_TIM_ICStruct.TIM_ICPolarity						= TIM_ICPolarity_BothEdge;
	Srf05_TIM_ICStruct.TIM_ICPrescaler					= TIM_ICPSC_DIV1;
	Srf05_TIM_ICStruct.TIM_ICSelection					= TIM_ICSelection_DirectTI;
	TIM_ICInit(psrf05->Srf05_ICTIM,&Srf05_TIM_ICStruct);
	
	
	Srf05_NVIC_Struct.NVIC_IRQChannel						= GetIRQHandlerFromTIMxCC(psrf05->Srf05_ICTIM);
	Srf05_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = psrf05->Srf05_PreemptionPriority;
	Srf05_NVIC_Struct.NVIC_IRQChannelSubPriority = psrf05->Srf05_SubPriority;
	Srf05_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&Srf05_NVIC_Struct);
	
	TIM_ITConfig(psrf05->Srf05_ICTIM,psrf05->Srf05_TIM_IT_CC,ENABLE);
	TIM_Cmd(psrf05->Srf05_ICTIM,ENABLE);
}

void	Srf05_ResetCounter(TIM_TypeDef *TIMx)
{
	TIM_SetCounter(TIMx,0);
}

void	Srf05_SetFlag(Srf05_Data *psrf05)
{
	psrf05->GetData_Flag = DUNG;
}

void	Srf05_ClearFlag(Srf05_Data *psrf05)
{
	psrf05->GetData_Flag = SAI;
}

void	Srf05_FirstInit(Srf05_Data *psrf05)
{
	psrf05->GetData_Flag = SAI;
	psrf05->PulseWidth	 = 0;
	psrf05->Distance		 = 0;
}





