#include "USART_Config.h"

void USART_Config(USART_VariablesInit *pusart)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(U1_RCC_AHB1Periph_GPIOx,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

	US_GPIO_Struct.GPIO_Pin 					= U1_GPIO_Pin_Tx|U1_GPIO_Pin_Rx;
	US_GPIO_Struct.GPIO_Mode					= GPIO_Mode_AF;
	US_GPIO_Struct.GPIO_OType					= GPIO_OType_PP;
	US_GPIO_Struct.GPIO_PuPd 					= GPIO_PuPd_UP;
	US_GPIO_Struct.GPIO_Speed					= GPIO_Speed_50MHz;
	GPIO_Init(U1_GPIOx,&US_GPIO_Struct);
	
	GPIO_PinAFConfig(U1_GPIOx,U1_GPIO_PinSourceTx,GPIO_AF_USART1);
	GPIO_PinAFConfig(U1_GPIOx,U1_GPIO_PinSourceRx,GPIO_AF_USART1);
	
	//Config USART Tx 
	US_USART_Struct.USART_BaudRate 							= BaudRate;
	US_USART_Struct.USART_Mode 									= USART_Mode_Tx|USART_Mode_Rx;
	US_USART_Struct.USART_WordLength 						= USART_WordLength_8b;
	US_USART_Struct.USART_Parity								= USART_Parity_No;
	US_USART_Struct.USART_StopBits 							= USART_StopBits_1;
	US_USART_Struct.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;
	USART_Init(USART1,&US_USART_Struct);
	USART_Cmd(USART1,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	
	//--------Config DMA USART Tx----------
	US_DMA_Struct.DMA_Channel									= DMA_Channel_4;
	US_DMA_Struct.DMA_BufferSize							= 100;
	US_DMA_Struct.DMA_Mode 										= DMA_Mode_Normal;
	US_DMA_Struct.DMA_DIR											= DMA_DIR_MemoryToPeripheral;
	US_DMA_Struct.DMA_Memory0BaseAddr					= (uint32_t)&U1_TxBuffer;
	US_DMA_Struct.DMA_MemoryBurst 						= DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize					= DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc								= DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr 			= (uint32_t)&USART1->DR;
	US_DMA_Struct.DMA_PeripheralBurst					=	DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize			= DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc						= DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode								= DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold						= DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority 								= DMA_Priority_High;
	DMA_Init(DMA2_Stream7,&US_DMA_Struct);
	//-----------Config DMA USART Rx------------------
	US_DMA_Struct.DMA_Channel									= DMA_Channel_4;
	US_DMA_Struct.DMA_BufferSize							= 47;
	US_DMA_Struct.DMA_Mode 										= DMA_Mode_Normal;
	US_DMA_Struct.DMA_DIR											= DMA_DIR_PeripheralToMemory;
	US_DMA_Struct.DMA_Memory0BaseAddr					= (uint32_t)&U1_RxBuffer;
	US_DMA_Struct.DMA_MemoryBurst 						= DMA_MemoryBurst_Single;
	US_DMA_Struct.DMA_MemoryDataSize					= DMA_MemoryDataSize_Byte;
	US_DMA_Struct.DMA_MemoryInc								= DMA_MemoryInc_Enable;
	US_DMA_Struct.DMA_PeripheralBaseAddr 			= (uint32_t)&USART1->DR;
	US_DMA_Struct.DMA_PeripheralBurst					=	DMA_PeripheralBurst_Single;
	US_DMA_Struct.DMA_PeripheralDataSize			= DMA_PeripheralDataSize_Byte;
	US_DMA_Struct.DMA_PeripheralInc						= DMA_PeripheralInc_Disable;
	US_DMA_Struct.DMA_FIFOMode								= DMA_FIFOMode_Disable;
	US_DMA_Struct.DMA_FIFOThreshold						= DMA_FIFOThreshold_HalfFull;
	US_DMA_Struct.DMA_Priority 								= DMA_Priority_High;
	DMA_Init(DMA2_Stream5,&US_DMA_Struct);
	DMA_Cmd(DMA2_Stream5,ENABLE);
	//------------NVIC Config----------
	US_NVIC_Struct.NVIC_IRQChannel 										= DMA2_Stream5_IRQn;
	US_NVIC_Struct.NVIC_IRQChannelPreemptionPriority 	= 0;
	US_NVIC_Struct.NVIC_IRQChannelSubPriority					= 2;
	US_NVIC_Struct.NVIC_IRQChannelCmd									= ENABLE;
	NVIC_Init(&US_NVIC_Struct);
	DMA_ITConfig(DMA2_Stream5,DMA_IT_TC,ENABLE);
}







































