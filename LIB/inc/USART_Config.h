#include "stm32f4xx.h"

#define			Max_RxBuffer									500
#define			Max_TxBuffer									500
#define 		Declare_Variable(type,name)   									type	USART_##name
#define			Declare_pVariable(type,name)  	  							type	*USART_##name
#define			Declare_BufferVariables(type,name,Max_Buffer)		type	USART_##name[Max_Buffer]
#define			Get_Variable(name)	USART_##name
typedef struct USART_VariablesInit{
	Declare_pVariable(USART_TypeDef, x);
	//RCC_Clock
	Declare_Variable(uint32_t, USARTPeriphClock);
	Declare_Variable(uint32_t, DMAPeriphClock);
	Declare_Variable(uint32_t, GPIOPeriphClock);
	//GPIO rx and tx config
	Declare_pVariable(GPIO_TypeDef, GPIOx);
	Declare_Variable(uint16_t, GPIO_Pin_Tx);
	Declare_Variable(uint16_t, GPIO_Pin_Rx);
	Declare_Variable(uint16_t, GPIO_PinSource_Tx);
	Declare_Variable(uint16_t, GPIO_PinSource_Rx);
	Declare_Variable(uint8_t, GPIO_AF_Tx);
	Declare_Variable(uint8_t, GPIO_AF_Rx);
	Declare_Variable(uint32_t, BaudRate);
	//Interrupt config

}USART_VariablesInit;

/* Functions */
void USART_Config(USART_VariablesInit *pusart);




























