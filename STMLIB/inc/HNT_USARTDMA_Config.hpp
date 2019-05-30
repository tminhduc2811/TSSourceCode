#include "stm32f4xx.h"

/* Type define */
#define									MaxRx_BufferSize 			200
#define									MaxTx_BufferSize			200
typedef struct USARTDMA_Config{
	/* USART and Pin AF configuration */
	USART_TypeDef					*USARTx;
	GPIO_TypeDef					*GPIOx;
	uint16_t							GPIO_Pin_Tx;
	uint16_t							GPIO_Pin_Rx;
	
	/* Size of rx and tx buffer size, baudrat */
	unsigned int					RxSize;
	unsigned int 					TxSize;
	uint32_t							BaudRate;
	/* Buffer for send and receive through DMA */
	uint8_t								RxBuffer[MaxRx_BufferSize];
	uint8_t								TxBuffer[MaxTx_BufferSize];
}USARTDMA_Config;	




































