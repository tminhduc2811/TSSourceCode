#include "stm32f4xx.h"

typedef struct USARTDMA_TypeDef{
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
}USARTDMA_TypeDef;	




































