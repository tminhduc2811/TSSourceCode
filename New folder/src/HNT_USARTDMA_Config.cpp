#include "HNT_USARTDMA_Config.h"


void RCC_USARTEnable(USART_TypeDef *USARTx)
{
	
}

void RCC_GPIOEnable(GPIO_TypeDef	*GPIOx)
{
	
}

void RCC_DMAEnable(DMA_TypeDef *DMAx)
{
	
}

class USART
{	
	public:
		uint8_t		RxBuffer[200];
		uint8_t		TxBuffer[200];
		
		void USART_DMA_Enable(USART_TypeDef *USARTx, GPIO_TypeDef *GPIOx, uint16_t TxPin, uint16_t RxPin)
		{
			RCC_GPIOEnable(GPIOx);
		}
		
		
};




