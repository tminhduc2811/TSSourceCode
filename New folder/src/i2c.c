#include "i2c.h"

GPIO_InitTypeDef		GPIO_Struct;
I2C_InitTypeDef			I2C_Struct;
DMA_InitTypeDef			DMA_Struct;
NVIC_InitTypeDef		NVIC_Struct;

void	EnableGPIORCCClock(GPIO_TypeDef *GPIOx)
{
	if(GPIOx == GPIOA)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	else if(GPIOx == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	else if(GPIOx == GPIOC)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	else if(GPIOx == GPIOD)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	else if(GPIOx == GPIOE)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	else if(GPIOx == GPIOF)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	else if(GPIOx == GPIOG)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	else if(GPIOx == GPIOH)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
	else if(GPIOx == GPIOI)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	else if(GPIOx == GPIOJ)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ,ENABLE);
}

void EnableI2CRCCClock(I2C_TypeDef *I2Cx)
{
	if(I2Cx == I2C1)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	else if(I2Cx == I2C2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	else if(I2Cx == I2C3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,ENABLE);
}

void EnableDMARCCClock(DMA_TypeDef *DMAx)
{
	if(DMAx == DMA1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	else if(DMAx == DMA2)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
}

void I2C_ParametersInit(GPIO_TypeDef *GPIOx, uint16_t GPIO_SDA_Pin, uint16_t GPIO_SCL_Pin, I2C_TypeDef *I2Cx, DMA_TypeDef *DMAx)
{
	EnableGPIORCCClock(GPIOx);
	EnableI2CRCCClock(I2Cx);
	EnableDMARCCClock(DMAx);
	/* Config GPIO pin */
	GPIO_Struct.GPIO_Mode									= GPIO_Mode_AF;
	GPIO_Struct.GPIO_Pin									= GPIO_SDA_Pin|GPIO_SCL_Pin;
	GPIO_Struct.GPIO_PuPd									= GPIO_PuPd_NOPULL;
	GPIO_Struct.GPIO_Speed								= GPIO_Speed_50MHz;
	GPIO_Init(GPIOx,&GPIO_Struct);
	
	//GPIO_PinAFConfig(GPIOx,GPIO_PinSource
}






























