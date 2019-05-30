#include "stm32f4xx.h"
#include <math.h>
// Control 2 wheels vehicle 
/* Define */
/* Hardware config */
/*----- Hardware Encoder config M1 ------*/
#define 	  		M1_TIMx												TIM3
#define					M1_GPIOx											GPIOA
#define					M1_RCC_PeriphClock						RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE)
#define					M1_RCC_AHB1Periph_GPIOx				RCC_AHB1Periph_GPIOA
#define					M1_GPIO_Pin_x1								GPIO_Pin_6
#define					M1_GPIO_Pin_x2								GPIO_Pin_7
#define					M1_GPIO_AF_TIMx								GPIO_AF_TIM3
#define					M1_GPIO_PinSourcex1						GPIO_PinSource6
#define					M1_GPIO_PinSourcex2						GPIO_PinSource7
#define					M1_TIMx_IRQn									TIM3_IRQn
/*----- Hardware Encoder config M2 ------*/
#define 	  		M2_TIMx												TIM4
#define					M2_GPIOx											GPIOD
#define					M2_RCC_PeriphClock						RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE)
#define					M2_RCC_AHB1Periph_GPIOx				RCC_AHB1Periph_GPIOD
#define					M2_GPIO_Pin_x1								GPIO_Pin_12
#define					M2_GPIO_Pin_x2								GPIO_Pin_13
#define					M2_GPIO_AF_TIMx								GPIO_AF_TIM4
#define					M2_GPIO_PinSourcex1						GPIO_PinSource12
#define					M2_GPIO_PinSourcex2						GPIO_PinSource13
#define					M2_TIMx_IRQn									TIM4_IRQn
/*----- Hardware PWM config for Motor 1 -*/
#define					PWM_TIMx											TIM9
#define					PWM_GPIOx											GPIOA
#define					PWM_GPIO_Pin_OC1							GPIO_Pin_2
#define					PWM_GPIO_Pin_OC2							GPIO_Pin_3
#define					PWM_GPIO_AF_TIMx							GPIO_AF_TIM9
#define					PWM_GPIO_PinSourceOC1					GPIO_PinSource2
#define					PWM_GPIO_PinSourceOC2					GPIO_PinSource3
#define					PWM_RCC_PeriphClock						RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE)
#define					PWM_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOA
/*----- Direction Pin  -----------------*/
#define					Dir_GPIOx											GPIOC
#define					Dir_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOC
#define					Dir_GPIO_Pin_M1								GPIO_Pin_3	      //Dir pin for M1
#define					Dir_GPIO_Pin_M2								GPIO_Pin_4				//Dir pin for M2
/*----- Parameter define ----------------*/
#define         Frequency_20KHz							  5000
/* Data Types */
/* Export Functions */
void M1_Forward(void);
void M1_Backward(void);
void M2_Forward(void);
void M2_Backward(void);
void Stop_Motor(void);
void Robot_Forward(void);
void Robot_Backward(void);
void Robot_Clockwise(void);
void Robot_AntiClockwise(void);
void Robot_Run(double duty_right, double duty_left);
void Encoder_Config(void);






















































