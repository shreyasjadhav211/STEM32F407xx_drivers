/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Feb 19, 2023
 *      Author: shrey
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber ;			/* possible values from @GPIO_PIN_NO*/
	uint8_t GPIO_PinMode ;    			/* possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed ;				/* possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPuPdControl ;		/* possible values from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType ;			/* possible values from @GPIO_PIN_OUTPUT*/
	uint8_t GPIO_PinAltFunMode ;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO Pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_INPUT 				0
#define GPIO_MODE_OUTPUT 				1
#define GPIO_MODE_ALTFN 				2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_IT_FT					4
#define GPIO_MODE_IT_RT					5
#define GPIO_MODE_IT_RFT				6


/*
 * @GPIO_PIN_OUTPUT
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP					0
#define GPIO_OP_TYPE_OD					1

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speed
 */
#define GPIO_SPEED_LOW 					0
#define GPIO_SPEED_MEDIUM 				1
#define GPIO_SPEED_FAST 				2
#define GPIO_MODE_HIGH	 				3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD 					0
#define GPIO_PU							1
#define GPIO_PD 						2

/*
 * @GPIO_PIN_NO
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0					0
#define GPIO_PIN_NO_1					1
#define GPIO_PIN_NO_2					2
#define GPIO_PIN_NO_3					3
#define GPIO_PIN_NO_4					4
#define GPIO_PIN_NO_5					5
#define GPIO_PIN_NO_6					6
#define GPIO_PIN_NO_7					7
#define GPIO_PIN_NO_8					8
#define GPIO_PIN_NO_9					9
#define GPIO_PIN_NO_10					10
#define GPIO_PIN_NO_11					11
#define GPIO_PIN_NO_12					12
#define GPIO_PIN_NO_13					13
#define GPIO_PIN_NO_14					14
#define GPIO_PIN_NO_15					15

/********************************************************************************************
 * 							APIs supported by this driver
 *
 *
 ********************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


















#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
