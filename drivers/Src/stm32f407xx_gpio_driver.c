/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 19, 2023
 *      Author: shrey
 */

#include"stm32f407xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*********************************************************************
 * @fn							- GPIO_PeriClockControl
 *
 * @brief						- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					- ENABLE or DISABLE macros
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOA_PCLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOA_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/*********************************************************************
 * @fn							- GPIO_Init
 *
 * @brief						- This function initializes the GPIO port
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. Configure the mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the bit
			pGPIOHandle->pGPIOx->MODER |= temp; //setting the bit
			temp = 0;
		}else
		{
			//interupt
		}
		temp = 0;
	//2. Configure the speed
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the bit
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;

	//3. Configure the pupd setting
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the bit
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp = 0;

	//4. Configure the optype
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing the bit
		pGPIOHandle->pGPIOx->OTYPER |= temp;
		temp = 0;

	//5. Configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
		{
			//configure alternate function register
			uint8_t temp1, temp2;

			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2)); //clearing the bit
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
		}

}

/*********************************************************************
 * @fn							- GPIO_DeInit
 *
 * @brief						- This function deinitializes the GPIO port
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOA_REG_RESET();
	}
}

/*
 * Data Read and Write
 */

/*********************************************************************
 * @fn							- GPIO_ReadFromInputPin
 *
 * @brief						- This function allows reading from the specific GPIO pin
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					- pin number
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- value
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @fn							- GPIO_ReadFromInputPort
 *
 * @brief						- This function allows reading from the specific GPIO port
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*********************************************************************
 * @fn							- GPIO_WriteToOutputPin
 *
 * @brief						- This function writes values to the specific GPIO pin
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					- pin number
 * @param[in]					- value
 *
 * @return						- none
 *
 * @Note						- value
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register at the bit field to corresponding pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		//write 0 to the output data register at the bit field to corresponding pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*********************************************************************
 * @fn							- GPIO_WriteToOutputPort
 *
 * @brief						- This function allows writes values to the specific GPIO port
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					- value
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
{
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn							- GPIO_ToggleOutputPin
 *
 * @brief						- This function allows pin to toggle
 *
 * @param[in]					- base address of gpio peripheral
 * @param[in]					- pin number
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */

/*********************************************************************
 * @fn							- GPIO_IRQConfig
 *
 * @brief						- This function is used to configure the ISR handeling
 *
 * @param[in]					- number of IRQ
 * @param[in]					- level of priority
 * @param[in]					- ENABLE or DISABLE Macros
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn							- GPIO_IRQHandling
 *
 * @brief						- This function is used for handeling the IRQ
 *
 * @param[in]					- Pin number
 * @param[in]					-
 * @param[in]					-
 *
 * @return						- none
 *
 * @Note						- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
