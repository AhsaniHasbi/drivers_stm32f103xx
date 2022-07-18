/*
 * stm32f103xx_gpio_driver.cpp
 *
 *  Created on: Jul 18, 2022
 *      Author: hasby
 */

#include <stm32f103xx_gpio_driver.h>

/*******************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 *******************************************************************************************/

/*
 * Peripheral Clock setup
 */

/*
 *************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 */

void GPIO_PeriClockControl(RCC_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if(pGPIO == GPIOB) {
			GPIOB_PCLK_EN();
		} else if(pGPIO == GPIOC) {
			GPIOC_PCLK_EN();
		} else if(pGPIO == GPIOD) {
			GPIOD_PCLK_EN();
		} else if(pGPIO == GPIOE) {
			GPIOE_PCLK_EN();
		} else if(pGPIO == GPIOF) {
			GPIOF_PCLK_EN();
		} else if(pGPIO == GPIOG) {
			GPIOG_PCLK_EN();
		}
	} else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if(pGPIO == GPIOB) {
			GPIOB_PCLK_DI();
		} else if(pGPIO == GPIOC) {
			GPIOC_PCLK_DI();
		} else if(pGPIO == GPIOD) {
			GPIOD_PCLK_DI();
		} else if(pGPIO == GPIOE) {
			GPIOE_PCLK_DI();
		} else if(pGPIO == GPIOF) {
			GPIOF_PCLK_DI();
		} else if(pGPIO == GPIOG) {
			GPIOG_PCLK_DI();
		}
	}
}

/*
 * Init and De-init
 */

/*
 *************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @note			- none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	//1. Configure the mode of GPIO pin.
	//2. Configure the speed.
	//3. Configure the pupd settings.
	//4. Configure the optype.
	//5. Configure the alt functionality
}

void GPIO_DeInit(GPIO_Handle_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


