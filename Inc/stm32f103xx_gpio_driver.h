/*
 * stm32f103xx_gpio_drivers.h
 *
 *  Created on: Jul 18, 2022
 *      Author: hasby
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_


#include <stm32f103xx.h>

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/*
 * This is Handle structure for a GPIO pin
 */
typedef struct {
	RCC_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*******************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 *******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(RCC_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
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


/************************************************
 * GPIO pin possible modes
 ************************************************/

#define GPIO_MODE_IN				0
#define GPIO_MODE_OUT_10			1
#define GPIO_MODE_OUT_2				2
#define GPIO_MODE_OUT_50			3

/*
 * INPUT MODE
 */

#define GPIO_MODE_IN_ANALOG			0	// analog mode
#define GPIO_MODE_IN_FLOAT			1	// floating input
#define GPIO_MODE_IN_PUPD			2	// pull down / pull up

/*
 * OUTPUT MODE
 */

#define GPIO_MODE_OUT_PP		0 	// General purpose output push-pull
#define GPIO_MODE_OUT_OD		1 	// General purpose output open-drain
#define GPIO_MODE_OUT_AFPP		2	// Alternate function output push-pull
#define GPIO_MODE_OUT_AFOD		3	// Alternate function output open-drain




#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
