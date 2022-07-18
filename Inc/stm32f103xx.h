/*
 * stm32f101xx.h
 *
 *  Created on: Jul 17, 2022
 *      Author: hasby
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>

#define _vo volatile

/*
 * base address of flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM_BASEADDR			0x20000000U
#define ROM_BASEADDR			0x1FFFF000U
#define SRAM 					SRAM_BASEADDR

/*
 * AHB and APBx Bus Peripheral base address
 */

#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHBPERIPH_BASE			0x40018000U

/*
 * Base addresses of peripherals which are hanging on AHB
 * TODO : Complete for all other peripherals
 */
#define SDIO_BASEADDR			(AHBPERIPH_BASE + 0x0000U)
//DMA
#define DMA1_BASEADDR			(AHBPERIPH_BASE + 0x8000U)
#define DMA2_BASEADDR			(AHBPERIPH_BASE + 0x8400U)
//RCC
#define RCC_BASEADDR			(AHBPERIPH_BASE + 0x9000U)
//flash memory interface
#define FMI_BASEADDR			(AHBPERIPH_BASE + 0xA000U)
//CRC
#define CRC_BASEADDR			(AHBPERIPH_BASE + 0xB000U)
//ETHERNET
#define ETHERNET_BASEADDR		(AHBPERIPH_BASE + 0x10000U)
//USB OTG FS
#define USBOTGFS_BASEADDR		(AHBPERIPH_BASE + 0xFFE8000U)
//FSMC
#define FSMC_BASEADDR			(AHBPERIPH_BASE + 0x5FFE8000U)


/*
 * Base addresses of peripherals which are hanging on APB1
 * TODO : Complete for all other peripherals
 */
//TIMER
#define TIM2_BASEADDR 			(APB1PERIPH_BASE + 0x0000U)
#define TIM3_BASEADDR 			(APB1PERIPH_BASE + 0x0400U)
#define TIM4_BASEADDR 			(APB1PERIPH_BASE + 0x0800U)
#define TIM5_BASEADDR 			(APB1PERIPH_BASE + 0x0C00U)
#define TIM6_BASEADDR 			(APB1PERIPH_BASE + 0x1000U)
#define TIM7_BASEADDR 			(APB1PERIPH_BASE + 0x1400U)
#define TIM12_BASEADDR 			(APB1PERIPH_BASE + 0x1800U)
#define TIM13_BASEADDR 			(APB1PERIPH_BASE + 0x1C00U)
#define TIM14_BASEADDR 			(APB1PERIPH_BASE + 0x2000U)
//SPI
#define SPI2_BASEADDR 			(APB1PERIPH_BASE + 0x3800U)
#define SPI3_BASEADDR 			(APB1PERIPH_BASE + 0x3C00U)
//I2C
#define I2C1_BASEADDR 			(APB1PERIPH_BASE + 0x5400U)
#define I2C2_BASEADDR 			(APB1PERIPH_BASE + 0x5800U)
//RTC, WWDG, IWDG, PWR, DAC, BKP
#define RTC_BASEADDR			(APB1PERIPH_BASE + 0x2800U)
#define WWDG_BASEADDR			(APB1PERIPH_BASE + 0x2C00U)
#define IWDG_BASEADDR			(APB1PERIPH_BASE + 0x3000U)
#define PWR_BASEADDR			(APB1PERIPH_BASE + 0x7000U)
#define DAC_BASEADDR			(APB1PERIPH_BASE + 0x7400U)
#define BKP_BASEADDR			(APB1PERIPH_BASE + 0x6C00U)
//USART
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400U)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800U)
//UART
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00U)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000U)

/*
 * Base addresses of peripherals which are hanging on APB2
 * TODO : Complete for all other peripherals
 */
//GPIO
#define GPIOA_BASEADDR			(APB2PERIPH_BASE + 0x0800U)
#define GPIOB_BASEADDR			(APB2PERIPH_BASE + 0x0C00U)
#define GPIOC_BASEADDR			(APB2PERIPH_BASE + 0x1000U)
#define GPIOD_BASEADDR			(APB2PERIPH_BASE + 0x1400U)
#define GPIOE_BASEADDR			(APB2PERIPH_BASE + 0x1800U)
#define GPIOF_BASEADDR			(APB2PERIPH_BASE + 0x1C00U)
#define GPIOG_BASEADDR			(APB2PERIPH_BASE + 0x2000U)
//TIMER
#define TIM1_BASEADDR			(APB2PERIPH_BASE + 0x2C00U)
#define TIM8_BASEADDR			(APB2PERIPH_BASE + 0x3400U)
#define TIM9_BASEADDR			(APB2PERIPH_BASE + 0x4C00U)
#define TIM10_BASEADDR			(APB2PERIPH_BASE + 0x5000U)
#define TIM11_BASEADDR			(APB2PERIPH_BASE + 0x5400U)
//ADC
#define ADC1_BASEADDR			(APB2PERIPH_BASE + 0x2400U)
#define ADC2_BASEADDR			(APB2PERIPH_BASE + 0x2800U)
#define ADC3_BASEADDR			(APB2PERIPH_BASE + 0x3C00U)
//SPI
#define SPI1_BASEADDR 			(APB1PERIPH_BASE + 0x3000U)
//AFIO, EXTI
#define AFIO_BASEADDR			(APB2PERIPH_BASE + 0x0000U)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x0400U)
//USART
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x3800U)

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of registers of SPI peripheral of STM32f103xx family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32f103xx family MCUs
 * Please check your device RM
 */
//GPIO
typedef struct {
	_vo uint32_t CRL;			/*!<Port configuration register low>			Address offset 0x00 */
	_vo uint32_t CRH;			/*!<Port configuration register high>			Address offset 0x04 */
	_vo uint32_t IDR;			/*!<Port input data register>					Address offset 0x08 */
	_vo uint32_t ODR;			/*!<Port output data register>					Address offset 0x0C */
	_vo uint32_t BSRR;			/*!<Port bit set/reset register>				Address offset 0x10 */
	_vo uint32_t BRR;			/*!<Port bit reset register>					Address offset 0x14 */
	_vo uint32_t LCKR;			/*!<Port configuration lock register>			Address offset 0x18 */
} GPIO_RegDef_t ;

//RCC
typedef struct {
	_vo uint32_t RCC_CR;
	_vo uint32_t RCC_CFGR;
	_vo uint32_t RCC_CIR;
	_vo uint32_t RCC_APB2RSTR;
	_vo uint32_t RCC_APB1RSTR;
	_vo uint32_t RCC_AHBENR;
	_vo uint32_t RCC_APB2ENR;
	_vo uint32_t RCC_APB1ENR;
	_vo uint32_t RCC_BDCR;
	_vo uint32_t RCC_CSR;
} RCC_RegDef_t;

/*
 * Peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
 */
//GPIO
#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
//RCC
#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * =====================================================================
 * NOTE : Enable macros peripheral
 * =====================================================================
 */

/*
 * Clock enable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 8))

/*
 * Clock enable macros for ADCx peripheral
 */
#define ADC1_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 9))
#define ADC2_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 10))
#define ADC3_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 15))

/*
 * Clock enable macros for TIMx peripheral
 */
#define TIM1_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 11))
#define TIM2_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 3))
#define TIM6_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 5))
#define TIM8_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 13))
#define TIM9_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 19))
#define TIM10_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 20))
#define TIM11_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 21))
#define TIM12_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 6))
#define TIM13_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 7))
#define TIM14_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 8))

/*
 * Clock enable macros for WWDG peripheral
 */
#define WWDG_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 11))

/*
 * Clock enable macros for I2Cx peripheral
 */
#define I2C1_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 22))

/*
 * Clock enable macros for USARTx peripheral
 */
#define USART1_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 18))
#define USART4_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 19))
#define USART5_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 20))

/*
 * Clock enable macros for SPIx peripheral
 */
#define SPI1_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 15))

/*
 * Clock enable macros for AFIO peripheral
 */
#define AFIO_PCLK_EN()				(RCC->RCC_APB2ENR |= (1 << 0))

/*
 * Clock enable macros for DMAx peripheral
 */
#define DMA1_PCLK_EN()				(RCC->RCC_AHBENR |= (1 << 0))
#define DMA2_PCLK_EN()				(RCC->RCC_AHBENR |= (1 << 1))

/*
 * Clock enable macros for SRAM peripheral
 */
#define SRAM_PCLK_EN()				(RCC->RCC_AHBENR |= (1 << 2))

/*
 * Clock enable macros for FLITF peripheral
 */
#define FLITF_PCLK_EN()				(RCC->RCC_AHBENR |= (1 << 4))

/*
 * Clock enable macros for CRCE peripheral
 */
#define CRCE_PCLK_EN()				(RCC->RCC_AHBENR |= (1 << 6))

/*
 * Clock enable macros for FSMC peripheral
 */
#define FSMC_PCLK_EN()				(RCC->RCC_AHBENR |= (1 << 8))

/*
 * Clock enable macros for SDIO peripheral
 */
#define SDIO_PCLK_EN()				(RCC->RCC_AHBENR |= (1 << 10))

/*
 * Clock enable macros for USB peripheral
 */
#define USB_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 23))

/*
 * Clock enable macros for CAN peripheral
 */
#define CAN_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 25))

/*
 * Clock enable macros for BKP peripheral
 */
#define BKP_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 27))

/*
 * Clock enable macros for PWR peripheral
 */
#define PWR_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 28))

/*
 * Clock enable macros for DAC peripheral
 */
#define DAC_PCLK_EN()				(RCC->RCC_APB1ENR |= (1 << 29))

/*
 * =====================================================================
 * NOTE : Disable macros peripheral
 * =====================================================================
 */

/*
 * Clock Disable macros for GPIOx peripheral
 */
#define GPIOA_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 8))

/*
 * Clock enable macros for ADCx peripheral
 */
#define ADC1_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 9))
#define ADC2_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 10))
#define ADC3_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 15))

/*
 * Clock enable macros for TIMx peripheral
 */
#define TIM1_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 11))
#define TIM2_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 3))
#define TIM6_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 4))
#define TIM7_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 5))
#define TIM8_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 13))
#define TIM9_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 19))
#define TIM10_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 20))
#define TIM11_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 21))
#define TIM12_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 6))
#define TIM13_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 7))
#define TIM14_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 8))

/*
 * Clock enable macros for WWDG peripheral
 */
#define WWDG_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 11))

/*
 * Clock enable macros for I2Cx peripheral
 */
#define I2C1_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 22))

/*
 * Clock enable macros for USARTx peripheral
 */
#define USART1_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 18))
#define USART4_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 19))
#define USART5_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 20))

/*
 * Clock enable macros for SPIx peripheral
 */
#define SPI1_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 15))

/*
 * Clock enable macros for AFIO peripheral
 */
#define AFIO_PCLK_DI()				(RCC->RCC_APB2ENR &= ~(1 << 0))

/*
 * Clock enable macros for DMAx peripheral
 */
#define DMA1_PCLK_DI()				(RCC->RCC_AHBENR &= ~(1 << 0))
#define DMA2_PCLK_DI()				(RCC->RCC_AHBENR &= ~(1 << 1))

/*
 * Clock enable macros for SRAM peripheral
 */
#define SRAM_PCLK_DI()				(RCC->RCC_AHBENR &= ~(1 << 2))

/*
 * Clock enable macros for FLITF peripheral
 */
#define FLITF_PCLK_DI()				(RCC->RCC_AHBENR &= ~(1 << 4))

/*
 * Clock enable macros for CRCE peripheral
 */
#define CRCE_PCLK_DI()				(RCC->RCC_AHBENR &= ~(1 << 6))

/*
 * Clock enable macros for FSMC peripheral
 */
#define FSMC_PCLK_DI()				(RCC->RCC_AHBENR &= ~(1 << 8))

/*
 * Clock enable macros for SDIO peripheral
 */
#define SDIO_PCLK_DI()				(RCC->RCC_AHBENR &= ~(1 << 10))

/*
 * Clock enable macros for USB peripheral
 */
#define USB_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 23))

/*
 * Clock enable macros for CAN peripheral
 */
#define CAN_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 25))

/*
 * Clock enable macros for BKP peripheral
 */
#define BKP_PCLK_DI()					(RCC->RCC_APB1ENR &= ~(1 << 27))

/*
 * Clock enable macros for PWR peripheral
 */
#define PWR_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 28))

/*
 * Clock enable macros for DAC peripheral
 */
#define DAC_PCLK_DI()				(RCC->RCC_APB1ENR &= ~(1 << 29))

/**********************************************************************************
 * Some generic macros
 **********************************************************************************/
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET



#endif /* INC_STM32F103XX_H_ */
