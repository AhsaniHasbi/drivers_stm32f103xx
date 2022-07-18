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

#define DRV_FLASH_BASEADDR			0x08000000U
#define DRV_SRAM_BASEADDR			0x20000000U
#define DRV_ROM_BASEADDR			0x1FFFF000U
#define DRV_SRAM 					SRAM_BASEADDR

/*
 * AHB and APBx Bus Peripheral base address
 */

#define DRV_PERIPH_BASE				0x40000000U
#define DRV_APB1PERIPH_BASE			PERIPH_BASE
#define DRV_APB2PERIPH_BASE			0x40010000U
#define DRV_AHBPERIPH_BASE			0x40018000U

/*
 * Base addresses of peripherals which are hanging on AHB
 * TODO : Complete for all other peripherals
 */
#define DRV_SDIO_BASEADDR			(DRV_AHBPERIPH_BASE + 0x0000U)
//DMA
#define DRV_DMA1_BASEADDR			(DRV_AHBPERIPH_BASE + 0x8000U)
#define DRV_DMA2_BASEADDR			(DRV_AHBPERIPH_BASE + 0x8400U)
//RCC
#define DRV_RCC_BASEADDR			(DRV_AHBPERIPH_BASE + 0x9000U)
//flash memory interface
#define DRV_FMI_BASEADDR			(DRV_AHBPERIPH_BASE + 0xA000U)
//CRC
#define DRV_CRC_BASEADDR			(DRV_AHBPERIPH_BASE + 0xB000U)
//ETHERNET
#define DRV_ETHERNET_BASEADDR		(DRV_AHBPERIPH_BASE + 0x10000U)
//USB OTG FS
#define DRV_USBOTGFS_BASEADDR		(DRV_AHBPERIPH_BASE + 0xFFE8000U)
//FSMC
#define DRV_FSMC_BASEADDR			(DRV_AHBPERIPH_BASE + 0x5FFE8000U)


/*
 * Base addresses of peripherals which are hanging on APB1
 * TODO : Complete for all other peripherals
 */
//TIMER
#define DRV_TIM2_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x0000U)
#define DRV_TIM3_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x0400U)
#define DRV_TIM4_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x0800U)
#define DRV_TIM5_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x0C00U)
#define DRV_TIM6_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x1000U)
#define DRV_TIM7_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x1400U)
#define DRV_TIM12_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x1800U)
#define DRV_TIM13_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x1C00U)
#define DRV_TIM14_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x2000U)
//SPI
#define DRV_SPI2_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x3800U)
#define DRV_SPI3_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x3C00U)
//I2C
#define DRV_I2C1_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x5400U)
#define DRV_I2C2_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x5800U)
//RTC, WWDG, IWDG, PWR, DAC, BKP
#define DRV_RTC_BASEADDR			(DRV_APB1PERIPH_BASE + 0x2800U)
#define DRV_WWDG_BASEADDR			(DRV_APB1PERIPH_BASE + 0x2C00U)
#define DRV_IWDG_BASEADDR			(DRV_APB1PERIPH_BASE + 0x3000U)
#define DRV_PWR_BASEADDR			(DRV_APB1PERIPH_BASE + 0x7000U)
#define DRV_DAC_BASEADDR			(DRV_APB1PERIPH_BASE + 0x7400U)
#define DRV_BKP_BASEADDR			(DRV_APB1PERIPH_BASE + 0x6C00U)
//USART
#define DRV_USART2_BASEADDR			(DRV_APB1PERIPH_BASE + 0x4400U)
#define DRV_USART3_BASEADDR			(DRV_APB1PERIPH_BASE + 0x4800U)
//UART
#define DRV_UART4_BASEADDR			(DRV_APB1PERIPH_BASE + 0x4C00U)
#define DRV_UART5_BASEADDR			(DRV_APB1PERIPH_BASE + 0x5000U)

/*
 * Base addresses of peripherals which are hanging on APB2
 * TODO : Complete for all other peripherals
 */
//GPIO
#define DRV_GPIOA_BASEADDR			(DRV_APB2PERIPH_BASE + 0x0800U)
#define DRV_GPIOB_BASEADDR			(DRV_APB2PERIPH_BASE + 0x0C00U)
#define DRV_GPIOC_BASEADDR			(DRV_APB2PERIPH_BASE + 0x1000U)
#define DRV_GPIOD_BASEADDR			(DRV_APB2PERIPH_BASE + 0x1400U)
#define DRV_GPIOE_BASEADDR			(DRV_APB2PERIPH_BASE + 0x1800U)
#define DRV_GPIOF_BASEADDR			(DRV_APB2PERIPH_BASE + 0x1C00U)
#define DRV_GPIOG_BASEADDR			(DRV_APB2PERIPH_BASE + 0x2000U)
//TIMER
#define DRV_TIM1_BASEADDR			(DRV_APB2PERIPH_BASE + 0x2C00U)
#define DRV_TIM8_BASEADDR			(DRV_APB2PERIPH_BASE + 0x3400U)
#define DRV_TIM9_BASEADDR			(DRV_APB2PERIPH_BASE + 0x4C00U)
#define DRV_TIM10_BASEADDR			(DRV_APB2PERIPH_BASE + 0x5000U)
#define DRV_TIM11_BASEADDR			(DRV_APB2PERIPH_BASE + 0x5400U)
//ADC
#define DRV_ADC1_BASEADDR			(DRV_APB2PERIPH_BASE + 0x2400U)
#define DRV_ADC2_BASEADDR			(DRV_APB2PERIPH_BASE + 0x2800U)
#define DRV_ADC3_BASEADDR			(DRV_APB2PERIPH_BASE + 0x3C00U)
//SPI
#define DRV_SPI1_BASEADDR 			(DRV_APB1PERIPH_BASE + 0x3000U)
//AFIO, EXTI
#define DRV_AFIO_BASEADDR			(DRV_APB2PERIPH_BASE + 0x0000U)
#define DRV_EXTI_BASEADDR			(DRV_APB2PERIPH_BASE + 0x0400U)
//USART
#define DRV_USART1_BASEADDR			(DRV_APB2PERIPH_BASE + 0x3800U)

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
} DRV_GPIO_RegDef_t ;

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
} DRV_RCC_RegDef_t;

/*
 * Peripheral definitions (peripheral base address typecasted to xxx_RegDef_t)
 */
//GPIO
#define DRV_GPIOA					((DRV_GPIO_RegDef_t*)DRV_GPIOA_BASEADDR)
#define DRV_GPIOB					((DRV_GPIO_RegDef_t*)DRV_GPIOB_BASEADDR)
#define DRV_GPIOC					((DRV_GPIO_RegDef_t*)DRV_GPIOC_BASEADDR)
#define DRV_GPIOD					((DRV_GPIO_RegDef_t*)DRV_GPIOD_BASEADDR)
#define DRV_GPIOE					((DRV_GPIO_RegDef_t*)DRV_GPIOE_BASEADDR)
#define DRV_GPIOF					((DRV_GPIO_RegDef_t*)DRV_GPIOF_BASEADDR)
#define DRV_GPIOG					((DRV_GPIO_RegDef_t*)DRV_GPIOG_BASEADDR)
//RCC
#define DRV_RCC						((DRV_RCC_RegDef_t*)DRV_RCC_BASEADDR)

/*
 * =====================================================================
 * NOTE : Enable macros peripheral
 * =====================================================================
 */

/*
 * Clock enable macros for GPIOx peripheral
 */
#define DRV_GPIOA_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 2))
#define DRV_GPIOB_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 3))
#define DRV_GPIOC_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 4))
#define DRV_GPIOD_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 5))
#define DRV_GPIOE_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 6))
#define DRV_GPIOF_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 7))
#define DRV_GPIOG_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 8))

/*
 * Clock enable macros for ADCx peripheral
 */
#define DRV_ADC1_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 9))
#define DRV_ADC2_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 10))
#define DRV_ADC3_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 15))

/*
 * Clock enable macros for TIMx peripheral
 */
#define DRV_TIM1_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 11))
#define DRV_TIM2_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 0))
#define DRV_TIM3_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 1))
#define DRV_TIM4_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 2))
#define DRV_TIM5_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 3))
#define DRV_TIM6_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 4))
#define DRV_TIM7_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 5))
#define DRV_TIM8_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 13))
#define DRV_TIM9_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 19))
#define DRV_TIM10_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 20))
#define DRV_TIM11_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 21))
#define DRV_TIM12_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 6))
#define DRV_TIM13_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 7))
#define DRV_TIM14_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 8))

/*
 * Clock enable macros for WWDG peripheral
 */
#define DRV_WWDG_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 11))

/*
 * Clock enable macros for I2Cx peripheral
 */
#define DRV_I2C1_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 21))
#define DRV_I2C2_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 22))

/*
 * Clock enable macros for USARTx peripheral
 */
#define DRV_USART1_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 14))
#define DRV_USART2_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 17))
#define DRV_USART3_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 18))
#define DRV_USART4_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 19))
#define DRV_USART5_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 20))

/*
 * Clock enable macros for SPIx peripheral
 */
#define DRV_SPI1_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 12))
#define DRV_SPI2_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 14))
#define DRV_SPI3_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 15))

/*
 * Clock enable macros for AFIO peripheral
 */
#define DRV_AFIO_PCLK_EN				(RCC->RCC_APB2ENR |= (1 << 0))

/*
 * Clock enable macros for DMAx peripheral
 */
#define DRV_DMA1_PCLK_EN				(RCC->RCC_AHBENR |= (1 << 0))
#define DRV_DMA2_PCLK_EN				(RCC->RCC_AHBENR |= (1 << 1))

/*
 * Clock enable macros for SRAM peripheral
 */
#define DRV_SRAM_PCLK_EN				(RCC->RCC_AHBENR |= (1 << 2))

/*
 * Clock enable macros for FLITF peripheral
 */
#define DRV_FLITF_PCLK_EN				(RCC->RCC_AHBENR |= (1 << 4))

/*
 * Clock enable macros for CRCE peripheral
 */
#define DRV_CRCE_PCLK_EN				(RCC->RCC_AHBENR |= (1 << 6))

/*
 * Clock enable macros for FSMC peripheral
 */
#define DRV_FSMC_PCLK_EN				(RCC->RCC_AHBENR |= (1 << 8))

/*
 * Clock enable macros for SDIO peripheral
 */
#define DRV_SDIO_PCLK_EN				(RCC->RCC_AHBENR |= (1 << 10))

/*
 * Clock enable macros for USB peripheral
 */
#define DRV_USB_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 23))

/*
 * Clock enable macros for CAN peripheral
 */
#define DRV_CAN_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 25))

/*
 * Clock enable macros for BKP peripheral
 */
#define DRV_BKP_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 27))

/*
 * Clock enable macros for PWR peripheral
 */
#define DRV_PWR_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 28))

/*
 * Clock enable macros for DAC peripheral
 */
#define DRV_DAC_PCLK_EN				(RCC->RCC_APB1ENR |= (1 << 29))



#endif /* INC_STM32F103XX_H_ */
