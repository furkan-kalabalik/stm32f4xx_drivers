/*
 * stm32f446ze.h
 *
 *  Created on: Feb 20, 2023
 *      Author: fk97
 */

#ifndef INC_STM32F446ZE_H_
#define INC_STM32F446ZE_H_

#include<stdint.h>

#define __vo volatile

// Base address of flash and SRAM memories
#define FLASH_BASEADDR 		0x08000000U
#define SRAM1_BASEADDR 		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define ROM_BASEADDR		0x1FFF0000U
#define SRAM 				( SRAM1_BASEADDR + SRAM2_BASEADDR )

// Base address of peripherals
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

// Base address of peripherals hanging on AHB1
#define GPIOA_BASEADDR		( AHB1PERIPH_BASE + 0x0000 )
#define GPIOB_BASEADDR		( AHB1PERIPH_BASE + 0x0400 )
#define GPIOC_BASEADDR		( AHB1PERIPH_BASE + 0x0800 )
#define GPIOD_BASEADDR		( AHB1PERIPH_BASE + 0x0C00 )
#define GPIOE_BASEADDR		( AHB1PERIPH_BASE + 0x1000 )
#define GPIOF_BASEADDR		( AHB1PERIPH_BASE + 0x1400 )
#define GPIOG_BASEADDR		( AHB1PERIPH_BASE + 0x1800 )
#define GPIOH_BASEADDR		( AHB1PERIPH_BASE + 0x1C00 )
#define RCC_BASEADDR		( AHB1PERIPH_BASE + 0x3800 )

// Base address of peripherals hanging on APB1
#define I2C_1_BASEADDR		( APB1PERIPH_BASE + 0x5400 )
#define I2C_2_BASEADDR		( APB1PERIPH_BASE + 0x5800 )
#define I2C_3_BASEADDR		( APB1PERIPH_BASE + 0x5C00 )

#define SPI_2_BASEADDR		( APB1PERIPH_BASE + 0x3800 )
#define SPI_3_BASEADDR		( APB1PERIPH_BASE + 0x3C00 )

#define USART_2_BASEADDR	( APB1PERIPH_BASE + 0x4400 )
#define USART_3_BASEADDR	( APB1PERIPH_BASE + 0x4800 )
#define UART_4_BASEADDR		( APB1PERIPH_BASE + 0x4C00 )
#define UART_5_BASEADDR		( APB1PERIPH_BASE + 0x5000 )

// Base address of peripherals hanging on APB2
#define SPI_1_BASEADDR		( APB2PERIPH_BASE + 0x3000 )
#define USART_1_BASEADDR	( APB2PERIPH_BASE + 0x1000 )
#define USART_6_BASEADDR	( APB2PERIPH_BASE + 0x1400 )
#define EXTI_BASEADDR		( APB2PERIPH_BASE + 0x3C00 )
#define SYSCFG_BASEADDR		( APB2PERIPH_BASE + 0x3800 )


/*********************PERIPH REG DEF STRUCTURES*********************/

typedef struct {
	__vo uint32_t MODER; 		//GPIO port mode register
	__vo uint32_t OTYPER; 		//GPIO port output type register
	__vo uint32_t OSPEEDER; 	//GPIO port output speed register
	__vo uint32_t PUPDR; 		//GPIO port pull-up/pull-down register
	__vo uint32_t IDR; 			//GPIO port input data register
	__vo uint32_t ODR; 			//GPIO port output data register
	__vo uint32_t BSRR; 		//GPIO port bit set/reset register
	__vo uint32_t LCKR; 		//GPIO port configuration lock register
	__vo uint32_t AFR[2]; 		//GPIO alternate function low register and GPIO alternate function high register
}GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t reserved1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t reserved2;
	__vo uint32_t reserved3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t reserved4;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t reserved5;
	__vo uint32_t reserved6;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t reserved7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t reserved8;
	__vo uint32_t reserved9;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t reserved10;
	__vo uint32_t reserved11;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/*********************PERIPH DEFINITIONS*********************/
#define GPIOA ( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB ( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC ( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD ( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE ( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF ( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG ( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH ( (GPIO_RegDef_t*) GPIOH_BASEADDR )

#define RCC	  ( (RCC_RegDef_t* ) RCC_BASEADDR 	)

/***************************CLOCK ENABLE MACROS***************************/
//GPIO
#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) )

//I2C
#define I2C_1_PCLK_EN()		( RCC->APB1ENR |= (1 << 21) )
#define I2C_2_PCLK_EN()		( RCC->APB1ENR |= (1 << 22) )
#define I2C_3_PCLK_EN()		( RCC->APB1ENR |= (1 << 23) )

//SPI
#define SPI_1_PCLK_EN()		( RCC->APB2ENR |= (1 << 12) )
#define SPI_2_PCLK_EN()		( RCC->APB1ENR |= (1 << 14) )
#define SPI_3_PCLK_EN()		( RCC->APB1ENR |= (1 << 15) )

//USART
#define USART_1_PCLK_EN()	( RCC->APB2ENR |= (1 << 4)  )
#define USART_2_PCLK_EN()	( RCC->APB1ENR |= (1 << 17) )
#define USART_3_PCLK_EN()	( RCC->APB1ENR |= (1 << 18) )
#define UART_4_PCLK_EN()	( RCC->APB1ENR |= (1 << 19) )
#define UART_5_PCLK_EN()	( RCC->APB1ENR |= (1 << 20) )
#define USART_6_PCLK_EN()	( RCC->APB2ENR |= (1 << 5)  )

//SYSCFG
#define SYSCFG_PCLK_EN()	( RCC->APB2ENR |= (1 << 14) )


/***************************CLOCK DISABLE MACROS***************************/
//GPIO
#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7) )

//I2C
#define I2C_1_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 21) )
#define I2C_2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 22) )
#define I2C_3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 23) )

//SPI
#define SPI_1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 12) )
#define SPI_2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 14) )
#define SPI_3_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 15) )

//USART
#define USART_1_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 4)  )
#define USART_2_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 17) )
#define USART_3_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 18) )
#define UART_4_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 19) )
#define UART_5_PCLK_DI()	( RCC->APB1ENR &= ~(1 << 20) )
#define USART_6_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 5)  )

//SYSCFG
#define SYSCFG_PCLK_DI()	( RCC->APB2ENR &= ~(1 << 14) )

/***************************PERIPHERAL RESET MACROS***************************/
//GPIO
#define GPIOA_REG_RESET()	do{ \
	( RCC->AHB1RSTR |= (1 << 0) ); \
	( RCC->AHB1RSTR &= ~(1 << 0) ); \
	}while(0)

#define GPIOB_REG_RESET()	do{ \
		( RCC->AHB1RSTR |= (1 << 1) ) ; \
		( RCC->AHB1RSTR &= ~(1 << 1) ) ; \
		}while(0)

#define GPIOC_REG_RESET()	do{ \
		( RCC->AHB1RSTR |= (1 << 2) ); \
		( RCC->AHB1RSTR &= ~(1 << 2) ); \
}while(0)

#define GPIOD_REG_RESET()	do{ \
		( RCC->AHB1RSTR |= (1 << 3) ); \
		( RCC->AHB1RSTR &= ~(1 << 3) ); \
}while(0)

#define GPIOE_REG_RESET()	do{ \
		( RCC->AHB1RSTR |= (1 << 4) ); \
		( RCC->AHB1RSTR &= ~(1 << 4) ); \
}while(0)

#define GPIOF_REG_RESET()	do{ \
		( RCC->AHB1RSTR |= (1 << 5) ); \
		( RCC->AHB1RSTR &= ~(1 << 5) ); \
}while(0)

#define GPIOG_REG_RESET()	do{ \
		( RCC->AHB1RSTR |= (1 << 6) ); \
		( RCC->AHB1RSTR &= ~(1 << 6) ); \
}while(0)

#define GPIOH_REG_RESET()	do{ \
		( RCC->AHB1RSTR |= (1 << 7) ); \
		( RCC->AHB1RSTR &= ~(1 << 7) ); \
}while(0)


#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET


#include "stm32f446ze_gpio.h"
#endif /* INC_STM32F446ZE_H_ */
