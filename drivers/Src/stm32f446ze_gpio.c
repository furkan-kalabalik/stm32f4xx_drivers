/*
 * stm32f446ze_gpio.c
 *
 *  Created on: Feb 22, 2023
 *      Author: fk97
 */

#include "stm32f446ze_gpio.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_EN();
	} else {
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG)
			GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH)
			GPIOH_PCLK_DI();
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp;

	//1. configure mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	} else {
		//interrupt
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			//configure FTSR
			EXT->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear RTSR
			EXT->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//configure RTSR
			EXT->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear FTSR
			EXT->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			//configure both FTSR RTSR
			EXT->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXT->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//configure GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4] |= (GPIO_BASEADDR_TO_PORT(pGPIOHandle->pGPIOx) << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4)));
		//enable the EXIT interrupt delivery using IMR
		EXT->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	//2. configure speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	//3. pupd settings
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//4. optype
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure alt mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN) {
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8] |=  0xF << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8));
		pGPIOHandle->pGPIOx->AFR[pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8] |=  pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode
				<< (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8));
	}
}

void GPIO_Deinit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF)
		GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG)
		GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH)
		GPIOH_REG_RESET();
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= 0x00000001 << pinNumber;
	} else {
		pGPIOx->ODR &= ~(0x00000001 << pinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value){
	if(value == GPIO_PIN_SET){
		pGPIOx->ODR = 0xFFFFFFFF;
	} else {
		pGPIOx->ODR = 0x00000000;
	}
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
	pGPIOx->ODR ^= (1 << pinNumber);
}

void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if(EnorDi == ENABLE) {
		if(IRQNumber <= 31) {
			//ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber <= 64) {
			//ISER1
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		} else if(IRQNumber > 64 && IRQNumber <= 96) {
			//ISER2
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}
	} else {
		if(IRQNumber <= 31) {
			//ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if(IRQNumber > 31 && IRQNumber <= 64) {
			//ICER1
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		} else if(IRQNumber > 64 && IRQNumber <= 96) {
			//ICER2
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t priority){
	*(NVIC_IPR_BASE+((IRQNumber/4))) |= priority <<  (8 - NUMBER_OF_PRI_BITS_IMPLEMENTED + (8 * (IRQNumber % 4)));
}

void GPIO_IRQHandling(uint8_t pinNumber) {
	if(EXT->PR & (1 << pinNumber)){
		//clear
		EXT->PR |= (1 << pinNumber);
	}
}
