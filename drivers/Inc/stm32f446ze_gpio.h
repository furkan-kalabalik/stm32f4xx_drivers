/*
 * stm32f446ze_gpio.h
 *
 *  Created on: Feb 22, 2023
 *      Author: fk97
 */

#ifndef INC_STM32F446ZE_GPIO_H_
#define INC_STM32F446ZE_GPIO_H_

#include "stm32f446ze.h"

typedef enum {
	GPIO_PIN_RESET,
	GPIO_PIN_SET
} GPIOPinStatus_t;

typedef enum {
	PIN_0,
	PIN_1,
	PIN_2,
	PIN_3,
	PIN_4,
	PIN_5,
	PIN_6,
	PIN_7,
	PIN_8,
	PIN_9,
	PIN_10,
	PIN_11,
	PIN_12,
	PIN_13,
	PIN_14,
	PIN_15
} GPIO_PinNumber_t;

typedef enum {
	GPIO_MODE_IN,
	GPIO_MODE_OUT,
	GPIO_MODE_ALT_FN,
	GPIO_MODE_ANALOG,
	GPIO_MODE_IT_FT,
	GPIO_MODE_IT_RT,
	GPIO_MODE_IT_RFT
} GPIO_PinMode_t;

typedef enum {
	GPIO_SPEED_LOW,
	GPIO_SPEED_MID,
	GPIO_SPEED_FAST,
	GPIO_SPEED_HIGH
} GPIO_PinSpeed_t;

typedef enum {
	GPIO_OP_TYPE_PP,
	GPIO_OP_TYPE_OD
} GPIO_OutputPinType_t;

typedef enum {
	GPIO_NO_PUPD,
	GPIO_PU,
	GPIO_PD
} GPIO_PuPdControl_t;

typedef struct {
	GPIO_PinNumber_t pinNumber;
	GPIO_PinMode_t pinMode;
	GPIO_PinSpeed_t pinSpeed;
	GPIO_PuPdControl_t pupdControl;
	GPIO_OutputPinType_t outputPinType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;

typedef struct {
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, EnableStatus_t enableStatus);

GPIOPinStatus_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOHandle, GPIO_PinNumber_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t *pGPIOHandle);

void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIOHandle, GPIO_PinNumber_t pinNumber, GPIOPinStatus_t value);
void GPIO_WriteToOutputPort(GPIO_Handle_t *pGPIOHandle, GPIOPinStatus_t value);

void GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIOHandle, GPIO_PinNumber_t pinNumber);

void GPIO_IRQITConfig(IRQNumber_t IRQNumber, EnableStatus_t enableStatus);
void GPIO_IRQPriorityConfig(IRQNumber_t IRQNumber, uint32_t priority);
void GPIO_IRQHandling(GPIO_PinNumber_t pinNumber);

#endif /* INC_STM32F446ZE_GPIO_H_ */
