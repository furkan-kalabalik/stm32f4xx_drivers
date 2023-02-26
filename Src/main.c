/*
 * main.c
 *
 *  Created on: Feb 22, 2023
 *      Author: fk97
 */

#include "stm32f446ze.h"

void delay(void) {
	for (int i = 0; i < 500000; ++i);
}

int main() {
	GPIO_Handle_t gpioLed7, gpioUserButton;
	gpioLed7.pGPIOx = GPIOB;
	gpioLed7.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	gpioLed7.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed7.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed7.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed7.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	gpioUserButton.pGPIOx = GPIOC;
	gpioUserButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpioUserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioUserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioUserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioLed7);
	GPIO_Init(&gpioUserButton);
	uint8_t status = 0;
	while(1) {
		if(status) {
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_7, SET);
		} else {
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_7, RESET);
		}

		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13)) {
			delay();
			status = ~(status);
		}
	}
	return 0;
}
