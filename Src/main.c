/*
 * main.c
 *
 *  Created on: Feb 22, 2023
 *      Author: fk97
 */

#include "stm32f446ze.h"
GPIO_Handle_t gpioLed7, gpioUserButton;

void delay(void) {
	for (int i = 0; i < 500000; ++i);
}

int main() {

	gpioLed7.pGPIOx = GPIOB;
	gpioLed7.GPIO_PinConfig.pinNumber = PIN_7;
	gpioLed7.GPIO_PinConfig.pinMode = GPIO_MODE_OUT;
	gpioLed7.GPIO_PinConfig.pinSpeed = GPIO_SPEED_FAST;
	gpioLed7.GPIO_PinConfig.outputPinType = GPIO_OP_TYPE_PP;
	gpioLed7.GPIO_PinConfig.pupdControl = GPIO_NO_PUPD;

	gpioUserButton.pGPIOx = GPIOC;
	gpioUserButton.GPIO_PinConfig.pinNumber = PIN_13;
	gpioUserButton.GPIO_PinConfig.pinMode = GPIO_MODE_IT_FT;
	gpioUserButton.GPIO_PinConfig.pinSpeed = GPIO_SPEED_FAST;
	gpioUserButton.GPIO_PinConfig.pupdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpioLed7);
	GPIO_Init(&gpioUserButton);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 15);
	GPIO_IRQITConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);
	return 0;
}

void EXTI15_10_IRQHandler(void) {
	delay();
	GPIO_IRQHandling(PIN_13);
	GPIO_ToggleOutputPin(&gpioLed7, PIN_7);
}

