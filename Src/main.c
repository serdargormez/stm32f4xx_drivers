/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "stm32f411xx.h"
#include "stm32f411xx_gpio_driver.h"
#include <stdint.h>
#include <string.h>

void delay(void)
{
	for(uint32_t i = 0; i <= 500000 ; i++);
}

GPIO_Handle_t GpioLed, GpioButton;

int main(void)
{

	memset(&GpioButton,0,sizeof(GpioButton));
	memset(&GpioLed,0,sizeof(GpioLed));

    GpioButton.pGPIOx = GPIOA;
    GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GpioButton.pGPIOx, ENABLE);
    GPIO_Init(&GpioButton);

    GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);
	GPIO_Init(&GpioLed);

	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GpioLed);
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GpioLed);
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GpioLed);

	GPIO_IRQ_PriortyConfig(IRQ_NO_EXTI0, 15);
	GPIO_IRQ_InterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1)
	{
		/*if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_12);
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13);
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_14);
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_15);
			delay();
		}*/
	}
}

void EXTI0_IRQHandler(void)
{

	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_14);
	GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_15);

}










