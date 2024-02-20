#include <stdio.h>
#include <string.h>

#include "stm32f411xx.h"

//char msg[1024] = "uart testing..\n\r";
uint8_t sendData = 0xAA;

USART_Handle_t usart2_handle;


void delay(void)
{
	for(uint32_t i = 0; i <= 250000 ; i++);
}


void USART2_Init(void)
{
	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart2_handle);
}

void USART_GPIOInit(void)
{
	GPIO_Handle_t usart_gpio;

	usart_gpio.pGPIOx = GPIOA;
	usart_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	usart_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	usart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	//USART2 TX
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart_gpio);

	//USART2 RX
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart_gpio);

}


void GPIO_Button_Init(void)
{
	GPIO_Handle_t GPIOLed, GPIOButton;

	memset(&GPIOButton,0,sizeof(GPIOButton));
	memset(&GPIOLed,0,sizeof(GPIOLed));


	/*Button configuration*/
	GPIOButton.pGPIOx = GPIOA;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOButton);


	/*Led configuration*/
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOLed);

	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GPIOLed);

	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GPIOLed);
}

int main(void)
{
	GPIO_Button_Init();

	USART_GPIOInit();

	USART2_Init();

	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);

	USART_PeripheralControl(USART2, ENABLE);


	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		while(USART_SendDataIT(&usart2_handle, &sendData, 1) != USART_READY);

	}
}


void USART2_IRQHandler()
{
	USART_IRQHandling(&usart2_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{
	if(AppEv == USART_EVENT_TX_CMPLT)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_13);
	}

	else if(AppEv == USART_EVENT_RX_CMPLT)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
	}
}

