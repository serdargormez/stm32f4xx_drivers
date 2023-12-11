
#include "stm32f411xx.h"
#include <string.h>

/*
PB12 -> SPI2_NSS
PB13 -> SPI2_SCLK
PB14 -> SPI2_MISO
PB15 -> SPI2_MOSI
Alternate function mode : 5
*/

SPI_Handle_t SPI2handle;

void delay(void)
{
	for(uint32_t i = 0; i <= 200000 ; i++);
}

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);
}

void SPI2_Init()
{


	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV128;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; //Software slave management enabled for NSS Pin

	SPI_Init(&SPI2handle);

}

uint8_t Rx_data[1] = {0};
uint8_t rec_data[1] = {1};

int main(void)
{
	uint8_t Tx_data[8] = {1,2,3,4,5,6,7,8};


	SPI2_GPIOInits();

	SPI2_Init();

	//This makes NSS internally high and avoids MODEF error
	SPI_SSIConfig(SPI2, ENABLE);

	SPI_IRQ_InterruptConfig(IRQ_NO_SPI2, ENABLE);
	SPI_IRQ_PriortyConfig(IRQ_NO_SPI2, 15);

	//LED config
	GPIO_Handle_t GpioLed;
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

	while(1)
	{
		SPI_PeripheralControl(SPI2, ENABLE);

		while(SPI_SendData_IT(&SPI2handle, Tx_data, 8) == SPI_BUSY_IN_RX);

		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		while(SPI_ReceiveData_IT(&SPI2handle, Rx_data, 1) == SPI_BUSY_IN_RX);

		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GPIO_PIN_NO_13);

		delay();

	}

	return 0;
}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2handle);
}


void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

	delay();

	if(0 == memcmp(Rx_data, rec_data, 1))
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_14);
	}
}
