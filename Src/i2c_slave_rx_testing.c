#include <string.h>
#include <stdio.h>

#include "stm32f411xx.h"


#define MY_ADDR			0x37

void delay(void)
{
	for(uint32_t i = 0; i <= 250000 ; i++);
}


I2C_Handle_t I2C1Handle;


/*Some data*/
uint8_t sendData = 0xAA;
uint8_t ReceiveData = 0;
uint8_t temp = 0;

/*
 * PB6 -> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}


void I2C1_Init()
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
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
	/*Button and led init*/
	GPIO_Button_Init();


	/*I2C pin init*/
	I2C1_GPIOInits();


	/*I2C peripheral configuration*/
	I2C1_Init();

	I2C_IRQ_InterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQ_InterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);


	while(1)
	{
		delay();

		temp = ReceiveData;

		if(temp == 170)
		{
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
			temp = 0;
		}

		else
		{
			//nothing
		}
	}
}



void I2C1_EV_IRQHandler()
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler()
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	if(AppEvent == I2C_EV_DATA_REQ)
	{
		I2C_SlaveSendData(pI2CHandle->pI2Cx, sendData);
	}

	else if(AppEvent == I2C_EV_DATA_RCV)
	{
		ReceiveData = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}

	else if(AppEvent == I2C_ERROR_AF)
	{

	}

	else if(AppEvent == I2C_EV_STOP)
	{

	}

	else
	{
		/*Do nothing*/
	}
}
