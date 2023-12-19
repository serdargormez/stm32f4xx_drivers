/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Dec 18, 2023
 *      Author: Serdar
 */


#include "stm32f411xx_i2c_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};


/*Peripheral Control API's*/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}

	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/*Peripheral clock setup */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}

		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}

		else
		{
			//do nothing
		}
	}

	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}

		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}

		else
		{
			//do nothing
		}

	}
}


uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc = 0, temp = 0, ahbp = 0, apb1p = 0;

	clksrc |= ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}

	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}

	else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	/*For AHB*/
	temp |= ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}

	else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	/*For APB*/
	temp |= ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}

	else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}


/*Init and De-init*/

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	/*ACK control bit*/
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;


	/*Configure the FREQ field of CR2*/
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);


	/*Program the device own address */
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14); //14th bit must be 1
	pI2CHandle->pI2Cx->OAR1 = tempreg;


	/*CCR calculations*/
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/*Standart Mode*/
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= ccr_value & 0xFFF;

	}

	else
	{
		/*Fast Mode*/
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}

		tempreg |= ccr_value & 0xFFF;

	}

	pI2CHandle->pI2Cx->CCR = tempreg;

}


void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}
