/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Dec 18, 2023
 *      Author: Serdar
 */


#include "stm32f411xx_i2c_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};

/*Helper functions*/

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}


static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);
	SlaveAddr &= ~(1 << 0); /*Slave address + r/nw bit = 0*/
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);
	SlaveAddr |= (1 << 0); /*Slave address + r/nw bit = 1*/
	pI2Cx->DR = SlaveAddr;
}


static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}


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

	else
	{
		/*Do nothing*/
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

	/*Enable the clock for the i2cx peripheral*/
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);


	/*Peripheral Enable*/
	I2C_PeripheralControl(I2C1, ENABLE);


	/*ACK control bit*/
	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);


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


	/*TRISE configuration*/
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/*Standart Mode*/
		/* Trise / Tpclk -> Trise x Fpclk*/
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}

	else
	{
		/*Fast Mode*/
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}

	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}

	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}

	else
	{
		//do nothing
	}
}


/*Data send and receive*/

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs)
{
	/*Generate start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	/*Confirm that start generation is completed by checking the SB flag in the SR1*/
	/*Note : Until SB is cleared SCL will be streched (pulled to LOW)*/
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));


	/*Send the address of the slave with r/nw bit set to w(0) (total bit 8)*/
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);


	/*Confirm that address phase is completed by checking the ADDR flag in the SR1*/
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	/*Clear the ADDR flag according to its software sequence*/
	/*Note : Until ADDR flag is cleared SCL will be streched (pulled to LOW)*/
	I2C_ClearAddrFlag(pI2CHandle->pI2Cx);


	/*Send data until Len becomes 0*/
	while(Len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}


	/*When Len becomes zero wait for TxE=1 and BTF=1 before generating the stop condition*/
	/*Note : TxE=1 and BTF=1, means that both SR and DR are empty*/
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));


	/*Generate the STOP condition*/
	/*Generating STOP , automatically clears by BTF*/
	if(Rs == I2C_RS_DISABLE)
	{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}
}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs)
{
	/*Generate start condition*/
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


	/*Confirm that start generation is completed by checking the SB flag in the SR1*/
	/*Note : Until SB is cleared SCL will be streched (pulled to LOW)*/
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));


	/*Send the address of the slave with r/nw bit set to r(1) (total bit 8)*/
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);


	/*Confirm that address phase is completed by checking the ADDR flag in the SR1*/
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	/*Procedure to read only 1 byte from slave*/
	if(Len == 1)
	{
		/*Disable acking*/
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


		/*Clear the ADDR flag*/
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);


		/*Wait until RXNE becomes 1*/
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


		/*Generate STOP condition*/
		if(Rs == I2C_RS_DISABLE)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}


		/*Read the data from data register in to buffer*/
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}


	/*Procedure to read more than 1 byte from slave*/
	else if(Len > 1)
	{
		/*Clear the ADDR flag*/
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);


		/*Read the data until Len becomes zero*/
		for(uint32_t i = Len; i > 0; i--)
		{
			/*Wait until RXNE becomes 1*/
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


			/*If last 2 byte are remaining */
			if(i == 2)
			{
				/*Clear the ACK bit*/
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


				/*Generate STOP condition*/
				if(Rs == I2C_RS_DISABLE)
				{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}
			}

			else
			{
				/*Do nothing*/
			}


			/*Read the data from data register in to buffer*/
			*pRxBuffer = pI2CHandle->pI2Cx->DR;


			/*Increment the buffer address*/
			pRxBuffer++;
		}
	}

	else
	{
		/*Do nothing*/
	}


	/*Re-enable ACKing*/
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

	else
	{
		/*Do nothing*/
	}
}


void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		/*Enable the ACK*/
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}

	else
	{
		/*Disable the ACK*/
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}

	else
	{
		//do nothing
	}

	return FLAG_RESET;
}
