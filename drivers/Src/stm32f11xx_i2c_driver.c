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
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);
static void I2CMasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2CMasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}


void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
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


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Rs = Rs;


		/*Implemet code to generate START condition*/
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		/*Implement the code to enable ITBUFEN control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		/*Implement the code to enable ITEVTEN control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		/*Implement the code to enable ITERREN control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX) )
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->RxSize = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Rs = Rs;

		/*Implemet code to generate START condition*/
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		/*Implement the code to enable ITBUFEN control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		/*Implement the code to enable ITEVTEN control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		/*Implement the code to enable ITERREN control bit*/
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}


static void I2CMasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		/*Load the data in to DR*/
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		/*Decrement the TxLen*/
		pI2CHandle->TxLen--;

		/*Increment the buffer address*/
		pI2CHandle->pTxBuffer++;
	}

	else
	{
		/*Do nothing*/
	}
}


static void I2CMasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	/*We have to do the data reception*/
	if(pI2CHandle->RxSize == 1)
	{
		/*Read DR*/
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxLen--;
	}

	else if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			/*clear the ACK bit*/
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		else
		{
			/*Do nothing*/
		}

		/*Read DR*/
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	else
	{
		/*Do nothing*/
	}


	if(pI2CHandle->RxLen == 0)
	{
		/*Close the I2C data reception and notify the application*/

		/*Generate the STOP condition*/
		if(pI2CHandle->Rs == I2C_RS_DISABLE)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		else
		{
			/*Do nothing*/
		}

		/*Close the I2C Rx*/
		I2C_CloseReceiveData(pI2CHandle);

		/*Notify the application*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}

	else
	{
		/*Do nothing*/
	}
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}


uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
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


void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	else
	{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
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


void I2C_IRQ_InterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}

		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}

		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}

	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}

		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}

		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}


void I2C_IRQ_PriortyConfig(uint8_t IRQNumber, uint32_t IRQPriorty)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriorty << (shift_amount));
}


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	/*Implement the code to disable ITBUFEN Control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	/*Implement the code to disable ITEVTEN Control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
	pI2CHandle->TxRxState = I2C_READY;
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	/*Implement the code to disable ITBUFEN Control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	/*Implement the code to disable ITEVTEN Control bit*/
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->RxSize = 0;
	pI2CHandle->RxLen = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->TxRxState = I2C_READY;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	/*Interrupt handling for both master and slave mode of a device*/

	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	/*Handle for interrupt generated by SB event*/
	/*SB flag is only applicable in master mode*/
	if(temp1 && temp3)
	{
		/*Executed address phase*/
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

		else
		{
			/*Do nothing*/
		}
	}

	else
	{
		/*Do nothing*/
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);

	/*Handle for interrupt generated by ADDR event*/
	if(temp1 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
			}

			else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize == 1)
				{
					I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
				}
			}

			else
			{
				/*Do nothing*/
			}
		}

		else
		{
			/*Do nothing*/
		}
	}

	else
	{
		/*Do nothing*/
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);

	/*Handle for interrupt generated by BTF(Byte transfer finished) event*/
	if(temp1 && temp3)
	{
		/*BTF flag is set*/
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			/*Make sure TXE also set*/
			if(pI2CHandle->pI2Cx->SR1 & (1<< I2C_SR1_TXE))
			{
				/*BTF, TXE = 1*/
				if(pI2CHandle->TxLen == 0)
				{
					/*Generate STOP condition*/
					if(pI2CHandle->Rs == I2C_RS_DISABLE)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					else
					{
						/*Do nothing*/
					}

					/*Reset all the member element of the handle structure*/
					I2C_CloseSendData(pI2CHandle);

					/*Notify the application about transmission complete*/
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

				else
				{
					/*Do nothing*/
				}
			}

			else
			{
				/*Do nothing*/
			}
		}

		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			/*Do nothing*/
		}

		else
		{
			/*Do nothing*/
		}
	}

	else
	{
		/*Do nothing*/
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);

	/*Handle for interrupt generated by STOPF event*/
	/*Stop detection flag is applicable only slave mode.*/
	if(temp1 && temp3)
	{
		/*STOPF flag is set*/
		/*Clear the STOPF*/
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		/*Notify the application that stop is detected*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	else
	{
		/*Do nothing*/
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);

	/*Handle for interrupt generated by TXE event*/
	if(temp1 && temp2 && temp3)
	{
		/*TXE flag is set*/
		/*Check for the device mod*/
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			/*Device is master*/

			/*We have to do data transmission*/
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2CMasterHandleTXEInterrupt(pI2CHandle);
			}

			else
			{
				/*Do nothing*/
			}
		}

		else
		{
			/*Slave*/
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}

			else
			{
				/*Do nothing*/
			}
		}
	}

	else
	{
		/*Do nothing*/
	}


	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);

	/*Handle for interrupt generated by RXNE event*/
	if(temp1 && temp2 && temp3)
	{
		/*RXNE flag is set*/
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			/*Device is master*/

			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2CMasterHandleRXNEInterrupt(pI2CHandle);
			}

			else
			{
				/*Do nothing*/
			}
		}

		else
		{
			/*Slave*/
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}

			else
			{
				/*Do nothing*/
			}
		}
	}

	else
	{
		/*Do nothing*/
	}
}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

	/*Know the status of ITERREN control bit in the CR2*/
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);


/***********************************Check for Bus error**********************************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);

	if(temp1 && temp2)
	{
		/*This is Bus error*/
		/*Implement the code to clear the buss error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		/*Notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	else
	{
		/*Do nothing*/
	}


/**********************************Check for Arbitration lost error**********************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO);

	if(temp1 && temp2)
	{
		/*This is Arbitration lost error*/
		/*Implement the code to clear the Arbitration lost error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		/*Notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	else
	{
		/*Do nothing*/
	}


/**********************************Check for ACK Failure error**************************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);

	if(temp1 && temp2)
	{
		/*This is ACK Failure error*/
		/*Implement the code to clear the ACK Failure error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		/*Notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	else
	{
		/*Do nothing*/
	}


/**********************************Check for Overrun/Underrun error**************************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);

	if(temp1 && temp2)
	{
		/*This is Overrun/Underrun error*/
		/*Implement the code to clear the Overrun/Underrun error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		/*Notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	else
	{
		/*Do nothing*/
	}


/**********************************Check for Time out error**************************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);

	if(temp1 && temp2)
	{
		/*This is Time out error*/
		/*Implement the code to clear the Time out error flag*/
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		/*Notify the application about the error*/
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

	else
	{
		/*Do nothing*/
	}
}









