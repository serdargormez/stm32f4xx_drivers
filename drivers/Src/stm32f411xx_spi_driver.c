/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: 22 Kas 2023
 *      Author: hsgor
 */

#include "stm32f411xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*Peripheral clock setup */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}

		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}

		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}

		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}

		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_EN();
		}

		else
		{
			//do nothing
		}
	}

	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}

		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}

		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}

		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}

		else if(pSPIx == SPI5)
		{
			SPI5_PCLK_DI();
		}

		else
		{
			//do nothing
		}

	}
}


/*Init and De-init*/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enable the peripheral clock

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//Configure the CR1 register

	uint32_t tempreg = 0;

	//Configure Device Mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//Configure the Bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RX only bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	else
	{
		//Do nothing
	}

	//Configure the SPI serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	//Configure the DFF
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	//Configure the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	//Configure the CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	//Configure the SSM
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempreg;

}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}

	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}

	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}

	else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}

	else if(pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}

	else
	{
		//do nothing
	}
}


/*Other peripheral Control API's*/

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}

	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}

	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}

	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*Flag Status*/

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	else
	{
		//do nothing
	}

	return FLAG_RESET;

}


/* Data send and receive */

//This is blocking call
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//Load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}

		else
		{
			//8 bit DFF
			//Load the data in to the DR
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//Load the data from DR register to Rxbuffer
			*((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer--;
		}

		else
		{
			//8 bit DFF
			//Load the data from DR register to Rxbuffer
			*pRxBuffer = (uint8_t)pSPIx->DR;
			Len--;
			pRxBuffer--;
		}
	}
}


uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxbuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	else
	{
		//Do nothing
	}

	return state;
}


uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxbuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//Mark the SPI state as busy in transmission so that
		//no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}

	else
	{
		//Do nothing
	}

	return state;

}


/* Irq config and ISR handling */

void SPI_IRQ_InterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

		else
		{
			//Do nothing
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

		else
		{
			//Do nothing
		}
	}
}


void SPI_IRQ_PriortyConfig(uint8_t IRQNumber, uint32_t IRQPriorty)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriorty << (shift_amount));
}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;
	//First lets check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	else
	{
		//Do nothing
	}

	//Check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//Handle RXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	else
	{
		//Do nothing
	}

	//Check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//Handle OVR error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}

	else
	{
		//Do nothing
	}


}


//Some helper function implementations
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//Load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxbuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxbuffer++;
	}

	else
	{
		//8 bit DFF
		//Load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint8_t*)pSPIHandle->pTxbuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxbuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		//TxLen is zero, so close the SPI transmission and inform the application that
		//Tx is over

		//This prevents interrupts from setting up of TXE flag
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
		SPI_CloseTransmission(pSPIHandle);

	}

	else
	{
		//Do nothing
	}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//Load the data from DR register to Rxbuffer
		*((uint16_t*)pSPIHandle->pRxbuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxbuffer--;
	}

	else
	{
		//8 bit DFF
		//Load the data from DR register to Rxbuffer
		*((uint8_t*)pSPIHandle->pRxbuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxbuffer--;
	}

	if(!pSPIHandle->RxLen)
	{
		//Reception is complete
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		SPI_CloseReception(pSPIHandle);

	}

	else
	{
		//Do nothing
	}

}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	else
	{
		//Do nothing
	}

	(void)temp;
	//Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pRxbuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxbuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//This is a weak implementation. The user application may override this function
}

/*End of file*/
