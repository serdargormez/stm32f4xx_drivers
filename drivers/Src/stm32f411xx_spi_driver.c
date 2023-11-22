/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: 22 Kas 2023
 *      Author: hsgor
 */

#include "stm32f411xx_spi_driver.h"


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

void SPI_Init(SPI_Handle_t *SPIHandle)
{

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


/* Data send and receive */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}


/* Irq config and ISR handling */

void SPI_IRQ_InterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}


void SPI_IRQ_PriortyConfig(uint8_t IRQNumber, uint32_t IRQPriorty)
{

}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

/*End of file*/
