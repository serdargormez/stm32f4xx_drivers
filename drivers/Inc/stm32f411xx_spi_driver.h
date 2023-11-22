/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: 22 Kas 2023
 *      Author: hsgor
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_


#include "stm32f411xx.h"
#include <stdint.h>

/*Configuration structure for SPIx peripheral*/
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;


/*Peripheral clock setup */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


/*Init and De-init*/

void SPI_Init(SPI_Handle_t *SPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/* Data send and receive */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);


/* Irq config and ISR handling */

void SPI_IRQ_InterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQ_PriortyConfig(uint8_t IRQNumber, uint32_t IRQPriorty);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
