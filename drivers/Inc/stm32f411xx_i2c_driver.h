/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: Dec 15, 2023
 *      Author: Serdar
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_


#include "stm32f411xx.h"


/*Configuration structure for I2Cx peripheral*/
typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_AckControl;
	uint16_t	I2C_FMDutyCycle;

}I2C_Config_t;


/*Handle structure for I2Cx peripheral*/
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 	  	*pTxBuffer;		/*To store the app. Tx buffer address*/
	uint8_t 	  	*pRxBuffer;		/*To store the app. Rx buffer address*/
	uint32_t 	  	TxLen;			/*To store Tx Len*/
	uint32_t 	  	RxLen;			/*To store Rx Len*/
	uint8_t 	  	TxRxState;		/*To store communication state*/
	uint8_t		  	DevAddr;		/*To store slave/device address */
	uint32_t	  	RxSize;			/*To store Rx size*/
	uint8_t		  	Rs;				/*To store repeated start value*/

}I2C_Handle_t;


/*@I2C_SCLSpeed*/

#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM2K		200000
#define I2C_SCL_SPEED_FM4K		400000


/*@I2C_AckControl*/

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0


/*@I2C_FMDutyCycle*/

#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


/*I2C related status flag definitions*/

#define I2C_FLAG_TXE 						(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE 						(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB							(1 << I2C_SR1_SB)
#define I2C_FLAG_OVR						(1 << I2C_SR1_OVR)
#define I2C_FLAG_AF							(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO						(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR						(1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF						(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10						(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF						(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 						(1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT					(1 << I2C_SR1_TIMEOUT)


/*Repeated Start*/

#define I2C_RS_ENABLE						SET
#define I2C_RS_DISABLE						RESET


/*I2C application state*/

#define I2C_READY							0
#define I2C_BUSY_IN_RX						1
#define I2C_BUSY_IN_TX						2


/*I2C application event macros*/

#define I2C_EV_TX_CMPLT						0
#define I2C_EV_RX_CMPLT						1
#define I2C_EV_STOP							2
#define I2C_ERROR_BERR						3
#define I2C_ERROR_ARLO						4
#define I2C_ERROR_AF						5
#define I2C_ERROR_OVR						6
#define I2C_ERROR_TIMEOUT					7


/*Peripheral clock setup */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


/*Init and De-init*/

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/* Data send and receive */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Rs);


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);


/*Other peripheral Control API's*/

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


/*Flag Status*/

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


/* Irq config and ISR handling */

void I2C_IRQ_InterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQ_PriortyConfig(uint8_t IRQNumber, uint32_t IRQPriorty);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*Application callback*/

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */
