#include "stm32f411xx_usart_driver.h"

/*Peripheral Clock setup*/

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}

		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}

		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}

		else
		{
			//do nothing
		}
	}

	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}

		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}

		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}

		else
		{
			//do nothing
		}

	}
}


/*Init and De-init*/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg = 0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}

	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	}

	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | ( 1 << USART_CR1_TE));
	}

	else
	{
		//Do nothing
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);


    //Configuration of parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}

	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//Implement the code to enable the parity control
	    tempreg |= (1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= (1 << USART_CR1_PS);

	}

	else
	{
		//Do nothing
	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg = 0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg = 0;

	//Configuration of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	}

	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE );
	}

	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	else
	{
		//Do nothing
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate


}


void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}


/*Data Send and Receive*/

void USART_SendData(USART_RegDef_t *pUSARTx,uint8_t *pTxBuffer, uint32_t Len)
{

}


void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/*
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{

}


uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{

}
*/

/*IRQ Configuration and ISR handling*/

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}


void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}


void USART_IRQHandling(USART_Handle_t *pHandle)
{

}


/*Other Peripheral Control APIs*/

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{

}


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}

	else
	{
		//do nothing
	}

	return FLAG_RESET;
}


void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{

}


/*Application callback*/

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{

}
