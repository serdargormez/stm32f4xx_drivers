#ifndef DRIVERS_INC_STM32F411XX_RCC_DRIVER_H_
#define DRIVERS_INC_STM32F411XX_RCC_DRIVER_H_

#include "stm32f411xx.h"

uint32_t RCC_GetPLLOutputClock(void);

/*This returns the APB1 clock value*/
uint32_t RCC_GetPCLK1Value(void);

/*This returns the APB2 clock value*/
uint32_t RCC_GetPCLK2Value(void);


#endif /* DRIVERS_INC_STM32F411XX_RCC_DRIVER_H_ */
