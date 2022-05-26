#ifndef __PORT_H__
#define __PORT_H__

#include "deca_device_api.h"

#ifdef STM32F103xB
#include <stm32f1xx_hal.h>
#else
#include <stm32f0xx_hal.h>
#endif

decaIrqStatus_t decamutexon(void);

void decamutexoff(decaIrqStatus_t s);

#define SPI_BaudRatePrescaler_4 0
#define SPI_BaudRatePrescaler_16 1

//void SPI_ConfigFastRate(uint16_t scalingfactor);
#define SPI_ConfigFastRate(X)

#define portGetTickCount() 			HAL_GetTick()

void reset_DW1000(void);

void setup_DW1000RSTnIRQ(int enable);

#define port_SPIx_clear_chip_select()
#define port_SPIx_set_chip_select()


#endif
