/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* uwb.c: Uwb radio implementation, low level handling */

#include <stm32f0xx_hal.h>

#include "uwb.h"

#include "dwOps.h"
#include "deca_device_api.h"
#include "examples_defines.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

// used in the example code
void test_run_info(unsigned char *data)
{
    printf("%s\n", data);
}

void Sleep(unsigned int time_ms)
{
    deca_sleep(time_ms);
}
/* Example application name and version to display on LCD screen/VCOM port. */
#define APP_NAME "READ DEV ID      "

/**
 * Application entry point.
 */
int my_read_dev_id(void)
{
    int err;
    /* Display application name on LCD. */
    test_run_info((unsigned char *)APP_NAME);

    /* Configure SPI rate, DW3000 supports up to 38 MHz */
    port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    /* Reads and validate device ID returns DWT_ERROR if it does not match expected else DWT_SUCCESS */
    if ((err=dwt_check_dev_id())==DWT_SUCCESS)
    {
        test_run_info((unsigned char *)"DEV ID OK");
    }
    else
    {
    	test_run_info((unsigned char *)"DEV ID FAILED");
    }

    return err;
}


void uwbInit()
{
  // Initializing the low level radio handling
  // static StaticSemaphore_t irqSemaphoreBuffer;
  // irqSemaphore = xSemaphoreCreateBinaryStatic(&irqSemaphoreBuffer);

  dwt_setleds(3);
  //uwbErrorCode = dwConfigure(dwm); // Configure the dw1000 chip
	uint32_t id = dwt_readdevid();
	printf("==============ID:%08x\n", id);

  my_read_dev_id();

	printf("example_pointer:%08x\n", example_pointer);
  build_examples();
	printf("example_pointer:%08x\n", example_pointer);
  example_pointer();

  // if (uwbErrorCode == 0) {
  //   dwEnableAllLeds(dwm);
  // } else {
  //   return;
  // }
  // dwTime_t delay = {.full = 0};
  // dwSetAntenaDelay(dwm, delay);

  // // Reading and setting node configuration
  // cfgReadU8(cfgAddress, &config.address[0]);
  // cfgReadU8(cfgMode, &config.mode);
  // cfgFieldSize(cfgAnchorlist, &config.anchorListSize);
  // if (config.anchorListSize <= MAX_ANCHORS) {
  //   cfgReadU8list(cfgAnchorlist, config.anchors, config.anchorListSize);
  // }

  // if (config.mode < uwbAlgorithmCount()) {
  //   algorithm = availableAlgorithms[config.mode].algorithm;
  // } else {
  //   algorithm = &dummyAlgorithm;
  // }

  // config.positionEnabled = cfgReadFP32list(cfgAnchorPos, config.position, 3);

  // dwAttachSentHandler(dwm, txcallback);
  // dwAttachReceivedHandler(dwm, rxcallback);
  // dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);
  // dwAttachReceiveFailedHandler(dwm, rxfailedcallback);

  // dwNewConfiguration(dwm);
  // dwSetDefaults(dwm);

  // uint8_t useLowBitrate = 0;
  // cfgReadU8(cfgLowBitrate, &useLowBitrate);
  // #ifdef LPS_LONGER_RANGE
  // useLowBitrate = 1;
  // #endif
  // config.lowBitrate = (useLowBitrate == 1);

  // uint8_t useLongPreamble = 0;
  // cfgReadU8(cfgLongPreamble, &useLongPreamble);
  // config.longPreamble = (useLongPreamble == 1);

  // const uint8_t* mode = MODE_SHORTDATA_FAST_ACCURACY;
  // if (useLowBitrate && !useLongPreamble) {
  //   mode = MODE_SHORTDATA_MID_ACCURACY;
  // } else if (!useLowBitrate && useLongPreamble) {
  //   mode = MODE_LONGDATA_FAST_ACCURACY;
  // } else if (useLowBitrate && useLongPreamble) {
  //   mode = MODE_LONGDATA_MID_ACCURACY;
  // }
  // dwEnableMode(dwm, mode);

  // dwSetChannel(dwm, CHANNEL_2);

  // // Enable smart power by default
  // uint8_t enableSmartPower = 1;
  // cfgReadU8(cfgSmartPower, &enableSmartPower);
  // config.smartPower = enableSmartPower != 0;
  // if (enableSmartPower) {
  //   dwUseSmartPower(dwm, true);
  // }

  // // Do not force power by default
  // uint8_t forceTxPower = 0;
  // cfgReadU8(cfgForceTxPower, &forceTxPower);
  // config.forceTxPower = forceTxPower != 0;
  // if (forceTxPower) {
  //   uint32_t txPower = 0x1F1F1F1Ful;
  //   cfgReadU32(cfgTxPower, &txPower);
  //   config.txPower = txPower;
  //   dwSetTxPower(dwm, txPower);
  // }

  // dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

  // dwCommitConfiguration(dwm);

  // isInit = true;
}

static void uwbTask(void* parameters)
{

  while(1) {
    vTaskDelay(1000);
  }
}

/**** DWM3000 interrupt handling *****/
#define DWM_IRQn EXTI0_1_IRQn
#define DWM_IRQ_PIN GPIO_PIN_0

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // BaseType_t higherPriorityTaskWoken = pdFALSE;

  // switch (GPIO_Pin) {
  //   case DWM_IRQ_PIN:
  //     xSemaphoreGiveFromISR(irqSemaphore, &higherPriorityTaskWoken);

  //     HAL_NVIC_ClearPendingIRQ(DWM_IRQn);
  //     break;
  //   default:
  //     break;
  // }
  // portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
