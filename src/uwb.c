#include "uwb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stm32f0xx_hal.h>
#include <stdbool.h>
#include <stdio.h>

static bool isInit = false;
static SemaphoreHandle_t irqSemaphore;

void uwbInit()
{
    printf("uwbInit");
    static StaticSemaphore_t irqSemaphoreBuffer;
    irqSemaphore = xSemaphoreCreateBinaryStatic(&irqSemaphoreBuffer);

    isInit = true;

    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);
    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    { };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        isInit = false;
        return;
    }
    if (dwt_configure(&config) == DWT_ERROR)
    {
        isInit = false;
        return;
    }
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    dwt_setrxaftertxdelay(0);

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);
    /* auto re-enable receiver After a frame reception failure (except a frame wait timeout), the receiver will re-enable to re-attempt reception.*/
    dwt_or32bitoffsetreg(SYS_CFG_ID, 0, SYS_CFG_RXAUTR_BIT_MASK);
    dwt_setrxtimeout(500000);

    dwt_setcallbacks(&tx_cb, &rx_cb, &rx_to_cb, &rx_err_cb, NULL, NULL);
    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                         SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                         SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                     0, DWT_ENABLE_INT);

    /*Clearing the SPI ready interrupt*/
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    isInit = true;
}

static int checkIrq()
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
}

static uint8_t tx_msg[] = {'C', 'b', 'A', 'A', 'A', 'A', 'A', 'A', 'A', 'A'};
#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN)
static void uwbTask(void *parameters)
{   
    while (!isInit) {
        printf("false");
        vTaskDelay(1000);
    }
    while (true)
    {   
        if (xSemaphoreTake(irqSemaphore, 100))
        {   
            do
            {
                dwt_isr();
            } while (checkIrq() != 0);
        }
        // dwt_writetxdata(FRAME_LENGTH-FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */
        // dwt_writetxfctrl(FRAME_LENGTH, 0, 0); 

        // /* Start transmission. */
        // dwt_starttx(DWT_START_TX_IMMEDIATE);
        // /* Clear TX frame sent event. */
        // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        
        // printf("TX Frame Sent");
    }
}

void uwbStart()
{   
    printf("uwbStart");
    static StaticTask_t uwbStaticTask;
    static StackType_t uwbStaticStack[2 * configMINIMAL_STACK_SIZE];

    xTaskCreateStatic(uwbTask, "uwbTask", 2 * configMINIMAL_STACK_SIZE, NULL,
                      configMAX_PRIORITIES - 1, uwbStaticStack, &uwbStaticTask);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // printf("HAL_GPIO_EXTI_Callback\n");
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    switch (GPIO_Pin)
    {
    case GPIO_PIN_0:
        xSemaphoreGiveFromISR(irqSemaphore, &higherPriorityTaskWoken);

        HAL_NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
        break;
    default:
        break;
    }
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static void rx_cb() {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    printf("rx_cb\r\n");
}

static void tx_cb() {
    // printf("tx_cb");
}

static void rx_to_cb() {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    printf("rx_to_cb\r\n");
}

static void rx_err_cb() {
    printf("rx_err_cb\r\n");
}