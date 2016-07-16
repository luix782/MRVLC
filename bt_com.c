/*
 * bt_uart.c
 *
 *  Created on: 18/06/2016
 *      Author: l.espinal
 */

#include "bt_com.h"

#ifdef flexio_uart
/* UART user callback */
void FLEXIO_UART_UserCallback(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_FLEXIO_UART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_FLEXIO_UART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}

void initFLEXIO_UART(void)
{
	flexio_uart_config_t config;
	// ********************************************************
	// config.enableUart = true;
	// config.enableInDoze = false;
	// config.enableInDebug = true;
	// config.enableFastAccess = false;
	// config.baudRate_Bps = 115200U;
	// config.bitCountPerChar = kFLEXIO_UART_8BitsPerChar;
	// ********************************************************
    FLEXIO_UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableUart = true;

    uartDev.flexioBase = BOARD_FLEXIO_BASE;
    uartDev.TxPinIndex = FLEXIO_UART_TX_PIN;
    uartDev.RxPinIndex = FLEXIO_UART_RX_PIN;
    uartDev.shifterIndex[0] = 2U;
    uartDev.shifterIndex[1] = 3U;
    uartDev.timerIndex[0] = 2U;
    uartDev.timerIndex[1] = 3U;
/*
    uartDev.shifterIndex[0] = 0U;
    uartDev.shifterIndex[1] = 1U;
    uartDev.timerIndex[0] = 0U;
    uartDev.timerIndex[1] = 1U;
*/
    FLEXIO_UART_Init(&uartDev, &config, FLEXIO_CLOCK_FREQUENCY);
    FLEXIO_UART_TransferCreateHandle(&uartDev, &g_uartHandle, FLEXIO_UART_UserCallback, NULL);
}
#endif

#ifdef lpuart
void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_LPUART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_LPUART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}


void initLPUART(void)
{
    lpuart_config_t config;
    lpuart_transfer_t xfer;

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kLPUART_ParityDisabled;
     * config.stopBitCount = kLPUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 0;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    LPUART_Init(DEMO_LPUART, &config, CLOCK_GetFreq(DEMO_LPUART_CLKSRC));
    LPUART_TransferCreateHandle(DEMO_LPUART, &g_lpuartHandle, LPUART_UserCallback, NULL);
    /* Send g_tipString out. */
    xfer.data = g_tipString;
    xfer.dataSize = sizeof(g_tipString) - 1;
    txOnGoing = true;
    LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &xfer);

    /* Wait send finished */
    while (txOnGoing)
    {
    }
    /* Start to echo. */


}
#endif
