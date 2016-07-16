/*
 * bt_uart.h
 *
 *  Created on: 18/06/2016
 *      Author: l.espinal
 */

#ifndef SOURCE_BT_COM_H_
#define SOURCE_BT_COM_H_

#include <stdio.h>
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"

#include "fsl_flexio_uart.h"
#include "fsl_lpuart.h"


#define lpuart
// #define flexio_uart

#ifdef flexio_uart
#define BOARD_FLEXIO_BASE FLEXIO0
#define FLEXIO_CLOCK_FREQUENCY 12000000U
#define FLEXIO_UART_TX_PIN 3U
#define FLEXIO_UART_RX_PIN 2U
//#define FLEXIO_CLOCK_FREQUENCY 12000000U // already defined
#define ECHO_BUFFER_LENGTH 1

flexio_uart_handle_t g_uartHandle;
FLEXIO_UART_Type uartDev;
uint8_t g_tipString[] = "\r\n**** Flexio uart enabled ****\r\n";
/* UART user callback */
void FLEXIO_UART_UserCallback(FLEXIO_UART_Type *base, flexio_uart_handle_t *handle, status_t status, void *userData);
#endif

#ifdef lpuart
// **********************************************************************
//	LPUART0 MODULE
// **********************************************************************
#define DEMO_LPUART LPUART0
#define DEMO_LPUART_CLKSRC kCLOCK_Osc0ErClk
#define ECHO_BUFFER_LENGTH 1

lpuart_handle_t g_lpuartHandle;

uint8_t g_tipString[] = "\r\n**** LPUART ENABLED ****\r\n";

lpuart_transfer_t sendXfer;
lpuart_transfer_t receiveXfer;
uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;
lpuart_transfer_t xfer;

void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
void initLPUART(void);
lpuart_config_t config;
#endif

#endif /* SOURCE_BT_COM_H_ */
