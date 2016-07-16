/*
 * pit.h
 *
 *  Created on: 29/06/2016
 *      Author: l.espinal
 */

#ifndef SOURCE_PIT_H_
#define SOURCE_PIT_H_

#include <stdio.h>
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "stdio.h"
#include "fsl_pit.h"

#define RED_LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define GREEN_LED_INIT() LED_GREEN_INIT(LOGIC_LED_ON)
#define BLUE_LED_INIT() LED_BLUE_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()
#define LED_INIT()	RED_LED_INIT(),\
					GREEN_LED_INIT(),\
					BLUE_LED_INIT(),\
					LED_RED_OFF(),\
					LED_GREEN_OFF(),\
					LED_BLUE_OFF();

// *** def for motor driver ***
#define ain1	5
#define ain2 	16
#define stby	13


#define PIT_HANDLER PIT0_IRQHandler
#define PIT_IRQ_ID PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

#define pit_mSec		10					//1000: 1000 mSec, 500: 500 mSec, 5: 5 mSec
#define pit_mSecValue 	pit_mSec*1000

extern uint16_t periodTime;

volatile bool pitIsrFlag = false;

void PIT_HANDLER(void);
void initPIT(void);

void initMotorPins(void);
void motorForward(void);
void motorReverse(void);
void motorBreak(void);
void motorStanby(void);


#endif /* SOURCE_PIT_H_ */
