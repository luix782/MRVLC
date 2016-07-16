/*
 * pit.c
 *
 *  Created on: 29/06/2016
 *      Author: l.espinal
 */

#include "pit.h"

void PIT_HANDLER(void)
{

	/* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, PIT_TFLG_TIF_MASK);

    periodTime++;
    pitIsrFlag = true;

}

void initPIT(void)
{
    /* Structure of initialize PIT */
     pit_config_t pitConfig;

     /*
     * pitConfig.enableRunInDebug = false;
     */
    PIT_GetDefaultConfig(&pitConfig);

    /* Init pit module */
    PIT_Init(PIT, &pitConfig);

    /* Set timer period for channel 0 */

    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(pit_mSecValue, PIT_SOURCE_CLOCK)); // 1 sec

    /* Enable timer interrupts for channel 0 */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(PIT_IRQ_ID);
    /* Start channel 0 */
    //PRINTF("\r\nStarting channel No.0 ...\r\n");
    PIT_StartTimer(PIT, kPIT_Chnl_0);

}



void initMotorPins(void)
{
	//GPIO_PinInit(GPIO_Type *base, uint32_t pin, const gpio_pin_config_t *config)
	gpio_pin_config_t configPin;
	configPin.outputLogic = 0;
	configPin.pinDirection = kGPIO_DigitalOutput;
	GPIO_PinInit(GPIOA, ain1, &configPin);
	GPIO_PinInit(GPIOA, ain2, &configPin);
	GPIO_PinInit(GPIOA, stby, &configPin);
}

void motorForward(void)
{
	GPIO_ClearPinsOutput(GPIOA , 1U << stby);
	GPIO_ClearPinsOutput(GPIOA , 1U << ain1);
	GPIO_SetPinsOutput(GPIOA, 1U << ain2);
	GPIO_SetPinsOutput(GPIOA, 1U << stby);
}

void motorReverse(void)
{
	GPIO_ClearPinsOutput(GPIOA , 1U << stby);
	GPIO_ClearPinsOutput(GPIOA , 1U << ain2);
	GPIO_SetPinsOutput(GPIOA, 1U << ain1);
	GPIO_SetPinsOutput(GPIOA, 1U << stby);
}

void motorBreak(void)
{
	GPIO_ClearPinsOutput(GPIOA , 1U << ain1);
	GPIO_ClearPinsOutput(GPIOA , 1U << ain2);
}

void motorStanby(void)
{
	GPIO_ClearPinsOutput(GPIOA , 1U << stby);
}
