/*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This is template for main module created by New Kinetis SDK 2.x Project Wizard. Enjoy!
 **/
/*
 * 		INITIAL STEP USE BLUETOOTH CHANNEL  FOR TO FIND CONSTANTS FOR PID CONTROLLERS
 * 		how to use this from bluetooth console:
 * 		PRESS:
 *
 * 		n					// stop data burst and wait for receiving command
 * 		p<value><CR>		// Proportional constant value for PID balance
 * 		i<value><CR>		// Integral constant value for PID balance
 * 		d<value><CR>		// Derivative constant value for PID balance
 * 		c<value><CR>		// Sets offset of vertical position for PID balance
 * 		r<value><CR>		// Proportional constant value for PID steering
 * 		e<value><CR>		// Derivative constant value for PID steering
 * 		j<value><CR>		// Integral constant value for PID steering
 * 		s<value><CR>		// Set direction, how much left o right steering
 * 		f<value><CR>		// Set forward speed
 * 		x<CR>				// Stop forward motion
 *
 * 		ONCE DISCOVERED THE CONSTANT, BLUETOOTH CHANNEL IS USED FOR STEERING  THE MRVLC
 */

/*  Standard C Included Files */
#include <stdio.h>
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "stdio.h"
#include "fsl_pit.h"
#include "fsl_ftm.h"
#include "stdlib.h"
#include "math.h"

#include "pit.h"
#include "bt_com.h"
#include "pwm.h"
#include "PID.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/

// enable terminal messages
#define noMsg		0
#define msgConsole 	1
#define msgBT		2
#define msg 		msgBT	// noMsg // msgConsole // msgBT

#define fc_clock		150					// 120: 120Mhz, 150: 150Mhz

#define flexio_i2c		1					// define uso I2C
//#define flexio_uart	1					// define use FLEXIO_UART
//#define 	lpuart		1					// define use UART module


#define pidTimer		150					// time between pid calculations (mSec)
#define servoTimer		150					// max time for servo to change to a new position
#define numPidCounts	pidTimer/pit_mSec	//
#define numServoCounts	servoTimer/pit_mSec	//

#define pi				3.14159265358979
#define halfPi 			pi/2

#define minBalance 		(-halfPi)
#define maxBalance 		halfPi
#define minSteer		(-20*pi/180)
#define maxSteer		(20*pi/180)
#define ratioBalance 	(spanBalance)
#define ratioSteer		-(spanSteer)



#define toleranceBalance 0.02652582384		// +/- 1.5° to vertical position

#define bufLen			120




/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#ifdef flexio_i2c
extern void initAccel(void);
extern struct accelDat_t readAccelData(void);
#endif
void moveStr( char * str);
#ifdef flexio_uart
extern void initFLEXIO_UART(void);
#endif
void txMsgBT( char *str);
extern char * ftoa(double f, char * buf, int precision);
void showLeds(float val, float ref, float tol);



/*******************************************************************************
 * Variables
 ******************************************************************************/
struct accelDat_t{
	int16_t xDat;
	int16_t yDat;
	int16_t zDat;
};

uint16_t periodTime;			//

/*
 *  profiles to smoother servo movement
 *  n = counts -1 = numPidCounts  - 1
 */
const uint16_t pwmProfile[]={ 3, 8, 12, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2};	// counts: 15
//const uint16_t pwmProfile[]={ 2, 4, 6, 8, 9, 10, 11, 11, 10, 9, 8, 6, 4, 2};	// counts: 15
//const uint16_t pwmProfile[]={ 3, 9, 11, 12, 12, 11, 10, 9, 8, 7, 5, 3};		// counts: 13
//const uint16_t pwmProfile[]={ 5, 10, 12, 13, 12, 11, 10, 9, 8, 6, 4};		// counts: 12
//const uint16_t pwmProfile[]={ 3, 12, 16, 16, 14, 12, 10, 8, 6, 3};		// counts: 11
//const uint16_t pwmProfile[]={ 4, 15, 20, 20, 17, 13, 8, 3};		// counts: 9
//const uint16_t pwmProfile[]={ 3, 10, 17, 20, 20, 17, 10, 3};		// counts: 9
//const uint16_t pwmProfile[]={ 10, 18, 22, 22, 18, 10};			// counts: 7
//const uint16_t pwmProfile[]={ 7, 27, 32, 27 , 7};					// counts: 6
//const uint16_t pwmProfile[]={7, 20, 46, 20, 7};					// counts: 6


/*******************************************************************************
 * Code
 ******************************************************************************/


/*!
 * @brief Main function
 */
int main(void)
{
	const float nSamples = numPidCounts;
	uint16_t pidCounter = 0;
	uint16_t servoCounter = 0;

	char inBuf[bufLen];
	char outBuf[bufLen];
	char outBuf2[bufLen];
	char buf[bufLen];
	struct accelDat_t accelData;
    int32_t x, y, z;
    uint16_t actualDutyCycleSteer = dcCenterSteer;
    uint16_t actualDutyCycleBalance = dcCenterBalance;
    uint16_t newDutyCycleBalance;
    uint16_t newDutyCycleSteer;
    float val;
    float bip;
    float sip;
    float angSum = 0;
    float rAng;
    float angArray[numPidCounts];
    int16_t dcBalanceStepVal = 0;

    int16_t dcSteerStepVal = 0; 		// int16_t dcSteerMod = 0;
    uint16_t stepBalanceAcumulator;
    uint16_t stepSteerAcumulator;

    float cmdBalance = 0;				//
    float cmdSteer = 0;
    bool changeBalance = false;
    bool changeSteer = false;

    bool newString = false;
    bool newCommand = false;

    bool changeMotor = false;
    uint16_t propSpeed = 0;				// const to speed up
    uint16_t setSpeed = 0;				// speed settle by user
    uint16_t motorSpeed = 0;			// speed signal applied to the motor
    int16_t newMotorSpeed = 0;			// speed calculated

    // pid status and constants
    PIDstruct_t pidBalance;
    pidBalance.maxInteg = maxBalance;
    pidBalance.minInteg = minBalance;
	pidBalance.gainProp = 0;
	pidBalance.gainInteg = 0;
	pidBalance.gainDeriv = 0;

	PIDstruct_t pidSteer;
	pidSteer.maxInteg = maxSteer;
	pidSteer.minInteg = minSteer;
	pidSteer.gainProp = 0;
	pidSteer.gainInteg = 0;
	pidSteer.gainDeriv = 0;


/*
 * 	From actual sdk 2.0 examples, flexio_uart and flexio_i2c can not be used at same time.
 */
#ifdef flexio_uartd
    flexio_uart_transfer_t sendXfer;
    flexio_uart_transfer_t receiveXfer;
#endif



    BOARD_InitPins();

#if fc_clock==120
    BOARD_BootClockRUN();		// 120 Mhz
#else
    BOARD_BootClockHSRUN();		// 150 Mhz
#endif
    BOARD_InitDebugConsole();
    CLOCK_SetFlexio0Clock(2U);
	LED_INIT();  					// Initialize and enable LEDs
	initPIT();
	initMotorPins();
	initPWM();
	motorForward();
/**/

#ifdef flexio_i2c
	initAccel();
#endif

#ifdef lpuart
	initLPUART();
#endif

	memset(inBuf, '\0', bufLen*sizeof(inBuf[0]));
	memset(outBuf, '\0', bufLen*sizeof(outBuf[0]));
	memset(angArray, '\0', numPidCounts*sizeof(angArray[0]));

	periodTime=0;

#ifdef flexio_uart
	initFLEXIO_UART();

    // Send g_tipString out.
    xfer.data = g_tipString;
    xfer.dataSize = sizeof(g_tipString) - 1;
    txOnGoing = true;
    FLEXIO_UART_TransferSendNonBlocking(&uartDev, &g_uartHandle, &xfer);

    // Wait send finished
    while (txOnGoing)
    {
    }

    // Start to echo.
    sendXfer.data = g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

#endif

    while (true)
    {

#ifdef flexio_uart
    	// If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer.
        if ((!rxBufferEmpty) && (!txBufferFull))
        {
            memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
            rxBufferEmpty = true;
            txBufferFull = true;
        }

    	// If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer.
        if ((!rxOnGoing) && rxBufferEmpty)
        {
            rxOnGoing = true;
            FLEXIO_UART_TransferReceiveNonBlocking(&uartDev, &g_uartHandle, &receiveXfer, NULL);
        }

        // If TX is idle and g_txBuffer is full, start to send data.
        if ((!txOnGoing) && txBufferFull)
        {
            txOnGoing = true;
            FLEXIO_UART_TransferSendNonBlocking(&uartDev, &g_uartHandle, &sendXfer);
        }
#endif

#ifdef lpuart
// messages from / to console
		if(newString)					// sets value for pid system
		{
			newString = false;			// is it in command mode?
			if((inBuf[0]== 'p')||(inBuf[0]== 'd')||(inBuf[0]== 'i')||(inBuf[0]== 'c')||(inBuf[0]== 'r')\
					||(inBuf[0]== 'e')||(inBuf[0]== 'j')||(inBuf[0]== 's')|| (inBuf[0]=='f')||(inBuf[0]=='x'))
			{
				switch (inBuf[0])
				{
				case 'p':								// sets Proportional const
					moveStr(inBuf);
					pidBalance.gainProp  = atof(inBuf);
					strcpy(outBuf, "prop bal: ");
					ftoa((float)pidBalance.gainProp, buf, 10);
					break;
				case 'i':								// sets Integral const
					moveStr(inBuf);
					pidBalance.gainInteg = atof(inBuf);
					strcpy(outBuf, "integ bal: ");
					ftoa((float)pidBalance.gainInteg, buf, 10);
					break;
				case 'd':								// sets derivative const
					moveStr(inBuf);
					pidBalance.gainDeriv = atof(inBuf);
					strcpy(outBuf, "deriv bal: ");
					ftoa( (float)pidBalance.gainDeriv, buf, 10);
					break;
				case 'c':								// sets command value
					moveStr(inBuf);
					cmdBalance = atof(inBuf);
					strcpy(outBuf, "cmd bal: ");
					ftoa(cmdBalance, buf, 10);
					break;
				case 'r':								// sets command value
					moveStr(inBuf);
					pidSteer.gainProp = atof(inBuf);
					strcpy(outBuf, "prop steer: ");
					ftoa(pidSteer.gainProp, buf, 10);
					break;
				case 'e':								// sets command value
					moveStr(inBuf);
					pidSteer.gainDeriv = atof(inBuf);
					strcpy(outBuf, "deriv steer: ");
					ftoa(pidSteer.gainDeriv , buf, 10);
					break;
				case 'j':								// sets command value
					moveStr(inBuf);
					pidSteer.gainInteg = atof(inBuf);
					strcpy(outBuf, "integ steer: ");
					ftoa(pidSteer.gainInteg , buf, 10);
					break;
				case 's':								// sets command value
					moveStr(inBuf);
					cmdSteer = atof(inBuf);
					strcpy(outBuf, "cmd steer: ");
					ftoa(cmdSteer, buf, 10);
					break;
				case 'f':								// sets Proportional const
					moveStr(inBuf);
					setSpeed = atoi(inBuf);
					strcpy(outBuf, "motor Speed: ");
					itoa((float)motorSpeed, buf, 10);
					changeMotor = true;
					break;
				case 'x':								// sets Integral const
					moveStr(inBuf);
					setSpeed = motorStop;
					//motorSpeed = motorStop;
					strcpy(outBuf, "motor stop!");
					changeMotor = true;
					break;
				}
				newCommand = false;
			}
			strcat(outBuf, buf);
			strcat(outBuf, "\r\n");
			txMsgBT(outBuf);
			memset(inBuf, '\0', bufLen*sizeof(inBuf[0]));

		}
		//If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer.
		if ((!rxBufferEmpty) && (!txBufferFull))
		{
			memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
			rxBufferEmpty = true;
			txBufferFull = true;
			if(g_rxBuffer[0]=='n') newCommand = true;
			else if(g_rxBuffer[0]!='\n') strncat(inBuf, g_rxBuffer, 1);
			if(g_rxBuffer[0]=='\r') newString = true;
		 }
		/* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
		if ((!rxOnGoing) && rxBufferEmpty)
		{
			rxOnGoing = true;
		    receiveXfer.data = g_rxBuffer;
		    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
			LPUART_TransferReceiveNonBlocking(DEMO_LPUART, &g_lpuartHandle, &receiveXfer, NULL);
		}

		/* If TX is idle and g_txBuffer is full, start to send data. */
		if ((!txOnGoing) && txBufferFull)
		{
			txOnGoing = true;
		    sendXfer.data = g_txBuffer;
		    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
			LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &sendXfer);
		}
#endif

// timer overflow? new accel measurement
    	if (true == pitIsrFlag)					// Check for PIT interupt
        {
    		pitIsrFlag=false;

    		accelData=readAccelData();			// 1st read data from accelerometer
			x=accelData.xDat;
			y=accelData.yDat;
			z=accelData.zDat;
			accelData=readAccelData();			// 2nd read data from accelerometer
			x+=accelData.xDat;
			y+=accelData.yDat;
			z+=accelData.zDat;
			x/=2;
			//y/=2;		//
			z/=2;
			rAng = atan2(x,z);					// angle in rads
			angSum += (rAng-angArray[pidCounter]); // sums last n samples, n = numPidCounts
			angArray[pidCounter] = rAng;
    		pidCounter++;
    		if(pidCounter >= (numPidCounts))		// calculate new PID control
     		{
    			pidCounter=0;

    			val=angSum/nSamples;

    			if(setSpeed != 0)
    			{
    					motorSpeed = (uint16_t)((float)propSpeed * fabs(cmdBalance - val)) + setSpeed; // if falling, accelerate
    			}
				if(motorSpeed != setSpeed) changeMotor = true;

    			bip = pidControl(&pidBalance, (cmdBalance - val), val);
				sip = pidControl(&pidSteer, (cmdSteer - val), val);

				newDutyCycleBalance = (uint16_t)((int16_t)dcCenterBalance + (int16_t) (bip * (float)ratioBalance));
				newDutyCycleSteer = (uint16_t)((int16_t)dcCenterSteer + (int16_t) (sip * (float)ratioSteer));

				dcBalanceStepVal = newDutyCycleBalance - actualDutyCycleBalance;
				changeBalance = true;
				dcSteerStepVal = newDutyCycleSteer - actualDutyCycleSteer;
				changeSteer = true;
				servoCounter = 0;
				stepBalanceAcumulator = 0;
				stepSteerAcumulator=0;
				showLeds(val, cmdBalance, (float) toleranceBalance);
     		}


    		if(!newCommand)	// if it's not in command mode, values -> console
    		{
				itoa(periodTime, outBuf);
				strcat(outBuf, "    ");
				ftoa((double)val, buf, 10);
				strcat(outBuf, buf);
				strcat(outBuf, "    ");
				ftoa((double)bip, buf, 10);
				strcat(outBuf, buf);
				strcat(outBuf, "    ");
				ftoa((double)sip, buf, 10);
				strcat(outBuf, buf);
				strcat(outBuf, "    ");
	//			itoa((int16_t)newDutyCycleBalance, buf);
	//			strcat(outBuf, buf);
				strcat(outBuf, " \r");
#if msg == msgConsole
				puts(outBuf);
#elif msg == msgBT
				strcpy(outBuf2, outBuf);
				txMsgBT(outBuf2);
#endif
				memset(outBuf, '\0', bufLen*sizeof(outBuf[0]));
    		}

// modify servos and motor
    		if(changeMotor)
    		{
    			changeMotor = false;
    			if(setSpeed == motorStop) motorSpeed = motorStop;
    			else if(newMotorSpeed > maxSpeed) motorSpeed = maxSpeed;
    			else motorSpeed = newMotorSpeed;
    			PWM_UpdatePwmDutycycle(PWM_MOTOR_FTM_BASEADDR, (ftm_chnl_t)PWM_MOTOR_CHNL, kFTM_EdgeAlignedPwm, motorSpeed);
    			FTM_SetSoftwareTrigger(PWM_MOTOR_FTM_BASEADDR, true); // Software trigger to update registers
    		}
			if(servoCounter < (numServoCounts-1) )
			{
				if(changeBalance)
				{
					actualDutyCycleBalance += dcBalanceStepVal*pwmProfile[servoCounter]/100;
					stepBalanceAcumulator += dcBalanceStepVal*pwmProfile[servoCounter]/100;
				}
				if(changeSteer)
				{
					actualDutyCycleSteer += dcSteerStepVal*pwmProfile[servoCounter]/100;
					stepSteerAcumulator += dcSteerStepVal*pwmProfile[servoCounter]/100;
				}
			}
			else
			{
				if(changeBalance)
				{
					actualDutyCycleBalance += dcBalanceStepVal - stepBalanceAcumulator;
					changeBalance=false;
				}
				if(changeSteer)
				{
					actualDutyCycleSteer += dcSteerStepVal - stepSteerAcumulator;
					changeSteer=false;
				}
			}
    		// Start PWM mode with updated duty cycle

    		if(changeBalance)
			{
				if(actualDutyCycleBalance > dcMaxBalance) actualDutyCycleBalance = dcMaxBalance;	// check borders
				if(actualDutyCycleBalance < dcMinBalance) actualDutyCycleBalance = dcMinBalance;	//
				PWM_UpdatePwmDutycycle(PWM_FTM_BASEADDR, (ftm_chnl_t)PWM_BALANCE_CHNL, kFTM_EdgeAlignedPwm, actualDutyCycleBalance);
				FTM_SetSoftwareTrigger(PWM_FTM_BASEADDR, true); // Software trigger to update registers
			}
			if(changeSteer)
			{
				if(actualDutyCycleSteer > dcMaxSteer) actualDutyCycleSteer = dcMaxSteer;
				if(actualDutyCycleSteer < dcMinSteer) actualDutyCycleSteer = dcMinSteer;
				PWM_UpdatePwmDutycycle(PWM_FTM_BASEADDR, (ftm_chnl_t)PWM_STEER_CHNL, kFTM_EdgeAlignedPwm, actualDutyCycleSteer);
				FTM_SetSoftwareTrigger(PWM_FTM_BASEADDR, true); // Software trigger to update registers
			}
			servoCounter++;
		}
    }

    while (1)
    {
    }
}

/*
 * 	moveStr: move elements from str one position ahead
 */
void moveStr( char * str)
{
	uint16_t i,l;
	l=strlen(str);
	for(i=0; i < l; i++)
	{
		str[i]=str[i+1];
	}
	str[i]='\0';
}

/*
 *	txMsgBT: transfer message through bluetooth terminal
 */
void txMsgBT( char *str)
{
	lpuart_transfer_t xfer;

	xfer.data = str;
    xfer.dataSize = strlen(str); 			//sizeof(str) - 1;
    txOnGoing = true;
    LPUART_TransferSendNonBlocking(DEMO_LPUART, &g_lpuartHandle, &xfer);
}

/*
 * 	showLeds: signal if mrvlc is too left inclined (red), too right inclined (green) or almost vertical (blue)
 */
void showLeds(float val, float ref, float tol)
{
	if(fabs(val-ref) <= tol )  // you are near vertical
	{
		LED_RED_OFF();
		LED_GREEN_OFF();
		LED_BLUE_ON();
	}
	else if(val < 0 )								// too inclined to left
	{
		LED_GREEN_OFF();
		LED_BLUE_OFF();
		LED_RED_ON();
	}
	else if(val > 0)								// too inclined to right
	{
		LED_RED_OFF();
		LED_BLUE_OFF();
		LED_GREEN_ON();
	}
}
