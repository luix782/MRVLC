/*
 * pwm.h
 *
 *  Created on: 23/06/2016
 *      Author: l.espinal
 */

#ifndef SOURCE_PWM_H_
#define SOURCE_PWM_H_

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
#include "modFSL_ftm.h"

/* The Flextimer instance/channel used for board */
#define PWM_FTM_BASEADDR FTM2
#define PWM_STEER_CHNL 0U
#define PWM_BALANCE_CHNL 1U
/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define PWM_FCIA		100U

#define PWM_MOTOR_FTM_BASEADDR	FTM1		// propulsion motor
#define PWM_MOTOR_CHNL			0U

#define modPWM			46874
#define spanBalance		6000	//
#define dcCenterBalance	7200	//
#define dcMinBalance	dcCenterBalance - spanBalance	// 	left
#define dcMaxBalance	dcCenterBalance + spanBalance	// 	right

#define dcCenterSteer	6700	//
#define spanSteer		2500	//
#define dcMinSteer		dcCenterSteer - spanSteer		// 	left
#define dcMaxSteer		dcCenterSteer + spanSteer		//	right

#define maxSpeed		780		// duty cycle: 100% for PWM motor
#define minSpeed		350		// duty cycle: aprox. 50%, mrvlc barely moves
#define motorStop		0

void initPWM(void);

void PWM_UpdatePwmDutycycle(FTM_Type *base,
                            ftm_chnl_t chnlNumber,
                            ftm_pwm_mode_t currentPwmMode,
                            uint16_t dutyCycleValue);
status_t Mod_FTM_SetupPwm(FTM_Type *base,
                      const ftm_chnl_pwm_signal_param_t *chnlParams,
                      uint8_t numOfChnls,
                      ftm_pwm_mode_t mode,
                      uint32_t pwmFreq_Hz,
                      uint32_t srcClock_Hz);

#endif /* SOURCE_PWM_H_ */
