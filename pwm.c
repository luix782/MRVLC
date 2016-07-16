/*
 * pwm.c
 *
 *  Created on: 23/06/2016
 *      Author: l.espinal
 */
#include "pwm.h"


void initPWM(void)
{

    ftm_config_t ftmInfo;
    //uint8_t updatedDutycycle = 0U;
    ftm_chnl_pwm_signal_param_t ftmParam[2];

    /* Configure ftm params with frequency 100 HZ */
    ftmParam[0].chnlNumber = (ftm_chnl_t)PWM_STEER_CHNL;
    ftmParam[0].level = kFTM_HighTrue;
    //ftmParam[0].level = kFTM_LowTrue;
    ftmParam[0].dutyCyclePercent = 0U;
    ftmParam[0].firstEdgeDelayPercent = 0U;

    ftmParam[1].chnlNumber = (ftm_chnl_t)PWM_BALANCE_CHNL;
    ftmParam[1].level = kFTM_HighTrue;
    //ftmParam[1].level = kFTM_LowTrue;
    ftmParam[1].dutyCyclePercent = 0U;
    ftmParam[1].firstEdgeDelayPercent = 0U;


    /* Print a note to terminal */
    PRINTF("\r\n**** FTM PWM outputs enabled ****\r\n");

    /*
     * ftmInfo.prescale = kFTM_Prescale_Divide_1;
     * ftmInfo.bdmMode = kFTM_BdmMode_0;
     * ftmInfo.pwmSyncMode = kFTM_SoftwareTrigger;
     * ftmInfo.reloadPoints = 0;
     * ftmInfo.faultMode = kFTM_Fault_Disable;
     * ftmInfo.faultFilterValue = 0;
     * ftmInfo.deadTimePrescale = kFTM_Deadtime_Prescale_1;
     * ftmInfo.deadTimeValue = 0;
     * ftmInfo.extTriggers = 0;
     * ftmInfo.chnlInitState = 0;
     * ftmInfo.chnlPolarity = 0;
     * ftmInfo.useGlobalTimeBase = false;
     */
    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale=kFTM_Prescale_Divide_16;
    ftmInfo.reloadPoints= kFTM_CntMax;
    /* Initialize FTM module */
    Mod_FTM_Init(PWM_FTM_BASEADDR, &ftmInfo);	// Modified
    Mod_FTM_SetupPwm(PWM_FTM_BASEADDR, ftmParam, 2U, kFTM_EdgeAlignedPwm, PWM_FCIA, FTM_SOURCE_CLOCK);
    FTM_StartTimer(PWM_FTM_BASEADDR, kFTM_SystemClock);

	PWM_UpdatePwmDutycycle(PWM_FTM_BASEADDR, (ftm_chnl_t)PWM_STEER_CHNL, kFTM_EdgeAlignedPwm, dcCenterSteer);
	PWM_UpdatePwmDutycycle(PWM_FTM_BASEADDR, (ftm_chnl_t)PWM_BALANCE_CHNL, kFTM_EdgeAlignedPwm, dcCenterBalance);
	// Software trigger to update registers
	FTM_SetSoftwareTrigger(PWM_FTM_BASEADDR, true);

	// init motor pwm
	ftmParam[0].chnlNumber = PWM_MOTOR_CHNL;
	ftmInfo.prescale = kFTM_Prescale_Divide_4;
	FTM_Init(PWM_MOTOR_FTM_BASEADDR, &ftmInfo);
    Mod_FTM_SetupPwm(PWM_MOTOR_FTM_BASEADDR, ftmParam, 1U, kFTM_EdgeAlignedPwm, 24000U, FTM_SOURCE_CLOCK);
    FTM_StartTimer(PWM_MOTOR_FTM_BASEADDR, kFTM_SystemClock);

}


