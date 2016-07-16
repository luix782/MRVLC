/*
 * modFSL_ftm.h
 *
 *  Created on: 13/07/2016
 *      Author: l.espinal
 */

#ifndef SOURCE_MODFSL_FTM_H_
#define SOURCE_MODFSL_FTM_H_

void Mod_FTM_SetPwmSync(FTM_Type *base, uint32_t syncMethod);
status_t Mod_FTM_Init(FTM_Type *base, const ftm_config_t *config);
status_t Mod_FTM_SetupPwm(FTM_Type *base,
                      const ftm_chnl_pwm_signal_param_t *chnlParams,
                      uint8_t numOfChnls,
                      ftm_pwm_mode_t mode,
                      uint32_t pwmFreq_Hz,
                      uint32_t srcClock_Hz);
void PWM_UpdatePwmDutycycle(FTM_Type *base,
                            ftm_chnl_t chnlNumber,
                            ftm_pwm_mode_t currentPwmMode,
                            uint16_t dutyCycleValue);

#endif /* SOURCE_MODFSL_FTM_H_ */
