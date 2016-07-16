/*
 * modFSL_ftm.c
 *
 *  Created on: 13/07/2016
 *      Author: l.espinal
 */

#include "fsl_ftm.h"
#include "modFSL_ftm.h"


/*! @brief Pointers to FTM bases for each instance. */
static FTM_Type *const s_ftmBases[] = FTM_BASE_PTRS;
/*! @brief Pointers to FTM clocks for each instance. */
static const clock_ip_name_t s_ftmClocks[] = FTM_CLOCKS;


static void FTM_SetReloadPoints(FTM_Type *base, uint32_t reloadPoints)
{
    uint32_t chnlNumber = 0;
    uint32_t reg = 0;

    /* Need CNTINC bit to be 1 for CNTIN register to update with its buffer value on reload  */
    base->SYNCONF |= FTM_SYNCONF_CNTINC_MASK;

    reg = base->COMBINE;
    for (chnlNumber = 0; chnlNumber < (FSL_FEATURE_FTM_CHANNEL_COUNTn(base) / 2); chnlNumber++)
    {
        /* Need SYNCEN bit to be 1 for CnV reg to update with its buffer value on reload  */
        reg |= (1U << (FTM_COMBINE_SYNCEN0_SHIFT + (FTM_COMBINE_COMBINE1_SHIFT * chnlNumber)));
    }
    base->COMBINE = reg;

    /* Set the reload points */
    reg = base->PWMLOAD;

    /* Enable the selected channel match reload points */
    reg &= ~((1U << FSL_FEATURE_FTM_CHANNEL_COUNTn(base)) - 1);
    reg |= (reloadPoints & ((1U << FSL_FEATURE_FTM_CHANNEL_COUNTn(base)) - 1));

#if defined(FSL_FEATURE_FTM_HAS_HALFCYCLE_RELOAD) && (FSL_FEATURE_FTM_HAS_HALFCYCLE_RELOAD)
    /* Enable half cycle match as a reload point */
    if (reloadPoints & kFTM_HalfCycMatch)
    {
        reg |= FTM_PWMLOAD_HCSEL_MASK;
    }
    else
    {
        reg &= ~FTM_PWMLOAD_HCSEL_MASK;
    }
#endif /* FSL_FEATURE_FTM_HAS_HALFCYCLE_RELOAD */

    base->PWMLOAD = reg;

    /* These reload points are used when counter is in up-down counting mode */
    reg = base->SYNC;
    if (reloadPoints & kFTM_CntMax)
    {
        /* Reload when counter turns from up to down */
        reg |= FTM_SYNC_CNTMAX_MASK;
    }
    else
    {
        reg &= ~FTM_SYNC_CNTMAX_MASK;
    }

    if (reloadPoints & kFTM_CntMin)
    {
        /* Reload when counter turns from down to up */
        reg |= FTM_SYNC_CNTMIN_MASK;
    }
    else
    {
        reg &= ~FTM_SYNC_CNTMIN_MASK;
    }
    base->SYNC = reg;
}


static uint32_t FTM_GetInstance(FTM_Type *base)
{
    uint32_t instance;
    uint32_t ftmArrayCount = (sizeof(s_ftmBases) / sizeof(s_ftmBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ftmArrayCount; instance++)
    {
        if (s_ftmBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ftmArrayCount);

    return instance;
}
status_t Mod_FTM_Init(FTM_Type *base, const ftm_config_t *config)
{
    assert(config);

    uint32_t reg;

    if (!(config->pwmSyncMode &
          (FTM_SYNC_TRIG0_MASK | FTM_SYNC_TRIG1_MASK | FTM_SYNC_TRIG2_MASK | FTM_SYNC_SWSYNC_MASK)))
    {
        /* Invalid PWM sync mode */
        return kStatus_Fail;
    }

    /* Ungate the FTM clock*/
    CLOCK_EnableClock(s_ftmClocks[FTM_GetInstance(base)]);

    /* Configure the fault mode, enable FTM mode and disable write protection */
    base->MODE = FTM_MODE_FAULTM(config->faultMode) | FTM_MODE_FTMEN_MASK | FTM_MODE_WPDIS_MASK;

    /* Configure the update mechanism for buffered registers */
    Mod_FTM_SetPwmSync(base, config->pwmSyncMode);

    if (config->reloadPoints)
    {
        /* Setup intermediate register reload points */
        FTM_SetReloadPoints(base, config->reloadPoints);
    }

    /* Set the clock prescale factor */
    base->SC = FTM_SC_PS(config->prescale);

    /* Setup the counter operation */
    base->CONF = (FTM_CONF_BDMMODE(config->bdmMode) | FTM_CONF_GTBEEN(config->useGlobalTimeBase));

    /* Initial state of channel output */
    base->OUTINIT = config->chnlInitState;

    /* Channel polarity */
    base->POL = config->chnlPolarity;

    /* Set the external trigger sources */
    base->EXTTRIG = config->extTriggers;
#if defined(FSL_FEATURE_FTM_HAS_RELOAD_INITIALIZATION_TRIGGER) && (FSL_FEATURE_FTM_HAS_RELOAD_INITIALIZATION_TRIGGER)
    if (config->extTriggers & kFTM_ReloadInitTrigger)
    {
        base->CONF |= FTM_CONF_ITRIGR_MASK;
    }
    else
    {
        base->CONF &= ~FTM_CONF_ITRIGR_MASK;
    }
#endif /* FSL_FEATURE_FTM_HAS_RELOAD_INITIALIZATION_TRIGGER */

    /* FTM deadtime insertion control */
    base->DEADTIME = (FTM_DEADTIME_DTPS(config->deadTimePrescale) | FTM_DEADTIME_DTVAL(config->deadTimeValue));

    /* FTM fault filter value */
    reg = base->FLTCTRL;
    reg &= ~FTM_FLTCTRL_FFVAL_MASK;
    reg |= FTM_FLTCTRL_FFVAL(config->faultFilterValue);
    base->FLTCTRL = reg;

    return kStatus_Success;
}



void Mod_FTM_SetPwmSync(FTM_Type *base, uint32_t syncMethod)
{
   uint8_t chnlNumber = 0;
   uint32_t reg = 0, syncReg = 0;

   syncReg = base->SYNC;
   /* Enable PWM synchronization of output mask register */
   syncReg |= FTM_SYNC_SYNCHOM_MASK;

   reg = base->COMBINE;
   for (chnlNumber = 0; chnlNumber < (FSL_FEATURE_FTM_CHANNEL_COUNTn(base) / 2); chnlNumber++)
   {
       /* Enable PWM synchronization of registers C(n)V and C(n+1)V */
       reg |= (1U << (FTM_COMBINE_SYNCEN0_SHIFT + (FTM_COMBINE_COMBINE1_SHIFT * chnlNumber)));
   }
   base->COMBINE = reg;

   reg = base->SYNCONF;

   /* Use enhanced PWM synchronization method. Use PWM sync to update register values */
   reg |= (FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_CNTINC_MASK | FTM_SYNCONF_INVC_MASK | FTM_SYNCONF_SWOC_MASK);

   if (syncMethod & FTM_SYNC_SWSYNC_MASK)
   {
       /* Enable needed bits for software trigger to update registers with its buffer value */

/*
*    reg |= (FTM_SYNCONF_SWRSTCNT_MASK | FTM_SYNCONF_SWWRBUF_MASK | FTM_SYNCONF_SWINVC_MASK |
*          FTM_SYNCONF_SWSOC_MASK | FTM_SYNCONF_SWOM_MASK);
*	// FTM_SYNCONF_SWRSTCNT_MASK resets FTM counter each time CnV reg is written, is no good for servos
*
*/
       reg |= ( FTM_SYNCONF_SWWRBUF_MASK | FTM_SYNCONF_SWINVC_MASK | FTM_SYNCONF_SWSOC_MASK | FTM_SYNCONF_SWOM_MASK);
   }

   if (syncMethod & (FTM_SYNC_TRIG0_MASK | FTM_SYNC_TRIG1_MASK | FTM_SYNC_TRIG2_MASK))
   {
       /* Enable needed bits for hardware trigger to update registers with its buffer value */
       reg |= (FTM_SYNCONF_HWRSTCNT_MASK | FTM_SYNCONF_HWWRBUF_MASK | FTM_SYNCONF_HWINVC_MASK |
               FTM_SYNCONF_HWSOC_MASK | FTM_SYNCONF_HWOM_MASK);

       /* Enable the appropriate hardware trigger that is used for PWM sync */
       if (syncMethod & FTM_SYNC_TRIG0_MASK)
       {
           syncReg |= FTM_SYNC_TRIG0_MASK;
       }
       if (syncMethod & FTM_SYNC_TRIG1_MASK)
       {
           syncReg |= FTM_SYNC_TRIG1_MASK;
       }
       if (syncMethod & FTM_SYNC_TRIG2_MASK)
       {
           syncReg |= FTM_SYNC_TRIG2_MASK;
       }
   }

   /* Write back values to the SYNC register */
   base->SYNC = syncReg;

   /* Write the PWM synch values to the SYNCONF register */
   base->SYNCONF = reg;
}



status_t Mod_FTM_SetupPwm(FTM_Type *base,
                      const ftm_chnl_pwm_signal_param_t *chnlParams,
                      uint8_t numOfChnls,
                      ftm_pwm_mode_t mode,
                      uint32_t pwmFreq_Hz,
                      uint32_t srcClock_Hz)
{
    assert(chnlParams);

    uint32_t mod, reg;
    uint32_t ftmClock = (srcClock_Hz / (1U << (base->SC & FTM_SC_PS_MASK)));
    uint16_t cnv, cnvFirstEdge;
    uint8_t i;

    switch (mode)
    {
        case kFTM_EdgeAlignedPwm:
        case kFTM_CombinedPwm:
            base->SC &= ~FTM_SC_CPWMS_MASK;
            mod = (ftmClock / pwmFreq_Hz) - 1;
            break;
        case kFTM_CenterAlignedPwm:
            base->SC |= FTM_SC_CPWMS_MASK;
            mod = ftmClock / (pwmFreq_Hz * 2);
            break;
        default:
            return kStatus_Fail;
    }

    /* Return an error in case we overflow the registers, probably would require changing
     * clock source to get the desired frequency */
    if (mod > 65535U)
    {
        return kStatus_Fail;
    }
    /* Set the PWM period */
    base->MOD = mod;

    /* Setup each FTM channel */
    for (i = 0; i < numOfChnls; i++)
    {
        /* Return error if requested dutycycle is greater than the max allowed */
        if (chnlParams->dutyCyclePercent > 100)
        {
            return kStatus_Fail;
        }

        if ((mode == kFTM_EdgeAlignedPwm) || (mode == kFTM_CenterAlignedPwm))
        {
            /* Clear the current mode and edge level bits */
            reg = base->CONTROLS[chnlParams->chnlNumber].CnSC;
            reg &= ~(FTM_CnSC_MSA_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);

            /* Setup the active level */
/**********************************************************************************************
* 	THIS MACRO DEFINITION HAS A BUG, IT DOESN'T WORKS FOR TRUE_HIGH PULSES
*/
            reg |= (FTM_CnSC_ELSA(chnlParams->level) | FTM_CnSC_ELSB(chnlParams->level));
            if(chnlParams->level==kFTM_HighTrue)reg|=0x08;	// workaround: easy and dirty
/**********************************************************************************************/

            /* Edge-aligned mode needs MSB to be 1, don't care for Center-aligned mode */
            reg |= FTM_CnSC_MSB(1U);

            /* Update the mode and edge level */
            base->CONTROLS[chnlParams->chnlNumber].CnSC = reg;

            if (chnlParams->dutyCyclePercent == 0)
            {
                /* Signal stays low */
                cnv = 0;
            }
            else
            {
                cnv = (mod * chnlParams->dutyCyclePercent) / 100;
                /* For 100% duty cycle */
                if (cnv >= mod)
                {
                    cnv = mod + 1;
                }
            }

            base->CONTROLS[chnlParams->chnlNumber].CnV = cnv;
        }
        else
        {
            /* This check is added for combined mode as the channel number should be the pair number */
            if (chnlParams->chnlNumber >= (FSL_FEATURE_FTM_CHANNEL_COUNTn(base) / 2))
            {
                return kStatus_Fail;
            }

            /* Return error if requested value is greater than the max allowed */
            if (chnlParams->firstEdgeDelayPercent > 100)
            {
                return kStatus_Fail;
            }

            /* Configure delay of the first edge */
            if (chnlParams->firstEdgeDelayPercent == 0)
            {
                /* No delay for the first edge */
                cnvFirstEdge = 0;
            }
            else
            {
                cnvFirstEdge = (mod * chnlParams->firstEdgeDelayPercent) / 100;
            }

            /* Configure dutycycle */
            if (chnlParams->dutyCyclePercent == 0)
            {
                /* Signal stays low */
                cnv = 0;
                cnvFirstEdge = 0;
            }
            else
            {
                cnv = (mod * chnlParams->dutyCyclePercent) / 100;
                /* For 100% duty cycle */
                if (cnv >= mod)
                {
                    cnv = mod + 1;
                }
            }

            /* Clear the current mode and edge level bits for channel n */
            reg = base->CONTROLS[chnlParams->chnlNumber * 2].CnSC;
            reg &= ~(FTM_CnSC_MSA_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);

            /* Setup the active level for channel n */
            reg |= (FTM_CnSC_ELSA(chnlParams->level) | FTM_CnSC_ELSB(chnlParams->level));

            /* Update the mode and edge level for channel n */
            base->CONTROLS[chnlParams->chnlNumber * 2].CnSC = reg;

            /* Clear the current mode and edge level bits for channel n + 1 */
            reg = base->CONTROLS[(chnlParams->chnlNumber * 2) + 1].CnSC;
            reg &= ~(FTM_CnSC_MSA_MASK | FTM_CnSC_MSB_MASK | FTM_CnSC_ELSA_MASK | FTM_CnSC_ELSB_MASK);

            /* Setup the active level for channel n + 1 */
            reg |= (FTM_CnSC_ELSA(chnlParams->level) | FTM_CnSC_ELSB(chnlParams->level));

            /* Update the mode and edge level for channel n + 1*/
            base->CONTROLS[(chnlParams->chnlNumber * 2) + 1].CnSC = reg;

            /* Set the channel pair values */
            base->CONTROLS[chnlParams->chnlNumber * 2].CnV = cnvFirstEdge;
            base->CONTROLS[(chnlParams->chnlNumber * 2) + 1].CnV = cnvFirstEdge + cnv;

            /* Set the combine bit for the channel pair */
            base->COMBINE |=
                (1U << (FTM_COMBINE_COMBINE0_SHIFT + (FTM_COMBINE_COMBINE1_SHIFT * chnlParams->chnlNumber)));
        }

#if defined(FSL_FEATURE_FTM_HAS_ENABLE_PWM_OUTPUT) && (FSL_FEATURE_FTM_HAS_ENABLE_PWM_OUTPUT)
        /* Set to output mode */
        FTM_SetPwmOutputEnable(base, chnlParams->chnlNumber, true);
#endif

        chnlParams++;
    }

    return kStatus_Success;
}

void PWM_UpdatePwmDutycycle(FTM_Type *base,
                            ftm_chnl_t chnlNumber,
                            ftm_pwm_mode_t currentPwmMode,
                            uint16_t dutyCycleValue)
{
    uint16_t  cnvFirstEdge = 0, mod;

    mod = base->MOD;
	if(dutyCycleValue>=mod){
		dutyCycleValue=mod+1;
	}
    if ((currentPwmMode == kFTM_EdgeAlignedPwm) || (currentPwmMode == kFTM_CenterAlignedPwm))
    {
/*	// eliminate percentage schema
    	cnv = (mod * dutyCycleValue) / 100;
        // For 100% duty cycle
        if (cnv >= mod)
        {
            cnv = mod + 1;
        }
        base->CONTROLS[chnlNumber].CnV = cnv;
*/
    	base->CONTROLS[chnlNumber].CnV = dutyCycleValue;
    }
    else
    {

        // This check is added for combined mode as the channel number should be the pair number
        if (chnlNumber >= (FSL_FEATURE_FTM_CHANNEL_COUNTn(base) / 2))
        {
            return;
        }
/*
        cnv = (mod * dutyCycleValue) / 100;
        cnvFirstEdge = base->CONTROLS[chnlNumber * 2].CnV;
        // For 100% duty cycle
        if (cnv >= mod)
        {
            cnv = mod + 1;
        }
        base->CONTROLS[(chnlNumber * 2) + 1].CnV = cnvFirstEdge + cnv;
*/
        cnvFirstEdge = base->CONTROLS[chnlNumber * 2].CnV;
        base->CONTROLS[(chnlNumber * 2) + 1].CnV = cnvFirstEdge + dutyCycleValue;
    }
}




