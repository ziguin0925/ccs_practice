/*
 * LPF.c
 *
 *  Created on: 2026. 9. 15.
 *      Author: h04
 */
#include "LPF.h"

t_LpfMembers  stLPF_HVIout;
t_LpfMembers  stLPF_LVVout;
t_LpfMembers  stLPF_LVIout;

void LPF1_Init(const float32_t fFcc,const float32_t fTsample, t_LpfMembers* const pstLPF)
{
    float32_t fTemp;

    fTemp = PI2 * fFcc * fTsample;
    pstLPF->fAlpha = (FLOAT_TWO - fTemp) / (FLOAT_TWO + fTemp);
    pstLPF->fBeta = fTemp /(FLOAT_TWO + fTemp);
}

void LPF_Init(void)
{
    LPF1_Init(CUTOFF_FREQ_10KHZ, TSAMPLE_1U, &stLPF_HVIout); ///0927
    LPF1_Init(CUTOFF_FREQ_10KHZ, TSAMPLE_10U, &stLPF_LVVout);
    LPF1_Init(CUTOFF_FREQ_10KHZ, TSAMPLE_10U, &stLPF_LVIout);
}


/*********************************************
 * \brief low pass filter calculation
 *
 * \param input value, t_LpfMembers
 *
 * \return TRUE(succeed)/FALSE(fail)
 *
 * \remarks
 *
 ***********************************************/
void LPF1_Calcul(const float32_t fInput, t_LpfMembers* const pstLPF)
{
    pstLPF->fFilterOut = (pstLPF->fFilterOut * pstLPF->fAlpha) + ((fInput + pstLPF->fFilterInOld)* pstLPF->fBeta);
    pstLPF->fFilterInOld = fInput;
}

/*********************************************
 * \brief low pass filter calculation
 *
 * \param input value, t_LpfMembers
 *
 * \return TRUE(succeed)/FALSE(fail)
 *
 * \remarks
 *
 ***********************************************/
void LPF1_Reset(t_LpfMembers* const pstLPF)
{
    pstLPF->fFilterInOld = FLOAT_ZERO;
    pstLPF->fFilterOut = FLOAT_ZERO;
}


void LPF_Reset(void)
{
    LPF1_Reset(&stLPF_HVIout);
    LPF1_Reset(&stLPF_LVVout);
    LPF1_Reset(&stLPF_LVIout);
}
