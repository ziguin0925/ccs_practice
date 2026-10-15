/*
 * LPF.h
 *
 *  Created on: 2026. 9. 15.
 *      Author: h04
 */

#ifndef CORE_SERVICE_LPF_H_
#define CORE_SERVICE_LPF_H_
/*************************************************************************
 * INCLUDES
 *************************************************************************/
#include "SystemPara.h"

/*************************************************************************
 * GLOBAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *************************************************************************/
typedef struct t_lpfmembers
{
    float32_t fAlpha;
    float32_t fBeta;
    float32_t fFilterInOld;
    float32_t fFilterOut;
}t_LpfMembers;

extern t_LpfMembers  stLPF_HVIout;
extern t_LpfMembers  stLPF_LVVout;
extern t_LpfMembers  stLPF_LVIout;

/*************************************************************************
 * GLOBAL CONSTANT MACROS
 *************************************************************************/

/*************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 *************************************************************************/
extern void LPF_Init(void);
extern void LPF1_Init(const float32_t fFcc,const float32_t fTsample, t_LpfMembers* const pstLPF);
extern void LPF1_Calcul(const float32_t fInput, t_LpfMembers* const pstLPF);
extern void LPF1_Reset(t_LpfMembers* const pstLPF);
extern void LPF_Reset(void);

#endif /* CORE_SERVICE_LPF_H_ */
