/*
 * TASK.c
 *
 *  Created on: 2026. 9. 15.
 *      Author: h04
 */

#include "TASK.h"
#include "LPF.h"
#include "adc.h"
#include "epwm.h"
#include "../pwm_custom/pwm_custom.h"
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "PI_control.h"
#include "LPF.h"
#include "MAF.h"

volatile uint16_t test_count = 0;

volatile uint16_t TBPRD_BASE = 500U;
volatile uint16_t epwm1_isr_count = 0U;
volatile float frequency_change = 100000;
volatile float current_frequency = 0.0f;
volatile bool is_PI_Control_v = false;
volatile bool is_PI_Control_i = false;
volatile uint8_t pattern_mode = LLC_PATTERN_1A;
volatile uint8_t current_pattern = LLC_PATTERN_1A;
volatile uint8_t start_bit = 0;
volatile static uint8_t prev_start_bit = 0;
volatile float target_fsw_v = 0;
volatile float target_fsw_i = 0;

volatile bool patternChanged = false;
volatile bool periodChanged = false;

__interrupt void epwm1_isr(void);
__interrupt void epwm4_isr(void);


float LVV = 0.0f;
float LVI = 0.0f;

float LVV_scale = 0.0f;
float LVI_scale = 0.0f;

float MAF_in_LVV = 0.0f;
float MAF_in_LVI = 0.0f;
float MAF_in_lol = 0.0f;

float FOV = 0.0f;
float FOI = 0.0f;


__interrupt void epwm1_isr(void)
{
    epwm1_isr_count++;

    LVV = (float)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0); // ADC Read  ADCbResultsRegs.ADCRESULT0
    LVI = (float)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0); // ADC Read  ADCbResultsRegs.ADCRESULT0

    LVV_scale = 0.1016*LVV + 0.1956;
    LVI_scale = 0.0303*LVI - 68.006;

    LPF1_Calcul(LVV_scale,&stLPF_LVVout);
    LPF1_Calcul(LVI_scale,&stLPF_LVIout);

    MAF_in_LVV = stLPF_LVVout.fFilterOut;
    MAF_in_LVI = stLPF_LVIout.fFilterOut;

    FOV = apply_moving_avg(&stMAF_LVV_out,MAF_in_LVV);
    FOI = apply_moving_avg(&stMAF_LVI_out,MAF_in_LVI);

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

int adc = 0;
__interrupt void epwm4_isr(void)
{
   adc++;
   if (adc > 4096){
       adc = 0;
   }

   //DAC 0 ~ 4096 / -10V ~ +10V
   //DAC +- 5V 안 넘도록 스케일링 해야함.

//   DAC_Out(0, 2048 + (LVV_scale*1024*0.005));
//   DAC_Out(1, 2048 + (MAF_in_LVV*1024*0.005));
//   DAC_Out(2, 2048 + (FOV*1024*0.005));

//   DAC_Out(0, 2048 + (LVI_scale*1024*0.033));
//   DAC_Out(1, 2048 + (MAF_in_LVI*1024*0.033));
//   DAC_Out(2, 2048 + (FOI*1024*0.033));

   if(start_bit == 0)
   {
       // 1A, 1B, 2A, 2B, 3A, 3B 전부 LOW
       ePWM_Force123_Trip();

       prev_start_bit = 0; // prev_start_bit 일단 설정.

       EPWM_clearEventTriggerInterruptFlag(EPWM4_BASE);
       Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

       return;
   }


   if(prev_start_bit == 0)
   {
       test_count++;

       ePWM_TZ123_Reset();        // Trip 해제 → PWM 출력 허용
       patternChanged = true;        // 현재 설정되어있는 pattern을 다시 확실하게 적용
       prev_start_bit = 1;
   }


   // 현재 패턴이 유효한 패턴인지 확인(패턴 변경 방지 constant)
   const LLC_Pattern *pattern = LLC_GetPattern(pattern_mode);

   uint16_t current_prd = EPWM_getTimeBasePeriod(EPWM1_BASE);
   uint16_t new_prd = current_prd;


   // 패턴 변화 확인 및 패턴 변경 비트 활성화.
   if(pattern != NULL)
   {
       if(current_pattern != pattern_mode)
       {
           current_pattern = pattern_mode;
           patternChanged = true;
       }
   }

   // PI 제어 비트
   if(is_PI_Control_v == true)
   {
       is_PI_Control_i = false;

       target_fsw_v = PI_V_control_PFM(FOV);

       new_prd = (uint16_t)((float)TBPRD_BASE * (100000.0f / target_fsw_v));

       if(new_prd < MIN_PRD)
       {
           new_prd = MIN_PRD;
       }
       else if(new_prd > MAX_PRD)
       {
           new_prd = MAX_PRD;
       }

       if(new_prd != current_prd)
       {
           periodChanged = true;
           EPWM_setTimeBasePeriod(EPWM1_BASE, new_prd);
       }
   }
   else if(is_PI_Control_i == true)
   {
       is_PI_Control_v = false;

       target_fsw_i = PI_I_control_PFM(FOI);

       new_prd = (uint16_t)((float)TBPRD_BASE * (100000.0f / target_fsw_i));

       if(new_prd < MIN_PRD)
       {
           new_prd = MIN_PRD;
       }
       else if(new_prd > MAX_PRD)
       {
           new_prd = MAX_PRD;
       }

       if(new_prd != current_prd)
       {
           periodChanged = true;
           EPWM_setTimeBasePeriod(EPWM1_BASE, new_prd);
       }
   }
   else // 수동 주기 변경 제어 (frequency_change = 수기 입력)
   {
       new_prd = (uint16_t)((float)TBPRD_BASE * (100000.0f / frequency_change));

       if(new_prd < MIN_PRD)
       {
           new_prd = MIN_PRD;
       }
       else if(new_prd > MAX_PRD)
       {
           new_prd = MAX_PRD;
       }

       if(new_prd != current_prd)
       {
           EPWM_setTimeBasePeriod(EPWM1_BASE, new_prd); // epwm1 주기 설정 (epwm2,3은 XLINK연결)
           periodChanged = true; // 주기 변경 비트 활성화
       }
   }

   if(pattern != NULL)
   {
       if(patternChanged == true) // 패턴 변경 시
       {
           // 패턴만 변경
           LLC_ApplyAQ(pattern);
       }

       /*
        * TODO : 현재 patternChanged가 중복으로 설정되어야함.
        *   패턴 변경 될 경우 - 패턴만 변경될 경우 주기의 데드밴드가 Duty로 설정되고 있는 것이 문제점
        *   주기만 변경될 경우 패턴 변경의 로직은 피하고 싶음.
        * */

       if(patternChanged == true || periodChanged == true) // 패턴 또는 주기 둘 다 변경 시.
       {
           // CMPA, B적용
           LLC_UpdateCompare(pattern, new_prd);

           // 코드상 frequency 적용 확인용
           current_frequency = ((float)TBPRD_BASE * (100000.0f / new_prd));

           patternChanged = false;
           periodChanged = false;
       }
   }

   EPWM_clearEventTriggerInterruptFlag(EPWM4_BASE);
   Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}








