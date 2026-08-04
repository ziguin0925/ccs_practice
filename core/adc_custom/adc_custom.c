#include "F28x_Project.h"
#include "adc.h"
#include "epwm.h"

void Init_Adc_Setup(void)
{
    // 0~3V입력 전압 가능

    // AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    // AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    // AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    ADC_enableConverter(ADCA_BASE);

    DELAY_US(1000);

    // AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    // AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    // AdcaRegs.ADCSOC0CTL.bit.ACQPS = 20;
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 20);

    // AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    // AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    // AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);


    // EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    // EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    // EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
}



