#include "F28x_Project.h"
#include "adc.h"
#include "epwm.h"

void Init_Adc_Setup(void)
{
    // 0~3V입력 전압 가능

    //HVI 설정
    // AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    // AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    // AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    ADC_enableConverter(ADCA_BASE);

    //LVV 설정
    // AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    // AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);
    // AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    ADC_enableConverter(ADCB_BASE);

    //LVI 설정
    // AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    // AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
    // AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    ADC_enableConverter(ADCC_BASE);

    DELAY_US(1000);

    // AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    // AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    // AdcaRegs.ADCSOC0CTL.bit.ACQPS = 20;
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 20);

    // AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    // AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    // AdcaRegs.ADCSOC0CTL.bit.ACQPS = 20;
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 20);

    // AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    // AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    // AdcaRegs.ADCSOC0CTL.bit.ACQPS = 20;
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN0, 20);

}



