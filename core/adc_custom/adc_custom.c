#include "F28x_Project.h"
#include "device.h"

void initADCA(void)
{

    //
    // ADC Trigger (SOCA)
    //
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    // ADC Clock
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);

    // ADC 클럭 분주 설정
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);

    // ADC 전원 켜기 ADCPWDNZ = 1;
    ADC_enableConverter(ADCA_BASE);

    DEVICE_DELAY_US(1000);

    //
    // SOC0 -> ADCINA1 (A1)
    // EPWM1 SOCA
    // ADCINA1(A1)
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 20);

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);

    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}
