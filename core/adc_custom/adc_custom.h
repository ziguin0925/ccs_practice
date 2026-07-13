void ADC_Init(void)
{
    //
    // ADC Clock Enable
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);

    //
    // ADC 설정
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);

    ADC_setMode(ADCA_BASE,
                ADC_RESOLUTION_12BIT,
                ADC_MODE_SINGLE_ENDED);

    ADC_enableConverter(ADCA_BASE);

    DEVICE_DELAY_US(1000);

    //
    // SOC0 : ADCINA1
    //
    ADC_setupSOC(ADCA_BASE,
                 ADC_SOC_NUMBER0,
                 ADC_TRIGGER_EPWM1_SOCA,
                 ADC_CH_ADCIN1,
                 15);

    //
    // ADC Interrupt
    //
    ADC_setInterruptSource(ADCA_BASE,
                           ADC_INT_NUMBER1,
                           ADC_SOC_NUMBER0);

    ADC_enableInterrupt(ADCA_BASE,
                        ADC_INT_NUMBER1);

    ADC_clearInterruptStatus(ADCA_BASE,
                             ADC_INT_NUMBER1);
}