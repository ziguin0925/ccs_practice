
#define PWM_ON          1
#define PWM_OFF         2

typedef struct
{
    bool                master_bit;
    uint16_t            phaseShift;
    EPWM_SignalParams   epwm_signal_params;
} EPWM_Custom;

void PWM_PRI_EN_DI(int enable);
void ePWM_Force123_Trip(void);
void ePWM_TZ123_Reset();

void pwm_init (void);
void CreateEPwm(uint16_t, EPWM_Custom);
void InitEPWMGpioPin(void);

__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);


static inline void TBCLKSYNC_disable(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
}

static inline void TBCLKSYNC_enable(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}
