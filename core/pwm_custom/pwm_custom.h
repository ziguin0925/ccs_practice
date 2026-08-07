
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

void EPWM_high_low_AQ(uint16_t base, bool high_bit);
void startLlcPattern1A(void);
void startLlcPattern1B(void);
void startLlcPattern2(void);
void startLlcPattern3A(void);
void startLlcPattern3B(void);
void startLlcPattern3C(void);
void startLlcPattern3D(void);
void startLlcPattern4(void);

__interrupt void epwm1_isr(void);



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
