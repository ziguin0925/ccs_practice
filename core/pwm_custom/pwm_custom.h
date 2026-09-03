#ifndef PWM_CUSTOM_H_
#define PWM_CUSTOM_H_

#define PWM_ON          1
#define PWM_OFF         2
#define LLC_PATTERN_MIN         1U
#define LLC_PATTERN_MAX         8U
#define LLC_PWM_COUNT           3U
#define LLC_DEAD_COUNT          15
#define MIN_PRD         250U
#define MAX_PRD         1667U

typedef struct
{
    bool                master_bit;
    uint16_t            phaseShift;
    EPWM_SignalParams   epwm_signal_params;
} EPWM_Custom;

typedef enum
{
    LLC_PATTERN_NONE = 0,
    LLC_PATTERN_1A =1,
    LLC_PATTERN_1B,
    LLC_PATTERN_2,
    LLC_PATTERN_3A,
    LLC_PATTERN_3B,
    LLC_PATTERN_3C,
    LLC_PATTERN_3D,
    LLC_PATTERN_4
} LLC_PatternMode;

typedef enum
{
    LLC_AQ_HIGH = 0,
    LLC_AQ_LOW
} LLC_AQType;

typedef struct
{
    bool useFullPeriod;
    int16_t offset;
} LLC_CompareConfig;

typedef struct
{
    LLC_AQType aqA;
    LLC_AQType aqB;
    LLC_CompareConfig cmpA;
    LLC_CompareConfig cmpB;
} LLC_EPWMPattern;

typedef struct
{
    LLC_EPWMPattern pwm[LLC_PWM_COUNT];
} LLC_Pattern;


extern volatile uint16_t TBPRD_BASE;
extern volatile uint16_t epwm1_isr_count;
extern volatile float frequency_change;
extern volatile bool is_PI_Control;
extern volatile uint8_t pattern_mode;
extern volatile uint8_t current_pattern;


void PWM_PRI_EN_DI(int enable);
void ePWM_Force123_Trip(void);
void ePWM_TZ123_Reset();

void pwm_init (void);
void CreateEPwm(uint32_t epwm_base, EPWM_Custom epwm_signal);
void InitEPWMGpioPin(void);

const LLC_Pattern *LLC_GetPattern(LLC_PatternMode pattern);
void LLC_ApplyPattern(const LLC_Pattern *pattern, bool updateAQ);
void LLC_UpdateCompare(const LLC_Pattern *pattern, uint16_t period);



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

#endif
