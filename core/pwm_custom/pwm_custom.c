#include "F28x_Project.h"
#include "driverlib.h"
#include "pwm_custom.h"
#include "device.h"
#include "../service/PI_control.h"

volatile uint16_t TBPRD_BASE = 1000U;
volatile uint16_t epwm1_isr_count = 0U;
volatile float frequency_change = 1.0f;
volatile bool is_PI_Control = false;
volatile uint8_t pattern_mode = LLC_PATTERN_NONE;
volatile uint8_t current_pattern = LLC_PATTERN_NONE;

static const uint32_t g_epwmBase[LLC_PWM_COUNT] =
{
    EPWM1_BASE,
    EPWM2_BASE,
    EPWM3_BASE
};

#define CMP_HALF_MINUS      { false, -LLC_DEAD_COUNT }
#define CMP_HALF_PLUS       { false,  LLC_DEAD_COUNT }
#define CMP_FULL            { true,   0 }

static const LLC_Pattern g_pattern1A =
{
 // HIGH 시작 dead band 줄거면  cmp에 -(minus)줘야함
    .pwm =
    {
        {
            .aqA = LLC_AQ_HIGH, //epwm1A HIGH start
            .aqB = LLC_AQ_LOW, // epwm1B LOW start
            .cmpA = CMP_HALF_MINUS, // dead band
            .cmpB = CMP_FULL // full duty
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_FULL
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_FULL
        }
    }
};

static const LLC_Pattern g_pattern1B =
{
    .pwm =
    {
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_FULL,
            .cmpB = CMP_HALF_MINUS
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_FULL,
            .cmpB = CMP_HALF_PLUS
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_FULL,
            .cmpB = CMP_HALF_MINUS
        }
    }
};

static const LLC_Pattern g_pattern2 =
{
    .pwm =
    {
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_HALF_PLUS
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_HALF_MINUS
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_HALF_PLUS
        }
    }
};

static const LLC_Pattern g_pattern3A =
{
    .pwm =
    {
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_HALF_PLUS
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_HALF_MINUS
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_HALF_MINUS
        }
    }
};

static const LLC_Pattern g_pattern3B =
{
    .pwm =
    {
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_HALF_PLUS
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_HALF_PLUS
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_HALF_MINUS
        }
    }
};

static const LLC_Pattern g_pattern3C =
{
    .pwm =
    {
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_FULL,
            .cmpB = CMP_HALF_PLUS
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_FULL,
            .cmpB = CMP_FULL
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_FULL,
            .cmpB = CMP_HALF_MINUS
        }
    }
};

static const LLC_Pattern g_pattern3D =
{
    .pwm =
    {
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_FULL
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_FULL,
            .cmpB = CMP_FULL
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_FULL
        }
    }
};

static const LLC_Pattern g_pattern4 =
{
    .pwm =
    {
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_LOW,
            .cmpA = CMP_HALF_MINUS,
            .cmpB = CMP_HALF_PLUS
        },
        {
            .aqA = LLC_AQ_HIGH,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_FULL,
            .cmpB = CMP_FULL
        },
        {
            .aqA = LLC_AQ_LOW,
            .aqB = LLC_AQ_HIGH,
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_HALF_MINUS
        }
    }
};

static const LLC_Pattern * const g_patternTable[] =
{
    NULL,
    &g_pattern1A,
    &g_pattern1B,
    &g_pattern2,
    &g_pattern3A,
    &g_pattern3B,
    &g_pattern3C,
    &g_pattern3D,
    &g_pattern4
};

static void LLC_SetAQ_A(uint32_t base, LLC_AQType type);
static void LLC_SetAQ_B(uint32_t base, LLC_AQType type);
static uint16_t LLC_CalcCompare(const LLC_CompareConfig *config, uint16_t period);
static void LLC_ApplyAQ(const LLC_Pattern *pattern);

volatile float target_fsw = 0;
void pwm_init(void)
{
    TBCLKSYNC_disable();

    Interrupt_enableInCPU(M_INT3);
    Interrupt_register(INT_EPWM1, &epwm1_isr);
    Interrupt_enable(INT_EPWM1);

    InitEPWMGpioPin();

    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
    EDIS;


    //PWM f = TBCLK / (2 * TBPRD)
    //50kHz = 100MHz / (2 * 1000)
    EPWM_SignalParams pwm_base_signal = {
        .freqInHz = 50000,
        .dutyValA = 0.5f,
        .dutyValB = 0.5f,
        .sysClkInHz = DEVICE_SYSCLK_FREQ,
        .invertSignalB = false,
        .tbCtrMode = EPWM_COUNTER_MODE_UP_DOWN,
        .tbClkDiv = EPWM_CLOCK_DIVIDER_1,
        .tbHSClkDiv = EPWM_HSCLOCK_DIVIDER_1
    };

    EPWM_Custom pwm_master =
    {
        .epwm_signal_params = pwm_base_signal,
        .master_bit = true,
        .phaseShift = 0U
    };

    EPWM_Custom pwm_slave =
    {
        .epwm_signal_params = pwm_base_signal,
        .master_bit = false,
        .phaseShift = 2U
    };

    CreateEPwm(EPWM1_BASE, pwm_master); // MASTER
    CreateEPwm(EPWM2_BASE, pwm_slave);
    CreateEPwm(EPWM3_BASE, pwm_slave);

   // ePWM 1의 TBPRD 기준
    EPWM_setupEPWMLinks(EPWM2_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);  // EPwm2Regs.EPWMXLINK.bit.TBPRDLINK
    EPWM_setupEPWMLinks(EPWM3_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);

    PWM_PRI_EN_DI(PWM_ON);

    TBCLKSYNC_enable();
}

void CreateEPwm(uint32_t epwm_base, EPWM_Custom epwm_signal)
{

    // 기본 파형 생성
    EPWM_configureSignal(epwm_base, &epwm_signal.epwm_signal_params);

    EPWM_setPeriodLoadMode(epwm_base, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_selectPeriodLoadEvent(epwm_base, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    if(epwm_signal.master_bit == true)
    {
        EPWM_disablePhaseShiftLoad(epwm_base);
        EPWM_setSyncOutPulseMode(epwm_base, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
    }
    else
    {
        EPWM_enablePhaseShiftLoad(epwm_base);
        EPWM_setPhaseShift(epwm_base, epwm_signal.phaseShift);
        EPWM_setSyncOutPulseMode(epwm_base, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
        EPWM_setCountModeAfterSync(epwm_base, EPWM_COUNT_MODE_UP_AFTER_SYNC);
    }


    // 데드밴드 설정 DBCTL[OUT_MODE]
    EPWM_setDeadBandDelayMode(epwm_base, EPWM_DB_RED, false);   // DBCTL.OUT_MODE RED 설정
    EPWM_setDeadBandDelayMode(epwm_base, EPWM_DB_FED, false);   // DBCTL.OUT_MODE FED 설정


    // 인터럽트 발생 초기화
    EPWM_setInterruptSource(epwm_base, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(epwm_base, 1);
    EPWM_enableInterrupt(epwm_base);


    // 트립 존 초기화
    EPWM_setTripZoneAction(epwm_base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(epwm_base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
}

void InitEPWMGpioPin(void)
{
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPinConfig(GPIO_5_EPWM3B);
}

void PWM_PRI_EN_DI(int enable)
{
    if(enable == PWM_ON)
    {
        ePWM_TZ123_Reset();
    }
    else
    {
        ePWM_Force123_Trip();
    }
}

void ePWM_TZ123_Reset(void)
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;        // Clear OST
    EPwm1Regs.TZCLR.bit.INT = 1;        // Clear INT
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;

    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.INT = 1;

    EDIS;
}

void ePWM_Force123_Trip(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;        // OST 발생 -> PWM 강제 LOW
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;

    EDIS;
}

const LLC_Pattern *LLC_GetPattern(uint8_t pattern)
{
    if(pattern < LLC_PATTERN_MIN || pattern > LLC_PATTERN_MAX)
    {
        return NULL;
    }

    return g_patternTable[pattern];
}

static void LLC_SetAQ_A(uint32_t base, LLC_AQType type)
{
    if(type == LLC_AQ_HIGH)
    {
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    }
    else
    {
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    }
}

static void LLC_SetAQ_B(uint32_t base, LLC_AQType type)
{
    if(type == LLC_AQ_HIGH)
    {
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    }
    else
    {
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
        EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    }
}

static uint16_t LLC_CalcCompare(const LLC_CompareConfig *config, uint16_t period)
{
    int32_t value;

    if(config->useFullPeriod == true) // 풀 듀티
    {
        return period;
    }

    value = ((int32_t)period >> 1) + (int32_t)config->offset; // (period / 2) + deadband

    if(value < 0)
    {
        value = 0;
    }

    if(value > period)
    {
        value = period;
    }

    return (uint16_t)value;
}

static void LLC_ApplyAQ(const LLC_Pattern *pattern)
{
    uint16_t i;

    for(i = 0U; i < LLC_PWM_COUNT; i++)
    {
        LLC_SetAQ_A(g_epwmBase[i], pattern->pwm[i].aqA);
        LLC_SetAQ_B(g_epwmBase[i], pattern->pwm[i].aqB);
    }
}

void LLC_UpdateCompare(const LLC_Pattern *pattern, uint16_t period)
{
    uint16_t i;
    uint16_t cmpA;
    uint16_t cmpB;

    // ePWM 1,2,3
    for(i = 0U; i < LLC_PWM_COUNT; i++)
    {
        cmpA = LLC_CalcCompare(&pattern->pwm[i].cmpA, period); // period : EPWM1 TBPRD
        cmpB = LLC_CalcCompare(&pattern->pwm[i].cmpB, period);

        EPWM_setCounterCompareValue(g_epwmBase[i], EPWM_COUNTER_COMPARE_A, cmpA);
        EPWM_setCounterCompareValue(g_epwmBase[i], EPWM_COUNTER_COMPARE_B, cmpB);
    }
}

void LLC_ApplyPattern(const LLC_Pattern *pattern, bool updateAQ)
{
    uint16_t currentPeriod;

    if(pattern == NULL)
    {
        return;
    }

    if(updateAQ == true)
    {
        LLC_ApplyAQ(pattern);
    }

    currentPeriod = EPWM_getTimeBasePeriod(EPWM1_BASE);
    LLC_UpdateCompare(pattern, currentPeriod);
}

__interrupt void epwm1_isr(void)
{
    const LLC_Pattern *pattern = LLC_GetPattern(pattern_mode);

    uint16_t current_prd = EPWM_getTimeBasePeriod(EPWM1_BASE);
    uint16_t new_prd = current_prd;

    bool patternChanged = false;
    bool periodChanged = false;

    epwm1_isr_count++;

    if(pattern != NULL)
    {
        if(current_pattern != pattern_mode)
        {
            current_pattern = pattern_mode;
            patternChanged = true;
        }
    }

    if(is_PI_Control == true)
    {
        float temp = 1.0f / 50000.0f;
        target_fsw = PI_control_PFM(temp);

        new_prd = (uint16_t)((float)TBPRD_BASE * (50000.0f / target_fsw));

        if(new_prd < 250U)
        {
            new_prd = 250U;
        }
        else if(new_prd > 1667U)
        {
            new_prd = 1667U;
        }

        if(new_prd != current_prd)
        {
            periodChanged = true;
            EPWM_setTimeBasePeriod(EPWM1_BASE, new_prd);
        }
    }

    if(pattern != NULL)
    {
        if(patternChanged == true)
        {
            LLC_ApplyAQ(pattern);
        }

        if(patternChanged == true || periodChanged == true)
        {
            LLC_UpdateCompare(pattern, new_prd);
        }
    }

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    EPWM_clearEventTriggerInterruptFlag(EPWM2_BASE);
    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}
