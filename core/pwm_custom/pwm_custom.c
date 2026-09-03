#include "F28x_Project.h"
#include "driverlib.h"
#include "pwm_custom.h"
#include "device.h"
#include "../service/PI_control.h"

volatile uint16_t test_count = 0;

volatile uint16_t TBPRD_BASE = 1000U;
volatile uint16_t epwm1_isr_count = 0U;
volatile float frequency_change = 50000;
volatile float current_frequency = 0.0f;
volatile bool is_PI_Control = false;
volatile uint8_t pattern_mode = LLC_PATTERN_1A;
volatile uint8_t current_pattern = LLC_PATTERN_1A;
volatile uint8_t only_one_time_on = 0;
volatile uint8_t start_bit = 0;
volatile static uint8_t prev_start_bit = 0;

volatile bool patternChanged = false;
volatile bool periodChanged = false;


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
            .aqA = LLC_AQ_LOW, //epwm2A HIGH start
            .aqB = LLC_AQ_HIGH, // epwm2B LOW start
            .cmpA = CMP_HALF_PLUS,
            .cmpB = CMP_FULL
        },
        {
            .aqA = LLC_AQ_HIGH, //epwm3A HIGH start
            .aqB = LLC_AQ_LOW, // epwm3B LOW start
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
    Interrupt_register(INT_EPWM1, &epwm1_isr); // 인터럽트 등록
    Interrupt_enable(INT_EPWM1);

    InitEPWMGpioPin();

    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
    EDIS;

    // default ePWM struct initialization
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



   // ePWM 1의 TBPRD 기준 (EPWMXLINK기능)
    EPWM_setupEPWMLinks(EPWM2_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);  // EPwm2Regs.EPWMXLINK.bit.TBPRDLINK
    EPWM_setupEPWMLinks(EPWM3_BASE, EPWM_LINK_WITH_EPWM_1, EPWM_LINK_TBPRD);

    PWM_PRI_EN_DI(PWM_OFF); // start_bit == 1이어야 풀리도록

    TBCLKSYNC_enable();
}

void CreateEPwm(uint32_t epwm_base, EPWM_Custom epwm_signal)
{

    // Implement default ePWM struct
    EPWM_configureSignal(epwm_base, &epwm_signal.epwm_signal_params);

    // Shadow Resistor
    EPWM_setPeriodLoadMode(epwm_base, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_selectPeriodLoadEvent(epwm_base, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);

    if(epwm_signal.master_bit == true)
    {

        EPWM_disablePhaseShiftLoad(epwm_base); // master일 때 Phase Shift는 비활성이 기본이라함.
        EPWM_setSyncOutPulseMode(epwm_base, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO); // master는 카운터가 0 일때 SYNC 신호를 보냄
    }
    else
    {
        EPWM_enablePhaseShiftLoad(epwm_base); // slave는 Phase Shift 활성화.
        EPWM_setPhaseShift(epwm_base, epwm_signal.phaseShift);
        EPWM_setSyncOutPulseMode(epwm_base, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); // SYNC신호 받는 동시에 SYNC 신호 내보내기(SYNC flow Diagram 확인)
        EPWM_setCountModeAfterSync(epwm_base, EPWM_COUNT_MODE_UP_AFTER_SYNC); // SYNC 신호 받으면 UPCOUNT 하도록
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

const LLC_Pattern *LLC_GetPattern(LLC_PatternMode pattern)
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

    // Clamp valid range 0 ~ period (없어도 상관은 없음)
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

    // 각 ePWMx 에 대한 패턴 형상 적용
    for(i = 0U; i < LLC_PWM_COUNT; i++)
    {
        LLC_SetAQ_A(g_epwmBase[i], pattern->pwm[i].aqA);
        LLC_SetAQ_B(g_epwmBase[i], pattern->pwm[i].aqB);
    }
}

/*
 *  pattern - pattern_mode를 통해 입력받은 유효한 패턴
 *  period - frequency_change를 통해 입력받은 유효한 주기
 * */
void LLC_UpdateCompare(const LLC_Pattern *pattern, uint16_t period)
{
    uint16_t i;
    uint16_t cmpA;
    uint16_t cmpB;

    // i =  ePWM 1,2,3
    for(i = 0U; i < LLC_PWM_COUNT; i++)
    {
        // 패턴에 맞는 해당 ePWMx에 Full, 0, half 듀티 적용(데드밴드 적용)
        cmpA = LLC_CalcCompare(&pattern->pwm[i].cmpA, period);
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

    if(start_bit == 0)
    {
        // 1A, 1B, 2A, 2B, 3A, 3B 전부 LOW
        ePWM_Force123_Trip();

        prev_start_bit = 0; // prev_start_bit 일단 설정.

        EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
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

    epwm1_isr_count++;

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
    if(is_PI_Control == true)
    {
        float temp = 1.0f / 50000.0f;
        target_fsw = PI_control_PFM(temp);

        new_prd = (uint16_t)((float)TBPRD_BASE * (50000.0f / target_fsw));

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
        new_prd = (uint16_t)((float)TBPRD_BASE * (50000.0f / frequency_change));

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
            current_frequency = ((float)TBPRD_BASE * (50000.0f / new_prd));

            patternChanged = false;
            periodChanged = false;
        }
    }

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}
