#include "F28x_Project.h"
#include "driverlib.h"
#include "pwm_custom.h"
#include "device.h"
#include "../service/PI_control.h"

volatile uint16_t TBPRD_BASE  = 1000;

volatile uint16_t epwm1_isr_count = 0;
volatile float frequency_change = 1.0;

volatile float  temp_epwm1 = 1;
volatile float  temp_epwm2 = 1;
volatile float  temp_epwm3 = 1;

volatile bool is_PI_Control = false;

volatile uint8_t pattern_mode = 0;




void pwm_init(void)
{
    TBCLKSYNC_disable();

    Interrupt_enableInCPU(M_INT3); // M_INT3 epwm 다킴
    // Interrupt registration is handled by the project’s PIE vector table setup in this build.

    Interrupt_register(INT_EPWM1, &epwm1_isr);
    Interrupt_enable(INT_EPWM1);

    // Use ePWM pin enable
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
//                .dutyValB = 0.5f,
                .sysClkInHz = DEVICE_SYSCLK_FREQ,
                .invertSignalB = false,
                .tbCtrMode = EPWM_COUNTER_MODE_UP_DOWN,
                .tbClkDiv = EPWM_CLOCK_DIVIDER_1,
                .tbHSClkDiv = EPWM_HSCLOCK_DIVIDER_1
        };

    EPWM_Custom pwm_master ={
           .epwm_signal_params = pwm_base_signal,
           .master_bit = true,
           .phaseShift = 0
    };


    EPWM_Custom pwm_slave ={
           .epwm_signal_params = pwm_base_signal,
           .master_bit = false,
           .phaseShift = 1000
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



// AQ(Action Qualifier)로 인한 단순 반전동작 사용 x -> DeadTime 서브 모듈 사용으로 상보 동작
void CreateEPwm(uint16_t epwm_base, EPWM_Custom epwm_signal)
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
    else{
        EPWM_enablePhaseShiftLoad(epwm_base);
        EPWM_setPhaseShift(epwm_base, epwm_signal.phaseShift);
        EPWM_setSyncOutPulseMode(epwm_base, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
        EPWM_setCountModeAfterSync(epwm_base, EPWM_COUNT_MODE_UP_AFTER_SYNC);
    }


    // 인터럽트 발생 초기화
    EPWM_setInterruptSource(epwm_base, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(epwm_base, 1);
    EPWM_enableInterrupt(epwm_base);


    // 트립 존 초기화
    EPWM_setTripZoneAction(epwm_base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(epwm_base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);

    // DBCTL[IN_MODE] 설정
    EPWM_setRisingEdgeDeadBandDelayInput(epwm_base, EPWM_DB_INPUT_EPWMA); // EPWM1A 신호를 RED 입력으로 사용
    EPWM_setFallingEdgeDeadBandDelayInput(epwm_base, EPWM_DB_INPUT_EPWMA); // RED 출력 후 하강 에지 입력으로 사용하여 A/B 보수 신호 생성

    // 데드밴드 설정 DBCTL[OUT_MODE]
    EPWM_setDeadBandDelayMode(epwm_base, EPWM_DB_RED, true);   // DBCTL.OUT_MODE RED 설정
    EPWM_setDeadBandDelayMode(epwm_base, EPWM_DB_FED, true);   // DBCTL.OUT_MODE FED 설정


    // 10 (AHC) ePWMxA는 정상, ePWMxB는 A의 보수
    EPWM_setDeadBandDelayPolarity(epwm_base, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(epwm_base, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);


    // RED/FED 값을 설정하여 deadband 시간을 결정
    EPWM_setRisingEdgeDelayCount(epwm_base, 20);   // RED = 20 TBCLK
    EPWM_setFallingEdgeDelayCount(epwm_base, 20);  // FED = 20 TBCLK

}


void InitEPWMGpioPin(void){

    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

    //    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    //    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);

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
    else if(enable == PWM_OFF)
    {
        ePWM_Force123_Trip();
    }
}


void ePWM_TZ123_Reset()
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


// typedef __interrupt void (*PINT)(void);
__interrupt void epwm1_isr(void)
{

    if (is_PI_Control == true){
        frequency_change = PI_control_PFM();
    }

    if(frequency_change != temp_epwm1)
    {
        epwm1_isr_count++;

        // 패턴 1-A
        TBCLKSYNC_disable();

        switch(pattern_mode){
        case 1:
            startLlcPattern1A();
            break;
        case 2:
            // LLC 패턴 1B 넣기
            break;
        case 3:
            startLlcPattern2();
            break;
        }
        TBCLKSYNC_enable();

        uint16_t new_prd = (uint16_t)(TBPRD_BASE / frequency_change);

        // ePWM 1
        EPWM_setTimeBasePeriod(EPWM1_BASE, new_prd); // TBPRD
        EPWM_setCounterCompareValue(
            EPWM1_BASE,
            EPWM_COUNTER_COMPARE_A,
            new_prd / 2
        );
        temp_epwm1 = frequency_change;


        // ePWM 2
        EPWM_setCounterCompareValue(
                EPWM2_BASE,
                EPWM_COUNTER_COMPARE_A,
                new_prd / 2
        );
    // AQ로 HIGH/LOW 액션을 반대로 주어 상보동작 하도록
//        EPWM_setPhaseShift(EPWM2_BASE, new_prd);

        // ePWM 3
        EPWM_setCounterCompareValue(
            EPWM3_BASE,
            EPWM_COUNTER_COMPARE_A,
            new_prd / 2
        );
//        EPWM_setPhaseShift(EPWM3_BASE, new_prd);
//        EPWM_forceSyncPulse(EPWM1_BASE); // SWFSYNC

    }

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    EPWM_clearEventTriggerInterruptFlag(EPWM2_BASE);
    EPWM_clearEventTriggerInterruptFlag(EPWM3_BASE);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


void EPWM_high_low_AQ(uint16_t base, bool high_bit)
{
    if(high_bit == true)
    {
        // 올라갈 때 COMP만나면 HIGH
        // 내려갈 때 COMP 만나면 LOW

        //
        // Clear PWMxA on Zero
        //
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

        //
        // Set PWMxA on event A, up count
        //
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

        //
        // Clear PWMxA on event A, down count
        //
        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    }
    else{
        // 올라갈 때 COMP만나면 LOW
        // 내려갈 때 COMP 만나면 HIGH

        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);


        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

        EPWM_setActionQualifierAction(base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    }
}


void startLlcPattern1A(){
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, false);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMB);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, false);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMB);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, false);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMB);

    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);

    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);

    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);

    // A 채널: Zero에서 HIGH, CMPA에서 LOW (정방향)
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    // B 채널: Continuous Software Force를 사용하여 강제로 LOW 고정 [3]
    EPWM_setActionQualifierContSWForceAction(EPWM1_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);

    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierContSWForceAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_LOW);


    // A 채널: Zero에서 LOW, CMPA에서 HIGH (1, 3과 반대 동작) [4, 5]
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    // B 채널: Continuous Software Force를 사용하여 강제로 HIGH 고정 [3]
    EPWM_setActionQualifierContSWForceAction(EPWM2_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_SW_OUTPUT_HIGH);
    EPWM_setPhaseShift(EPWM1_BASE, 0);
    EPWM_setPhaseShift(EPWM2_BASE, 0);
    EPWM_setPhaseShift(EPWM3_BASE, 0);
}


void startLlcPattern2(){
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);


    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);

    // A 채널: Zero에서 HIGH, CMPA에서 LOW (정방향)
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setPhaseShift(EPWM1_BASE, 0);
    EPWM_setPhaseShift(EPWM2_BASE, 0);
    EPWM_setPhaseShift(EPWM3_BASE, 0);
}
