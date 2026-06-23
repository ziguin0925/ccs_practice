#include "F28x_Project.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "sysctl.h"
#include "driverlib.h"
#include "device.h"

#define PWM_ON          1
#define PWM_OFF         2


int fortesta = 0;
float for_test_B = 0.02;
int Loopcnt;
volatile uint16_t TBPRD_BASE  = 1000;

volatile uint16_t epwm1_isr_count = 0;
volatile uint16_t epwm2_isr_count = 0;
volatile float frequency_change = 1.0;
volatile uint16_t temp_epwm2 = 1;
volatile uint16_t TBPRD_epwm2 = 0;
volatile uint16_t CMPA_epwm2 = 0;
volatile uint16_t TBPHS_epwm2 = 0;

volatile uint16_t temp_epwm1 = 1;
volatile uint16_t TBPRD_epwm1 = 0;
volatile uint16_t CMPA_epwm1 = 0;

void ePWM_Force12_Trip(void);
void ePWM_Force34_Trip(void);
void PWM_PRI_EN_DI(int enable);
void PWM_SEC_EN_DI(int enable);
void ePWM_TZ12_Reset();
void ePWM_TZ34_Reset();

__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
void pwm_init (void);
void Init_EPwm_1(void);
void Init_EPwm_2(void);
void InitEPwm1Gpio(void);
void InitEPwm2Gpio(void);




int main(void)
{
    InitSysCtrl(); // �겢�윮 �꽕�젙 諛� Watchdog 鍮꾪솢�꽦�솕
    InitGpio();

    /*  GTAE_EN_33      */
//    GPIO_SetupPinMux(31, 0, 0);                             //pin GPIO31 - general in/output        //update for DSP ver.2 (�츰 ���� 1�̴ϱ� gpio8�� ����ϳ�?)
//    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PULLUP);     //GPIO31 - OUTPUT, PULLUP active        //update for DSP ver.2

    /*  FAULT_CLR   */
    GPIO_SetupPinMux(14, 0, 0);                             //pin GPIO14 - general in/output        �ֳ״� PUD Ǯ�� �� ���ֳ�?
    GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_PULLUP);     //GPIO14 - OUTPUT, PULLUP active

    DINT;
    InitPieCtrl();
    IER = 0x0000;               // Core Interrupts Enable Register 占십깍옙화
    IFR = 0x0000;               // Core Interrupt Flag Register 占십깍옙화
    InitPieVectTable();


    pwm_init();

    // init ePWM
    InitEPwm1Gpio();
    InitEPwm2Gpio();

    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    Init_EPwm_1();
    Init_EPwm_2();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;       //All ePWM TBCLK Start
    EDIS;
    // end init ePWM
    easyDSP_SCI_Init();

    EINT;
    ERTM;

    PWM_PRI_EN_DI(PWM_ON);
    PWM_SEC_EN_DI(PWM_OFF);

    while(1)
    {
        Loopcnt++;
    };
}

void InitEPwm1Gpio(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}
void InitEPwm2Gpio(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}

void Init_EPwm_1()      //PWM f = TBCLK / (2 * TBPRD) = 100MHz / (2 * 1000) = 50kHz
{
    EPWM_SignalParams pwmSignal = {
        .freqInHz = 50000,
        .dutyValA = 0.5f,
        .sysClkInHz = DEVICE_SYSCLK_FREQ,
        .invertSignalB = true,
        .tbCtrMode = EPWM_COUNTER_MODE_UP_DOWN,
        .tbClkDiv = EPWM_CLOCK_DIVIDER_1,
        .tbHSClkDiv = EPWM_HSCLOCK_DIVIDER_1
    };

    // 기본 파형 생성
    EPWM_configureSignal(EPWM1_BASE, &pwmSignal);

    // 인터럽트 발생 초기화
    EPWM_setInterruptSource(
        EPWM1_BASE,
        EPWM_INT_TBCTR_ZERO
    );

    EPWM_setInterruptEventCount(
        EPWM1_BASE,
        1
    );

    EPWM_enableInterrupt(
        EPWM1_BASE
    );

    // 트립 존 초기화
    EPWM_setTripZoneAction(
        EPWM1_BASE,
        EPWM_TZ_ACTION_EVENT_TZA,
        EPWM_TZ_ACTION_LOW
    );

    EPWM_setTripZoneAction(
        EPWM1_BASE,
        EPWM_TZ_ACTION_EVENT_TZB,
        EPWM_TZ_ACTION_LOW
    );
}


void Init_EPwm_2()      //PWM f = TBCLK / (2 * TBPRD) = 100MHz / (2 * 1000) = 50kHz
{
    EALLOW;
    EPwm2Regs.TZCTL.bit.TZA         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1A to low
    EPwm2Regs.TZCTL.bit.TZB         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1B to low
    EPwm2Regs.TZFRC.bit.OST         = 1;                    // One shot trip active -> Clear PWM
    EDIS;

    EPwm2Regs.TBPRD                 =   TBPRD_BASE;      // Set timer period, PRD = 1000
    EPwm2Regs.TBCTR                 =   0x0000;             // Clear counter
    EPwm2Regs.TBPHS.bit.TBPHS       =   1000;             // Clear phase

    //TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) = SYSCLK = 100MHz
    EPwm2Regs.TBCTL.bit.HSPCLKDIV   =   TB_DIV1;            // divider = /1
    EPwm2Regs.TBCTL.bit.CLKDIV      =   TB_DIV1;            // divider = /1
    EPwm2Regs.TBCTL.bit.CTRMODE     =   TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.PHSEN       =   TB_ENABLE;          // Phase Loading 활성화 -> 다른 PWM 모듈과 동기화 시 필요
    EPwm2Regs.TBCTL.bit.PRDLD       =   TB_SHADOW;          // 주기(TBPRD) 값을 shadow 레지스터에서 읽어옴 (카운터 = 0 일 때)
    EPwm2Regs.TBCTL.bit.SYNCOSEL    =   TB_CTR_ZERO;        // 다른 PMW 모듈과 동기화 위해, 카운터 0일 때 Sync out 신호 발생 -> Master

    EPwm2Regs.CMPCTL.bit.SHDWAMODE  =   CC_SHADOW;          // CMPA Shadow mode Enable
    EPwm2Regs.CMPCTL.bit.SHDWBMODE  =   CC_SHADOW;          // CMPB Shadow mode Enable
    EPwm2Regs.CMPCTL.bit.LOADAMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)
    EPwm2Regs.CMPCTL.bit.LOADBMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)

    EPwm2Regs.AQCTLA.bit.CAU        =   AQ_CLEAR;           // Action when counter = CMPA on up-count is Clear
    EPwm2Regs.AQCTLA.bit.CAD        =   AQ_SET;             // Action when counter = CMPA on down-count is Set

    EPwm2Regs.CMPA.bit.CMPA         =   (0.5)*EPwm2Regs.TBPRD;             //Clear Compare Regs

    EPwm2Regs.DBCTL.bit.OUT_MODE    =   DB_FULL_ENABLE;     // EPWMxA/B에 대해 상승(RED)/하강(FED) 에지 모두 데드 밴드 활성화
    EPwm2Regs.DBCTL.bit.POLSEL      =   DB_ACTV_HIC;        // EPWMxB는 EPWMxA의 반전(inverted) 신호로 출력 (상보적인 신호)
    EPwm2Regs.DBCTL.bit.IN_MODE     =   DBA_ALL;            // EPWMxA 신호를 데드 밴드 로직의 입력 소스로 사용
    EPwm2Regs.DBRED.bit.DBRED       =   20;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay
    EPwm2Regs.DBFED.bit.DBFED       =   20;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay

    EPwm2Regs.ETSEL.bit.SOCAEN      = 1;                    // SOCA 이벤트 생성 기능 활성화
    EPwm2Regs.ETSEL.bit.SOCASEL     = 2;                    // 트리거 시점: 카운터가 '주기값(TBPRD)'에 도달했을 때.
    EPwm2Regs.ETPS.bit.SOCAPRD      = 1;                    // Generate pulse on 1st event

    EPwm2Regs.ETSEL.bit.INTSEL      = ET_CTR_ZERO;          // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN       = 1;                    // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD       = ET_1ST;               // Generate INT on 1st event
}


void pwm_init (void)
{
//    IER |= M_INT3;      // epwm1, 4
    Interrupt_enableInCPU(M_INT3);

    Interrupt_register(INT_EPWM1, &epwm1_isr);
    Interrupt_enable(INT_EPWM1);

    EALLOW;
    // Peripheral Interrupt Expansion
    PieVectTable.EPWM2_INT = &epwm2_isr;
    // Table 3-5. PIE Interrupt Vectors
    // PIE Group 3 Vectors - Muxed into CPU INT3
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;      //ePWM2
    EDIS;
}


__interrupt void epwm1_isr(void)
{
    epwm1_isr_count++;

    if(epwm1_isr_count >= 65535)
    {
        epwm1_isr_count = 0;
    }

    if(frequency_change != temp_epwm1)
    {

        EPWM_setTimeBasePeriod(
            EPWM1_BASE,
            TBPRD_BASE / frequency_change
        );

        EPWM_setCounterCompareValue(
            EPWM1_BASE,
            EPWM_COUNTER_COMPARE_A,
            (TBPRD_BASE / frequency_change) / 2
        );

        temp_epwm1 = frequency_change;
    }

    TBPRD_epwm1 = EPWM_getTimeBasePeriod(EPWM1_BASE);

    CMPA_epwm1 = EPWM_getCounterCompareValue(
            EPWM1_BASE,
            EPWM_COUNTER_COMPARE_A
        );

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


__interrupt void epwm2_isr(void)
{
    epwm2_isr_count++;

    if(epwm2_isr_count >= 65535)
        {
            epwm2_isr_count = 0;
        }


    if(frequency_change != temp_epwm2){
        EALLOW;
        EPwm2Regs.TBPRD                 =   TBPRD_BASE / frequency_change;
        EDIS;
        EPwm2Regs.CMPA.bit.CMPA         =   (0.5) * EPwm2Regs.TBPRD;
        EPwm2Regs.TBPHS.bit.TBPHS       =   EPwm2Regs.TBPRD;
        temp_epwm2 = frequency_change;
    }

    TBPRD_epwm2 = EPwm2Regs.TBPRD;
    CMPA_epwm2 = EPwm2Regs.CMPA.bit.CMPA;
    TBPHS_epwm2 = EPwm2Regs.TBPHS.bit.TBPHS;



    EPwm2Regs.ETCLR.bit.INT = 1;                            //ePWM 모듈 내부의 인터럽트 요청 플래그 off, 안 해주면 똑같은 INT 다시 걸림
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;                  //Clear INT Flag

}



void PWM_PRI_EN_DI(int enable)
{
    if(enable == PWM_ON)
    {
        ePWM_TZ12_Reset();
    }
    else if(enable == PWM_OFF)
    {
        ePWM_Force12_Trip();
    }
}

void PWM_SEC_EN_DI(int enable)
{
    if(enable == PWM_ON)
    {
        ePWM_TZ34_Reset();
    }
    else if(enable == PWM_OFF)
    {
        ePWM_Force34_Trip();
    }
}


void ePWM_TZ12_Reset()
{
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;        // Clear OST
    EPwm1Regs.TZCLR.bit.INT = 1;        // Clear INT
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EDIS;
}

void ePWM_TZ34_Reset()
{
    EALLOW;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.INT = 1;
    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm4Regs.TZCLR.bit.INT = 1;
    EDIS;
}

void ePWM_Force12_Trip(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;        // OST 발생 -> PWM 강제 LOW
    EPwm2Regs.TZFRC.bit.OST = 1;
    EDIS;
}

void ePWM_Force34_Trip(void)
{
    EALLOW;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EDIS;
}


