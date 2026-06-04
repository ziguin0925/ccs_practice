#include "F28x_Project.h"
#include "hw_ints.h"
#include "sysctl.h"
#include "interrupt.h"
#include "driverlib.h"



int fortesta = 0;
float for_test_B = 0.02;

void Init_EPwm_1(void);
void InitEPwm1Gpio(void);

int main(void)
{
    InitSysCtrl(); // 클럭 설정 및 Watchdog 비활성화
    InitGpio();
    InitPieCtrl();
    IER = 0x0000;               // Core Interrupts Enable Register �ʱ�ȭ
    IFR = 0x0000;               // Core Interrupt Flag Register �ʱ�ȭ
    InitPieVectTable();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; 
    EDIS;
    ERTM;


    easyDSP_SCI_Init();

    Interrupt_initModule();
    Interrupt_initVectorTable();
    
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EINT;

    while(1)
    {};
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

void Init_EPwm_1()      //PWM f = TBCLK / (2 * TBPRD) = 100MHz / (2 * 1000) = 50kHz
{
    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1A to low
    EPwm1Regs.TZCTL.bit.TZB         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1B to low

    EPwm1Regs.TZFRC.bit.OST         = 1;                    // One shot trip active -> Clear PWM
    EDIS;

    EPwm1Regs.TBPRD                 =   1000;      // Set timer period, PRD = 1000
    EPwm1Regs.TBCTR                 =   0x0000;             // Clear counter
    EPwm1Regs.TBPHS.bit.TBPHS       =   0x0000;             // Clear phase

    //TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) = SYSCLK = 100MHz
    EPwm1Regs.TBCTL.bit.HSPCLKDIV   =   TB_DIV1;            // divider = /1
    EPwm1Regs.TBCTL.bit.CLKDIV      =   TB_DIV1;            // divider = /1
    EPwm1Regs.TBCTL.bit.CTRMODE     =   TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.PHSEN       =   TB_ENABLE;          // Phase Loading 활성화 -> 다른 PWM 모듈과 동기화 시 필요
    EPwm1Regs.TBCTL.bit.PRDLD       =   TB_SHADOW;          // 주기(TBPRD) 값을 shadow 레지스터에서 읽어옴 (카운터 = 0 일 때)
    EPwm1Regs.TBCTL.bit.SYNCOSEL    =   TB_CTR_ZERO;        // 다른 PMW 모듈과 동기화 위해, 카운터 0일 때 Sync out 신호 발생 -> Master

    EPwm1Regs.CMPCTL.bit.SHDWAMODE  =   CC_SHADOW;          // CMPA Shadow mode Enable
    EPwm1Regs.CMPCTL.bit.SHDWBMODE  =   CC_SHADOW;          // CMPB Shadow mode Enable
    EPwm1Regs.CMPCTL.bit.LOADAMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)
    EPwm1Regs.CMPCTL.bit.LOADBMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)

    EPwm1Regs.AQCTLA.bit.CAU        =   AQ_CLEAR;           // Action when counter = CMPA on up-count is Clear
    EPwm1Regs.AQCTLA.bit.CAD        =   AQ_SET;             // Action when counter = CMPA on down-count is Set

    EPwm1Regs.CMPA.bit.CMPA         =   0x0000;             //Clear Compare Regs

    EPwm1Regs.DBCTL.bit.OUT_MODE    =   DB_FULL_ENABLE;     // EPWMxA/B에 대해 상승(RED)/하강(FED) 에지 모두 데드 밴드 활성화
    EPwm1Regs.DBCTL.bit.POLSEL      =   DB_ACTV_HIC;        // EPWMxB는 EPWMxA의 반전(inverted) 신호로 출력 (상보적인 신호)
    EPwm1Regs.DBCTL.bit.IN_MODE     =   DBA_ALL;            // EPWMxA 신호를 데드 밴드 로직의 입력 소스로 사용
    EPwm1Regs.DBRED.bit.DBRED       =   20;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay
    EPwm1Regs.DBFED.bit.DBFED       =   20;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay

    EPwm1Regs.ETSEL.bit.SOCAEN      = 1;                    // SOCA 이벤트 생성 기능 활성화
    EPwm1Regs.ETSEL.bit.SOCASEL     = 2;                    // 트리거 시점: 카운터가 '주기값(TBPRD)'에 도달했을 때.
    EPwm1Regs.ETPS.bit.SOCAPRD      = 1;                    // Generate pulse on 1st event

    EPwm1Regs.ETSEL.bit.INTSEL      = ET_CTR_ZERO;          // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN       = 1;                    // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD       = ET_1ST;               // Generate INT on 1st event
}
