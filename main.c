#include "F28x_Project.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "sysctl.h"
#include "driverlib.h"
#include "device.h"
#include <stdbool.h>
#include "./core/pwm_custom/pwm_custom.h"



int fortesta = 0;
float for_test_B = 0.02;
int Loopcnt;



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

    // end init ePWM
    easyDSP_SCI_Init();

    EINT;
    ERTM;


    while(1)
    {
        Loopcnt++;
    };
}

//void Init_EPwm_2()      //PWM f = TBCLK / (2 * TBPRD) = 100MHz / (2 * 1000) = 50kHz
//{
//    EALLOW;
//    EPwm2Regs.TZCTL.bit.TZA         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1A to low
//    EPwm2Regs.TZCTL.bit.TZB         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1B to low
//    EPwm2Regs.TZFRC.bit.OST         = 1;                    // One shot trip active -> Clear PWM
//    EDIS;
//
//    EPwm2Regs.TBPRD                 =   TBPRD_BASE;      // Set timer period, PRD = 1000
//    EPwm2Regs.TBCTR                 =   0x0000;             // Clear counter
//    EPwm2Regs.TBPHS.bit.TBPHS       =   1000;             // Clear phase
//
//    //TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) = SYSCLK = 100MHz
//    EPwm2Regs.TBCTL.bit.HSPCLKDIV   =   TB_DIV1;            // divider = /1
//    EPwm2Regs.TBCTL.bit.CLKDIV      =   TB_DIV1;            // divider = /1
//    EPwm2Regs.TBCTL.bit.CTRMODE     =   TB_COUNT_UPDOWN;
//    EPwm2Regs.TBCTL.bit.PHSEN       =   TB_ENABLE;          // Phase Loading 활성화 -> 다른 PWM 모듈과 동기화 시 필요
//    EPwm2Regs.TBCTL.bit.PRDLD       =   TB_SHADOW;          // 주기(TBPRD) 값을 shadow 레지스터에서 읽어옴 (카운터 = 0 일 때)
//    EPwm2Regs.TBCTL.bit.SYNCOSEL    =   TB_CTR_ZERO;        // 다른 PMW 모듈과 동기화 위해, 카운터 0일 때 Sync out 신호 발생 -> Master
//
//    EPwm2Regs.CMPCTL.bit.SHDWAMODE  =   CC_SHADOW;          // CMPA Shadow mode Enable
//    EPwm2Regs.CMPCTL.bit.SHDWBMODE  =   CC_SHADOW;          // CMPB Shadow mode Enable
//    EPwm2Regs.CMPCTL.bit.LOADAMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)
//    EPwm2Regs.CMPCTL.bit.LOADBMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)
//
//    EPwm2Regs.AQCTLA.bit.CAU        =   AQ_CLEAR;           // Action when counter = CMPA on up-count is Clear
//    EPwm2Regs.AQCTLA.bit.CAD        =   AQ_SET;             // Action when counter = CMPA on down-count is Set
//
//    EPwm2Regs.CMPA.bit.CMPA         =   (0.5)*EPwm2Regs.TBPRD;             //Clear Compare Regs
//
//    EPwm2Regs.DBCTL.bit.OUT_MODE    =   DB_FULL_ENABLE;     // EPWMxA/B에 대해 상승(RED)/하강(FED) 에지 모두 데드 밴드 활성화
//    EPwm2Regs.DBCTL.bit.POLSEL      =   DB_ACTV_HIC;        // EPWMxB는 EPWMxA의 반전(inverted) 신호로 출력 (상보적인 신호)
//    EPwm2Regs.DBCTL.bit.IN_MODE     =   DBA_ALL;            // EPWMxA 신호를 데드 밴드 로직의 입력 소스로 사용
//    EPwm2Regs.DBRED.bit.DBRED       =   20;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay
//    EPwm2Regs.DBFED.bit.DBFED       =   20;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay
//
//    EPwm2Regs.ETSEL.bit.SOCAEN      = 1;                    // SOCA 이벤트 생성 기능 활성화
//    EPwm2Regs.ETSEL.bit.SOCASEL     = 2;                    // 트리거 시점: 카운터가 '주기값(TBPRD)'에 도달했을 때.
//    EPwm2Regs.ETPS.bit.SOCAPRD      = 1;                    // Generate pulse on 1st event
//
//    EPwm2Regs.ETSEL.bit.INTSEL      = ET_CTR_ZERO;          // Select INT on Zero event
//    EPwm2Regs.ETSEL.bit.INTEN       = 1;                    // Enable INT
//    EPwm2Regs.ETPS.bit.INTPRD       = ET_1ST;               // Generate INT on 1st event
//}

