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
