#include "F28x_Project.h"
#include "hw_ints.h"
#include "sysctl.h"
#include "interrupt.h"
#include "driverlib.h"

#define EPWM1_TIMER_TBPRD  2000U
#define EPWM1_MAX_CMPA     1950U
#define EPWM1_MIN_CMPA       50U
#define EPWM1_MAX_CMPB     1950U
#define EPWM1_MIN_CMPB       50U


#define EPWM_CMP_UP           1U
#define EPWM_CMP_DOWN         0U


int fortesta = 0;
float for_test_B = 0.02;
typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmCompADirection;
    uint16_t epwmCompBDirection;
    uint16_t epwmTimerIntCount;
    uint16_t epwmMaxCompA;
    uint16_t epwmMinCompA;
    uint16_t epwmMaxCompB;
    uint16_t epwmMinCompB;
}epwmInformation;


epwmInformation epwm1Info;
/**
 * main.c
 */

__interrupt void epwm1ISR(void)
{
    //
    // Update the CMPA and CMPB values
    //
    updateCompare(&epwm1Info);

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}



void initEPWM1()
{
    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm1Info.epwmCompADirection = EPWM_CMP_UP;
    epwm1Info.epwmCompBDirection = EPWM_CMP_DOWN;
    epwm1Info.epwmTimerIntCount = 0U;
    epwm1Info.epwmModule = EPWM1_BASE;
    epwm1Info.epwmMaxCompA = EPWM1_MAX_CMPA;
    epwm1Info.epwmMinCompA = EPWM1_MIN_CMPA;
    epwm1Info.epwmMaxCompB = EPWM1_MAX_CMPB;
    epwm1Info.epwmMinCompB = EPWM1_MIN_CMPB;
}

void updateCompare(epwmInformation epwmInfo)
{
    uint16_t compAValue;
    uint16_t compBValue;

    compAValue = EPWM_getCounterCompareValue(epwmInfo.epwmModule,
                                             EPWM_COUNTER_COMPARE_A);

    compBValue = EPWM_getCounterCompareValue(epwmInfo.epwmModule,
                                             EPWM_COUNTER_COMPARE_B);

    //
    //  Change the CMPA/CMPB values every 10th interrupt.
    //
    if(epwmInfo.epwmTimerIntCount == 10U)
    {
        epwmInfo.epwmTimerIntCount = 0U;

        //
        // If we were increasing CMPA, check to see if we reached the max
        // value. If not, increase CMPA else, change directions and decrease
        // CMPA
        //
        if(epwmInfo.epwmCompADirection == EPWM_CMP_UP)
        {
            if(compAValue < (epwmInfo.epwmMaxCompA))
            {
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            ++compAValue);
            }
            else
            {
                epwmInfo.epwmCompADirection = EPWM_CMP_DOWN;
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            --compAValue);
            }
        }
        //
        // If we were decreasing CMPA, check to see if we reached the min
        // value. If not, decrease CMPA else, change directions and increase
        // CMPA
        //
        else
        {
            if( compAValue == (epwmInfo.epwmMinCompA))
            {
                epwmInfo.epwmCompADirection = EPWM_CMP_UP;
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            ++compAValue);
            }
            else
            {
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            --compAValue);
            }
        }

        //
        // If we were increasing CMPB, check to see if we reached the max
        // value. If not, increase CMPB else, change directions and decrease
        // CMPB
        //
        if(epwmInfo.epwmCompBDirection == EPWM_CMP_UP)
        {
            if(compBValue < (epwmInfo.epwmMaxCompB))
            {
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_B,
                                            ++compBValue);
            }
            else
            {
                epwmInfo.epwmCompBDirection = EPWM_CMP_DOWN;
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_B,
                                            --compBValue);
            }
        }
        //
        // If we were decreasing CMPB, check to see if we reached the min
        // value. If not, decrease CMPB else, change directions and increase
        // CMPB
        //
        else
        {
            if(compBValue == (epwmInfo.epwmMinCompB))
            {
                epwmInfo.epwmCompBDirection = EPWM_CMP_UP;
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_B,
                                            ++compBValue);
            }
            else
            {
                EPWM_setCounterCompareValue(epwmInfo.epwmModule,
                                            EPWM_COUNTER_COMPARE_B,
                                            --compBValue);
            }
        }
    }
    else
    {
        epwmInfo.epwmTimerIntCount++;
    }
}


int main(void)
{
    InitSysCtrl(); // 클럭 설정 및 Watchdog 비활성화
    InitGpio();
    InitPieCtrl();
    IER = 0x0000;               // Core Interrupts Enable Register �ʱ�ȭ
    IFR = 0x0000;               // Core Interrupt Flag Register �ʱ�ȭ
    InitPieVectTable();
    easyDSP_SCI_Init();

    Interrupt_initModule();
    Interrupt_initVectorTable();

    
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    
    // Board_init();
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    
    Interrupt_register(INT_EPWM1, &epwm1ISR);

    Interrupt_enable(INT_EPWM1);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    while(1)
    {};
  return 0;
}

