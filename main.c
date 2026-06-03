#include "F28x_Project.h"

int fortesta = 0;
float for_test_B = 0.02;
/**
 * main.c
 */
int main(void)
{
    InitSysCtrl();
    InitGpio();
    InitPieCtrl();
    IER = 0x0000;               // Core Interrupts Enable Register 초기화
    IFR = 0x0000;               // Core Interrupt Flag Register 초기화
    InitPieVectTable();
    easyDSP_SCI_Init();
    while(1)
    {};
  return 0;
}
