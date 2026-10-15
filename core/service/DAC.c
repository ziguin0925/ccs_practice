/*
 * DAC.c
 *
 *  Created on: 2026. 9. 16.
 *      Author: h04
 */

/*************************************************************************
* INCLUDES
*************************************************************************/
#include "DAC.h"
/*************************************************************************
* LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
*************************************************************************/

/*************************************************************************
* LOCAL CONSTANT DEFINITIONS
*************************************************************************/

/*************************************************************************
* LOCAL VARIABLE DEFINITIONS
*************************************************************************/

/*************************************************************************
* LOCAL FUNCTION PROTOTYPES
*************************************************************************/

/*************************************************************************
* Function
*************************************************************************/
void InitDac(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0; // Enable pull-up on GPIO63 (SPISIMOB)
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0; // Enable pull-up on GPIO65 (SPICLKB)
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0; // Enable pull-up on GPIO66 (SPISTEB)


    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3; // Asynch input GPIO63 (SPISIMOB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 3; // Asynch input GPIO65 (SPICLKB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 3; // Asynch input GPIO66 (SPISTEB)


    GpioCtrlRegs.GPAMUX2.bit.GPIO30     = 2;//3; // Configure GPIO63 as SPISIMOB
    GpioCtrlRegs.GPAGMUX2.bit.GPIO30    = 2;        // MUX POSITION 15

    GpioCtrlRegs.GPAMUX2.bit.GPIO26     = 2; // Configure GPIO65 as SPICLKB
    GpioCtrlRegs.GPAGMUX2.bit.GPIO26    = 2;        // MUX POSITION 15

    GpioCtrlRegs.GPAMUX2.bit.GPIO27     = 2;//3; // Configure GPIO66 as SPISTEB
    GpioCtrlRegs.GPAGMUX2.bit.GPIO27     = 2;       // MUX POSITION 6

    EDIS;


    EALLOW;

    GPIO_SetupPinMux(30, GPIO_MUX_CPU1, 3); //DA_LOAD
    GPIO_SetupPinOptions(30, GPIO_OUTPUT, GPIO_PULLUP); //

    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 6); //DA_LOAD
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PULLUP); //

    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 6); //DA_LOAD
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PULLUP); //

    GPIO_SetupPinMux(25, 0, 0); //DA_LOAD
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PULLUP); //


    EDIS;


    SpibRegs.SPICCR.all = 0x000F;                   // Reset on, rising edge, 16-bit char bits
    SpibRegs.SPICTL.all = 0x000E;                       // Enable master mode, normal phase,
                                                                                            // enable talk, and SPI int disabled.
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 3;          // 약 22.2Mhz

//    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = (LSPCLK / SpiBitRate) - 1;
    SpibRegs.SPICCR.all =0x008F;    // 0x009F;    // Relinquish SPI from Reset
    SpibRegs.SPIPRI.bit.FREE = 1;               // Set so breakpoints don't disturb xmission


    SpibRegs.SPIFFTX.all=0xA000;//0xE05F;
    SpibRegs.SPIFFRX.all=0x2000;
    SpibRegs.SPIFFCT.all=0x0;//0x8;//0x4;   //Transmission delay.

    SpibRegs.SPICCR.bit.SPISWRESET =1;
}

void DAC_Out(int ch, int dat)
{
    int d;

    switch(ch)
    {
        case 0  :
            d = (0x0000 | (dat & 0xfff));   // DAC_A
            SpibRegs.SPIDAT = d;
            while(SpibRegs.SPISTS.bit.INT_FLAG == 0);
            d = SpibRegs.SPIRXBUF;          // clear SPI INT Flag
        break;

        case 1  :
            d = (0x4000 | (dat & 0xfff));   // DAC_B
            SpibRegs.SPIDAT = d;
            while(SpibRegs.SPISTS.bit.INT_FLAG == 0);
            d = SpibRegs.SPIRXBUF;          // clear SPI INT Flag
        break;

        case 2  :
            d = (0x8000 | (dat & 0xfff));   // DAC_C
            SpibRegs.SPIDAT = d;
            while(SpibRegs.SPISTS.bit.INT_FLAG == 0);
            d = SpibRegs.SPIRXBUF;          // clear SPI INT Flag
        break;

        case 3  :
            d = (0xc000 | (dat & 0xfff));   // DAC_D
            SpibRegs.SPIDAT = d;
            while(SpibRegs.SPISTS.bit.INT_FLAG == 0);
            d = SpibRegs.SPIRXBUF;          // clear SPI INT Flag
        break;

        default :
        break;

    }

    DELAY_US(0.01L);
    GpioDataRegs.GPACLEAR.bit.GPIO25    = 1;  // GpioDataRegs.GPCCLEAR.bit.GPIO64   = 1;
    DELAY_US(0.01L);
    GpioDataRegs.GPASET.bit.GPIO25      = 1;
    DELAY_US(0.01L);
}
