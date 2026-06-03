/***************************************************************
    easy28x_bitfield_v*.*.c
****************************************************************/
// Included Files
#include "F28x_Project.h"
#include "easy28x_bitfield_v11.2.h"

//////////////////////////////////////////////////
// DON'T CHANGE THIS FILE
// 소스 파일이 아닌 헤더 파일을 변경하셔야 합니다.
//////////////////////////////////////////////////

// function declaration
__interrupt void easy_RXINT_ISR(void);
void easyDSP_SCIBootCPU2(void);
void easyDSP_SCIBootCM(void);                // for F2838xS and F2838xD
void easyDSP_FlashBootCPU2(void);
void easyDSP_2838x_Sci_Boot_Sync(void);
void ezDSP_GPIO_SetupPinMux(Uint16 gpioNumber, Uint16 cpu, Uint16 muxPosition);
void ezDSP_GPIO_setMasterCoreToCm(uint32_t pin);

#define F2838xX_CPU1    (F2838xS_CPU1       || F2838xD_CPU1)
#define F2838xX_CPU1_CM (F2838xS_CPU1_CM    || F2838xD_CPU1_CM)
#define F2838xX_ALL     (F2838xS_CPU1       || F2838xS_CPU1_CM || F2838xD_CPU1 || F2838xD_CPU1_CPU2 || F2838xD_CPU1_CM || F2838xD_CPU1_CPU2_CM)

// SCI-A is mandatory for single core or multi core CPU1
// SCI-B is mandatory for CPU2 of multi core
// in case you like to use SCI-C or SCI-D for F2837xD/F2838xD CPU2,
// you have to change the coding in easyDSP_SCI_Init() accordingly
#if (F28P65xD_CPU1_CPU2 || F2837xD_CPU1_CPU2 || F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM) && defined(CPU2)
#define ScixRegs   ScibRegs
#else
#define ScixRegs   SciaRegs
#endif


// for internal use : don't change or delete
// v10.1 : location moved
#define VER_OF_THIS_FILE    1120
volatile unsigned int ezDSP_Version_SCI = VER_OF_THIS_FILE;                     // don't change the variable name
volatile unsigned int ezDSP_uRead16BPossible = 1, ezDSP_uRead8BPossible = 1;    // don't change the variable name
bool ezDSP_b32bitAddress = false;                                               // don't change the variable name

#if defined(CPU1)
unsigned int ezDSP_uCPU = 1;
#elif defined(CPU2)
unsigned int ezDSP_uCPU = 2;
#elif !defined(CPU1)    // v10.6
#define CPU1
unsigned int ezDSP_uCPU = 1;
#endif

unsigned int ezDSP_uOnChipFlash = 0;


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SCI initialization
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void easyDSP_SCI_Init()
{
	int i;

	// v10.1 : make sure that these variables will survive even after optimization
	ezDSP_Version_SCI = VER_OF_THIS_FILE;
	ezDSP_uRead16BPossible = 1;
	ezDSP_uRead8BPossible = 1;

#if defined(CPU1)
	ezDSP_uCPU = 1;
#elif defined(CPU2)
	ezDSP_uCPU = 2;
#endif

	// CLOCK ENABLE AND PIN MAPPING
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if F28004x || F28002x || F28003x || F28001x || F28P55xS
    // SCI-A CLOCK ENABLE
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    EDIS;
    // init the pins for the SCI-A port.
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC); //v9.2  GPIO_PUSHPULL
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif F2807x || F2837xS || F2837xD_CPU1
#if F2837xD_CPU1
    // SCI-A to CPU1
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_A = 0;       // This register must be configured prior to enabling the peripheral clocks.
    EDIS;
#endif

     // SCI-A CLOCK ENABLE
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    EDIS;

    GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC); // v9.2
    GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif F2837xD_CPU1_CPU2
#ifdef CPU1
    ///////////////////////////////////////////
    // SCI-A assigned to CPU1
    ///////////////////////////////////////////
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_A = 0;       // This register must be configured prior to enabling the peripheral clocks.
    EDIS;

    ///////////////////////////////////////////
    // SCI-B assigned to CPU2
    ///////////////////////////////////////////
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_B = 0;       // SCI-B CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_B = 1;
    EDIS;

    ///////////////////////////////////////////
    // SCI-A GPIO setting : SCIRXDA = GPIO 85, SCITXDA = GPIO84
    ///////////////////////////////////////////
    GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 5);
    GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC); // v9.2

    ///////////////////////////////////////////
    // SCI-B GPIO Setting : SCIRXDB = GPIO 87, SCITXDB = GPIO86
    ///////////////////////////////////////////
    GPIO_SetupPinMux(86, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinMux(87, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_ASYNC);
    GPIO_SetupPinOptions(87, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC); // v9.2

#endif

    EALLOW;
#if defined (CPU1)
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;       // SCI-A CLOCK Enabled
#elif defined (CPU2)
    CpuSysRegs.PCLKCR7.bit.SCI_B = 1;       // SCI-B CLOCK Enabled
#endif
    EDIS;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #elif F28P65xS || F28P65xD_CPU1
    #if F28P65xD_CPU1
        // SCI-A to CPU1
        EALLOW;
        CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
        DevCfgRegs.CPUSEL5.bit.SCI_A = 0;       // This register must be configured prior to enabling the peripheral clocks.
        EDIS;
    #endif
        // SCI-A CLOCK ENABLE
        EALLOW;
        CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
        EDIS;
        // SCI-A GPIO
        GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(13, GPIO_INPUT, GPIO_PUSHPULL);//GPIO_PULLUP | GPIO_ASYNC);
        GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_ASYNC);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #elif F28P65xD_CPU1_CPU2
    #ifdef CPU1
        ///////////////////////////////////////////
        // SCI-A assigned to CPU1
        ///////////////////////////////////////////
        EALLOW;
        CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
        DevCfgRegs.CPUSEL5.bit.SCI_A = 0;       // This register must be configured prior to enabling the peripheral clocks.
        EDIS;
        ///////////////////////////////////////////
        // SCI-B assigned to CPU2
        ///////////////////////////////////////////
        EALLOW;
        CpuSysRegs.PCLKCR7.bit.SCI_B = 0;       // SCI-B CLOCK Disabled
        DevCfgRegs.CPUSEL5.bit.SCI_B = 1;
        EDIS;
        ///////////////////////////////////////////
        // SCI-A GPIO setting : SCIRXDA = GPIO 13, SCITXDA = GPIO 12
        ///////////////////////////////////////////
        GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(13, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
        GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_ASYNC);

        ///////////////////////////////////////////
        // SCI-B GPIO Setting : SCIRXDB = GPIO 87, SCITXDB = GPIO 86
        // GPIO for SCI-B can be changed by modifying below code
        ///////////////////////////////////////////
        GPIO_SetupPinMux(86, GPIO_MUX_CPU2, 5);
        GPIO_SetupPinMux(87, GPIO_MUX_CPU2, 5);
        GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_ASYNC);
        GPIO_SetupPinOptions(87, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    #endif
        EALLOW;
    #if defined (CPU1)
        CpuSysRegs.PCLKCR7.bit.SCI_A = 1;       // SCI-A CLOCK Enabled
    #elif defined (CPU2)
        CpuSysRegs.PCLKCR7.bit.SCI_B = 1;       // SCI-B CLOCK Enabled
    #endif
        EDIS;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    #elif F2838xX_ALL
#ifdef CPU1
    ///////////////////////////////////////////
    // SCI-A assigned to CPU1
    ///////////////////////////////////////////
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_A = 0;       // This register must be configured prior to enabling the peripheral clocks.
    EDIS;
    ///////////////////////////////////////////
    // SCI-A GPIO setting : SCIRXDA = GPIO 28, SCITXDA = GPIO29
    ///////////////////////////////////////////
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

    ///////////////////////////////////////////
    // SCI-A CLOCK ENABLE
    ///////////////////////////////////////////
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    EDIS;
#if F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
    ///////////////////////////////////////////
    // SCI-B assigned to CPU2
    ///////////////////////////////////////////
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_B = 0;       // SCI-B CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_B = 1;
    EDIS;
    ///////////////////////////////////////////
    // SCI-B GPIO Setting : SCIRXDB = GPIO 15, SCITXDB = GPIO14
    ///////////////////////////////////////////
    GPIO_SetupPinMux(15, GPIO_MUX_CPU2, 2);
    GPIO_SetupPinMux(14, GPIO_MUX_CPU2, 2);
    GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_ASYNC);
    ///////////////////////////////////////////
    // SCI-B CLOCK ENABLE
    ///////////////////////////////////////////
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_B = 1;       // SCI-B CLOCK Enabled
    EDIS;
#endif
#if F2838xX_CPU1_CM || F2838xD_CPU1_CPU2_CM
    ///////////////////////////////////////////
    // UART GPIO Setting : UART-A RX = GPIO 85, UART-A Tx = GPIO84
    ///////////////////////////////////////////
    GPIO_SetupPinMux(85, GPIO_MUX_CPU1, 11);    // first, to CPU1
    GPIO_SetupPinMux(84, GPIO_MUX_CPU1, 11);
    EALLOW;
    GpioCtrlRegs.GPCCSEL3.bit.GPIO85 = 0x4;//0b100;   // switch to CM
    GpioCtrlRegs.GPCCSEL3.bit.GPIO84 = 0x4;//0b100;
    EDIS;
    GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
#endif
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#else
    this will create compiler error intentionally
#endif

    // configure SCI
    ScixRegs.SCICCR.bit.STOPBITS = 0;   // 1 stop
    ScixRegs.SCICCR.bit.PARITYENA = 0;  // no parity
    ScixRegs.SCICCR.bit.LOOPBKENA = 0;  // no loopback
    ScixRegs.SCICCR.bit.SCICHAR = 7;    // 8bit char
    i = (int)(((float)LSP_CLK/(BAUDRATE*8.) - 1) + 0.5);
    ScixRegs.SCIHBAUD.all = i >> 8;
    ScixRegs.SCILBAUD.all = i & 0xFF;

    // enable Module
    ScixRegs.SCICTL1.bit.SWRESET = 1;
    ScixRegs.SCICTL1.bit.TXENA = 1;
    ScixRegs.SCICTL1.bit.RXENA = 1;

    // enable FIFO
    ScixRegs.SCIFFTX.bit.SCIRST = 1;
    ScixRegs.SCIFFTX.bit.SCIFFENA = 1;
    ScixRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScixRegs.SCIFFRX.bit.RXFIFORESET = 1;

    // enable FIFO interrupt
    ScixRegs.SCIFFRX.bit.RXFFIENA = 1;
    //ScixRegs.SCICTL1.bit.RXERRINTENA = 1;

    // FIFO interrupts level
    ScixRegs.SCIFFTX.bit.TXFFIL = 1;
    ScixRegs.SCIFFRX.bit.RXFFIL = 1;

    // perform SW reset
    ScixRegs.SCICTL1.bit.SWRESET = 0;
    ScixRegs.SCICTL1.bit.SWRESET = 1;

    // reset FIFO
    ScixRegs.SCIFFTX.bit.TXFIFORESET = 0;
    ScixRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScixRegs.SCIFFRX.bit.RXFIFORESET = 0;
    ScixRegs.SCIFFRX.bit.RXFIFORESET = 1;

    // Reassign ISR for easyDSP. Don't use SCIRXINTA_ISR & SCITXINTA_ISR found in F28x_DefaultIsr.c.
	EALLOW;
#if (F28P65xD_CPU1_CPU2 || F2837xD_CPU1_CPU2 || F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM) && defined(CPU2) // v9.3
	PieVectTable.SCIB_RX_INT = &easy_RXINT_ISR;
#else
	PieVectTable.SCIA_RX_INT = &easy_RXINT_ISR;
#endif
	EDIS;

	// Enable interrupts required
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;		// Enable the PIE block
#if (F28P65xD_CPU1_CPU2 || F2837xD_CPU1_CPU2 || F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM) && defined(CPU2) // v9.3
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      // Enable SCI-B RX INT in the PIE: Group 9 interrupt 3
#else
	PieCtrlRegs.PIEIER9.bit.INTx1 = 1;      // Enable SCI-A RX INT in the PIE: Group 9 interrupt 1
#endif

	IER |= M_INT9;							// Enable CPU INT9 for SCI-A  (M_INT9 = 0x100)
	EINT;									// Enable Global interrupt INTM

    // others
#ifdef _FLASH
    ezDSP_uOnChipFlash = 1;
#endif
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// F2837x multi core boot operation
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#if F2837xD_CPU1_CPU2
void easyDSP_Boot_Sync(void)
{
#ifdef CPU1
#ifdef _FLASH
    easyDSP_FlashBootCPU2();
#else
    easyDSP_SCIBootCPU2();
#endif
#endif

#ifdef CPU2
#ifndef _FLASH
    easyDSP_SCIBootCPU2();
#endif
#endif
}

#ifdef CPU1
#include "F2837xD_Ipc_drivers.h"
#ifndef _FLASH
#define BROM_IPC_EXECUTE_BOOTMODE_CMD           0x00000013
#define C1C2_BROM_BOOTMODE_BOOT_FROM_SCI        0x00000001
#define C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH      0x0000000B
void easyDSP_SCIBootCPU2(void)
{
    // SCIA connected to CPU2
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1;   // This register must be configured prior to enabling the peripheral clocks.
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    EDIS;

    // GPIO 84, 85 to CPU2
    GPIO_SetupPinMux(84, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinMux(85, GPIO_MUX_CPU2, 5);

#if 0   // from v11
    // Assign all GS RAMs to CPU2
    EALLOW;
    MemCfgRegs.GSxMSEL.all = 0xFFFF;    // NOTE : don't assign GSxRAM to CPU2 where .text of CPU1 is located !!!!!!!
    EDIS;
#endif

    IpcRegs.IPCBOOTMODE = C1C2_BROM_BOOTMODE_BOOT_FROM_SCI; // CPU1 to CPU2 IPC Boot Mode Register
    IpcRegs.IPCSENDCOM = BROM_IPC_EXECUTE_BOOTMODE_CMD;     // C1C2_BROM_IPC_EXECUTE_BOOTMODE_CMD;  // CPU1 to CPU2 IPC Command Register
    IpcRegs.IPCSET.all = 0x80000001;                        // CPU1 to CPU2 IPC flag register

    //wait until CPU2 is booted and set IPC5
    while(IpcRegs.IPCSTS.bit.IPC5 != 1);
    IpcRegs.IPCACK.bit.IPC5 = 1; //clearing the ack flag

#if 0   // from v11
    // Assign all GS RAMs to CPU1
    EALLOW;
    MemCfgRegs.GSxMSEL.all = 0x0000;
    EDIS;
#endif

    // SCIA connected to CPU1
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;
    DevCfgRegs.CPUSEL5.bit.SCI_A = 0;   // This register must be configured prior to enabling the peripheral clocks.
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    EDIS;
}
#endif  // _FLASH
#ifdef _FLASH
void easyDSP_FlashBootCPU2(void)
{
    // Wait until CPU02 control system boot ROM is ready to receive CPU01 to CPU02 INT1 interrupts.
    uint32_t bootStatus;
    while(1) {
        bootStatus = IPCGetBootStatus() & 0x0000000F;
        if(bootStatus == C2_BOOTROM_BOOTSTS_SYSTEM_READY) break;
    }

    // Loop until CPU02 control system IPC flags 1 and 32 are available
    while ((IPCLtoRFlagBusy(IPC_FLAG0) == 1) || (IPCLtoRFlagBusy(IPC_FLAG31) == 1));

    IpcRegs.IPCBOOTMODE = C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH;   // CPU1 to CPU2 IPC Boot Mode Register
    IpcRegs.IPCSENDCOM = BROM_IPC_EXECUTE_BOOTMODE_CMD; // C1C2_BROM_IPC_EXECUTE_BOOTMODE_CMD;  // CPU1 to CPU2 IPC Command Register
    IpcRegs.IPCSET.all = 0x80000001; // CPU1 to CPU2 IPC flag register
}
#endif  // _FLASH
#endif  // CPU1

#ifdef CPU2
#ifndef _FLASH
void easyDSP_SCIBootCPU2(void)
{
    // booting feedback still working even this moment
    // So no switch SCIA channel to CPU1 immediately
    DELAY_US(1000*100);

    // set IPC5 to inform to CPU1 that booting is done
    IpcRegs.IPCSET.bit.IPC5 = 1;
    while(IpcRegs.IPCFLG.bit.IPC5 == 1);
    asm(" NOP");
    asm(" NOP");

    // Wait until CPU1 is configuring SCI-B for CPU2 in easyDSP_SCI_Init()
    DELAY_US(1000*10);
}
#endif  // _FLASH
#endif  // CPU2
#endif  // F2837xD_CPU1_CPU2


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// F2838x multi core boot operation
// note : For 2838xD, you can modify flash sector location if sector 0 is not preferred
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#if F2838xS_CPU1_CM || F2838xD_CPU1_CM || F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
void ezDSP_Device_bootCPU2(uint32_t u32BootMode);
void ezDSP_Device_bootCM(uint32_t u32BootMode);
#define BOOTMODE_BOOT_TO_FLASH_SECTOR0          0x03U
#define BOOTMODE_BOOT_TO_FLASH_SECTOR4          0x23U
#define BOOTMODE_BOOT_TO_FLASH_SECTOR8          0x43U
#define BOOTMODE_BOOT_TO_FLASH_SECTOR13         0x63U
void easyDSP_Boot_Sync(void)
{
#ifdef _FLASH
#ifdef CPU1
#if F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
    ezDSP_Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
    //ezDSP_Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR4);  // other options
    //ezDSP_Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR8);
    //ezDSP_Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR13);
#endif
#if F2838xS_CPU1_CM || F2838xD_CPU1_CM || F2838xD_CPU1_CPU2_CM
    ezDSP_Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
    //ezDSP_Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR4);    // other options
    //ezDSP_Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR8);
    //ezDSP_Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR13);
#endif
#endif // CPU1
#else   // if not flash
#if F2838xD_CPU1_CPU2 || F2838xD_CPU1_CM || F2838xS_CPU1_CM || F2838xD_CPU1_CPU2_CM
    easyDSP_2838x_Sci_Boot_Sync();
#endif
#endif  // _FLASH
}

#ifdef CPU1
#define BOOT_KEY                                0x5A000000UL
#define CPU2_BOOT_FREQ_200MHZ                   0xC800U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_1000W   0xA0000U
#define BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM  0x0CU
#define BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_S0RAM  0x0CU
#define CM_BOOT_FREQ_125MHZ                     0x7D00U
#include "f2838x_ipc_defines.h"
void ezDSP_Device_bootCPU2(uint32_t u32BootMode)
{
    // Configure the CPU1TOCPU2IPCBOOTMODE register
    Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCBOOTMODE = (BOOT_KEY | CPU2_BOOT_FREQ_200MHZ | u32BootMode);
    // Set IPC Flag 0
    IPCLtoRFlagSet(IPC_FLAG0);
    // Bring CPU2 out of reset. Wait for CPU2 to go out of reset.
    EALLOW;
    DevCfgRegs.CPU2RESCTL.all = 0xA5A50000;
    EDIS;
    while(DevCfgRegs.RSTSTAT.bit.CPU2RES == 0);
}

void ezDSP_Device_bootCM(uint32_t u32BootMode)
{
    // Configure the CPU1TOCMIPCBOOTMODE register
    Cpu1toCmIpcRegs.CPU1TOCMIPCBOOTMODE = (BOOT_KEY | CM_BOOT_FREQ_125MHZ | u32BootMode);
    // Set IPC Flag 0
    //Cpu1toCmIpcRegs.CPU1TOCMIPCSET.bit.IPC0 = 1;
    Cpu1toCmIpcRegs.CPU1TOCMIPCSET.all |= IPC_FLAG0;
    // Bring CM out of reset only when CM is in reset state. Wait for CM to go out of reset.
    if(CmConfRegs.CMRESCTL.bit.RESETSTS == 0) {
        EALLOW;
        CmConfRegs.CMRESCTL.all = 0xA5A50000;
        EDIS;
    }
    while(CmConfRegs.CMRESCTL.bit.RESETSTS == 0);
}
#endif  // CPU1

#ifndef _FLASH
#define CPU1_CPU2_SYNC_FLAG         IPC_FLAG5
#define CPU1_CPU2_2ND_SYNC_FLAG     IPC_FLAG6
#define CPU1_CM_SYNC_FLAG           IPC_FLAG31
#define CPU1_CM_COMM_FLAG           IPC_FLAG30
void easyDSP_2838x_Sci_Boot_Sync(void)
{
#ifdef CPU1
#if F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
    easyDSP_SCIBootCPU2();
#endif
#if F2838xX_CPU1_CM || F2838xD_CPU1_CPU2_CM
    easyDSP_SCIBootCM();
#endif
#if F2838xD_CPU1_CPU2_CM
    // second sync with CPU2 after CM boot
    // simulating IPC_sync(IPC_CPU1_L_CPU2_R, CPU1_CPU2_2ND_SYNC_FLAG)
    Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCSET.all = CPU1_CPU2_2ND_SYNC_FLAG;
    while((Cpu1toCpu2IpcRegs.CPU2TOCPU1IPCSTS.all & CPU1_CPU2_2ND_SYNC_FLAG) == 0);
    Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCACK.all = CPU1_CPU2_2ND_SYNC_FLAG;
    while((Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCFLG.all & CPU1_CPU2_2ND_SYNC_FLAG) != 0);
#endif
#endif  // CPU1
#ifdef CPU2
#if F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
    easyDSP_SCIBootCPU2();
    DELAY_US(1000);    // Wait until CPU1 is configuring SCI-B for CPU2 in easyDSP_SCI_Init()
#endif
#if F2838xD_CPU1_CPU2_CM
    // second sync with CPU1 after CM boot
    // simulating IPC_sync(IPC_CPU2_L_CPU1_R, CPU1_CPU2_2ND_SYNC_FLAG);
    Cpu2toCpu1IpcRegs.CPU2TOCPU1IPCSET.all = CPU1_CPU2_2ND_SYNC_FLAG;
    while((Cpu2toCpu1IpcRegs.CPU1TOCPU2IPCSTS.all & CPU1_CPU2_2ND_SYNC_FLAG) == 0);
    Cpu2toCpu1IpcRegs.CPU2TOCPU1IPCACK.all = CPU1_CPU2_2ND_SYNC_FLAG;
    while((Cpu2toCpu1IpcRegs.CPU2TOCPU1IPCFLG.all & CPU1_CPU2_2ND_SYNC_FLAG) != 0);

    DELAY_US(1000);    // Wait until CPU1 is configuring SCI-B for CPU2 in easyDSP_SCI_Init()
#endif
#endif  // CPU2
}

//-----------------------------------------------------------------------------------------------------

#ifdef CPU1
uint32_t ezDSP_u32CPU2BlockWordSize = 0, ezDSP_u32CMBlockWordSize = 0;  // only for monitoring
uint32_t ezDSP_u32CPU2BootStatus = 0, ezDSP_u32CMBootStatus = 0;

uint16_t easyDSP_GetWordData(void)
{
    uint16_t wordData, byteData;
    wordData = byteData = 0x0000;

    // Fetch the LSB and verify back to the host
    while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
    wordData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;
    SciaRegs.SCITXBUF.bit.TXDT = wordData;

    // Fetch the MSB and verify back to the host
    while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
    byteData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;
    SciaRegs.SCITXBUF.bit.TXDT = byteData;

    // form the wordData from the MSB:LSB
    wordData |= (byteData << 8);

    return wordData;
}

uint32_t easyDSP_GetLongData(void)
{
    uint32_t longData;
    longData = ( (uint32_t)easyDSP_GetWordData() << 16);    // Fetch the upper 1/2 of the 32-bit value
    longData |= (uint32_t)easyDSP_GetWordData();            // Fetch the lower 1/2 of the 32-bit value
    return longData;
}

#define  CPU1TOCPU2MSGRAM1_BASE   0x0003A400U    //CPU1 to CPU2 MSGRAM1
#define  CPUXTOCMMSGRAM1_BASE     0x00039400U    //CPU to CM MSGRAM1
void easyDSP_CopyData(uint16_t uMCU)
{
    struct HEADER {
        uint32_t DestAddr;
        uint16_t BlockSize;
    } BlockHeader;

    uint16_t wordData;
    uint16_t i;

   BlockHeader.BlockSize = easyDSP_GetWordData();
   if(uMCU == 3) BlockHeader.BlockSize /= 2;

   // monitoring
   if(uMCU == 2)        ezDSP_u32CPU2BlockWordSize    += BlockHeader.BlockSize;
   else if(uMCU == 3)   ezDSP_u32CMBlockWordSize      += BlockHeader.BlockSize;

   while(BlockHeader.BlockSize != 0)
   {
      BlockHeader.DestAddr = easyDSP_GetLongData();
      if(uMCU == 2) {
          BlockHeader.DestAddr -= 0x400;
          BlockHeader.DestAddr += CPU1TOCPU2MSGRAM1_BASE;
      }
      else if(uMCU == 3) {
          BlockHeader.DestAddr -= 0x20000800;
          BlockHeader.DestAddr /= 2;    // CM address
          BlockHeader.DestAddr += CPUXTOCMMSGRAM1_BASE;
      }

      for(i = 0; i < BlockHeader.BlockSize; i++) {
          wordData = easyDSP_GetWordData();
          if(uMCU == 2)         *(uint16_t *)BlockHeader.DestAddr = wordData;
          else if(uMCU == 3)    *(uint16_t *)BlockHeader.DestAddr = (wordData >> 8) | (wordData << 8);
          BlockHeader.DestAddr++;
      }

      BlockHeader.BlockSize = easyDSP_GetWordData();
      if(uMCU == 3) BlockHeader.BlockSize /= 2;

      // monitoring
      if(uMCU == 2)        ezDSP_u32CPU2BlockWordSize += BlockHeader.BlockSize;
      else if(uMCU == 3)   ezDSP_u32CMBlockWordSize += BlockHeader.BlockSize;
   }
}

// auto baud implementation
void easyDSP_AutoBaud(void)
{
    uint16_t byteData;

    /////////////////////////////////////////////////////////////////////////////
    // Initialize the SCI-A port for communications with the host.
    /////////////////////////////////////////////////////////////////////////////
    // Enable the SCI-A clocks
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    ClkCfgRegs.LOSPCP.all = 0x0002;
    SciaRegs.SCIFFTX.all=0x8000;
    SciaRegs.SCICCR.all = 0x0007;
    SciaRegs.SCICTL1.all = 0x0003;
    SciaRegs.SCICTL2.all = 0x0000;
    SciaRegs.SCICTL1.all = 0x0023;
    EDIS;

    /////////////////////////////////////////////////////////////////////////////
    // Perform autobaud lock with the host.
    /////////////////////////////////////////////////////////////////////////////
    SciaRegs.SCILBAUD.bit.BAUD = 1;
    SciaRegs.SCIFFCT.bit.CDC = 1;
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;
    while(SciaRegs.SCIFFCT.bit.ABD != 1);
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;
    SciaRegs.SCIFFCT.bit.CDC = 0;
    while(SciaRegs.SCIRXST.bit.RXRDY != 1);
    byteData = SciaRegs.SCIRXBUF.bit.SAR;
    SciaRegs.SCITXBUF.bit.TXDT = byteData;
}

#if F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
void easyDSP_SCIBootCPU2(void)
{
    ///////////////////////////////////////////
    // SCI-A to CPU1 : SCIRXDA = GPIO 28, SCITXDA = GPIO29
    ///////////////////////////////////////////
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

    // copy SCI booting agent code to CPU1TOCPU2MSGRAM1
    uint16_t i;
    easyDSP_AutoBaud();
    if(easyDSP_GetWordData() != 0x08AA) while(1);   // KeyValue
    for(i = 0; i < 8; i++) easyDSP_GetWordData();   // 8 reserved words
    easyDSP_GetWordData();                          // entry point
    easyDSP_GetWordData();                          // entry point
    easyDSP_CopyData(2);

    // secure the last echo byte
    DELAY_US(1000*10);

    // SCI-A assigned to CPU2
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1;       // This register must be configured prior to enabling the peripheral clocks.
    EDIS;

    //Allows CPU02 bootrom to take control of clock configuration registers
    EALLOW;
    ClkCfgRegs.CLKSEM.all = 0xA5A50000;
    ClkCfgRegs.LOSPCP.all = 0x0002;
    EDIS;

    ///////////////////////////////////////////
    // SCI-A to CPU2 : SCIRXDA = GPIO 28, SCITXDA = GPIO29
    ///////////////////////////////////////////
    GPIO_SetupPinMux(28, GPIO_MUX_CPU2, 1);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU2, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

#if 0   // v11
    // Assign all GS RAMs to CPU2
    EALLOW;
    MemCfgRegs.GSxMSEL.all = 0xFFFF;    // NOTE : don't assign GSxRAM to CPU2 where .text of CPU1 is located !!!!!!!
    EDIS;
#endif

    // Boot CPU2 /w agent. max. (1000) copy
    //ezDSP_Device_bootCPU2(BOOTMODE_IPC_MSGRAM_COPY_LENGTH_1000W | BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM);    // deleted from v9.5
    Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCBOOTMODE = (BOOT_KEY | CPU2_BOOT_FREQ_200MHZ | BOOTMODE_IPC_MSGRAM_COPY_LENGTH_1000W | BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM); // Configure the CPU1TOCPU2IPCBOOTMODE register
    IPCLtoRFlagSet(IPC_FLAG0);  // Set IPC Flag 0
    // Bring CPU2 out of reset. Wait for CPU2 to go out of reset.
    EALLOW;
    DevCfgRegs.CPU2RESCTL.all = 0xA5A50000;
    EDIS;
    while(DevCfgRegs.RSTSTAT.bit.CPU2RES == 0);

    DELAY_US(1000*1);    // wait CPU2 boot
    ezDSP_u32CPU2BootStatus = Cpu1toCpu2IpcRegs.CPU2TOCPU1IPCBOOTSTS;

    ///////////////////////////////////////////////////////////////////////////////////
    // download target CPU2 code via SCI-A. This is done by agent program in CPU2
    ///////////////////////////////////////////////////////////////////////////////////

    //wait until CPU2 is booted and set CPU1_CPU2_SYNC_FLAG
    // Synchronize both the cores
    while(!IPCRtoLFlagBusy(CPU1_CPU2_SYNC_FLAG));
    IPCRtoLFlagAcknowledge(CPU1_CPU2_SYNC_FLAG);

#if 0   // v11
    // Assign all GS RAMs to CPU1
    EALLOW;
    MemCfgRegs.GSxMSEL.all = 0x0000;
    EDIS;
#endif

    // SCI-A assigned to CPU1
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_A = 0;       // This register must be configured prior to enabling the peripheral clocks.
    EDIS;
}
#endif  // F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM

#if F2838xX_CPU1_CM || F2838xD_CPU1_CPU2_CM
void easyDSP_SendWordToCM(uint16_t wordData)
{
    Cpu1toCmIpcRegs.CPU1TOCMIPCREPLY = wordData;
    //Cpu1toCmIpcRegs.CMTOCPU1IPCREPLY = wordData;

    // simulating IPC_sync(IPC_CPU1_L_CM_R, CPU1_CM_COMM_FLAG)
    Cpu1toCmIpcRegs.CPU1TOCMIPCSET.all = CPU1_CM_COMM_FLAG;
    while((Cpu1toCmIpcRegs.CMTOCPU1IPCSTS.all & CPU1_CM_COMM_FLAG) == 0);
    Cpu1toCmIpcRegs.CPU1TOCMIPCACK.all = CPU1_CM_COMM_FLAG;
    while((Cpu1toCmIpcRegs.CPU1TOCMIPCFLG.all & CPU1_CM_COMM_FLAG) != 0);

    Cpu1toCmIpcRegs.CPU1TOCMIPCSET.all = CPU1_CM_COMM_FLAG;
    while((Cpu1toCmIpcRegs.CMTOCPU1IPCSTS.all & CPU1_CM_COMM_FLAG) == 0);
    Cpu1toCmIpcRegs.CPU1TOCMIPCACK.all = CPU1_CM_COMM_FLAG;
    while((Cpu1toCmIpcRegs.CPU1TOCMIPCFLG.all & CPU1_CM_COMM_FLAG) != 0);
}

void easyDSP_SendAppToCM(void)
{
    struct HEADER {
        uint32_t DestAddr;
        uint16_t BlockSize;
    } BlockHeader;
    uint16_t wordData;
    uint16_t i;

    BlockHeader.BlockSize = easyDSP_GetWordData();
    BlockHeader.BlockSize /= 2; // byte to word
    easyDSP_SendWordToCM(BlockHeader.BlockSize);

    while(BlockHeader.BlockSize != 0)
    {
        BlockHeader.DestAddr = easyDSP_GetLongData();
        easyDSP_SendWordToCM((uint16_t)(BlockHeader.DestAddr >> 16));   // MSW
        easyDSP_SendWordToCM((uint16_t)BlockHeader.DestAddr);           // LSW

        for(i = 1; i <= BlockHeader.BlockSize; i++) {
            wordData = easyDSP_GetWordData();
            easyDSP_SendWordToCM(wordData);
        }
        BlockHeader.BlockSize = easyDSP_GetWordData();
        BlockHeader.BlockSize /= 2;
        easyDSP_SendWordToCM(BlockHeader.BlockSize);
    }
}

bool ezDSP_bIsCMResetBeforeBooting;
void easyDSP_SCIBootCM(void)
{
    uint16_t i;

    ///////////////////////////////////////////
    // SCI-A to CPU1 : SCIRXDA = GPIO 28, SCITXDA = GPIO29
    ///////////////////////////////////////////
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

    // clear message ram for test purpose
    //for(i = 0; i < (1000/2); i++) HWREG(CPUXTOCMMSGRAM1_BASE + i) = 0;  // 1000x16bit = 500x32bit.

    // copy SCI booting agent code to CPU1TOCMMSGRAM1
    easyDSP_AutoBaud();
    if(easyDSP_GetWordData() != 0x08AA) while(1);   // KeyValue
    for(i = 0; i < 8; i++) easyDSP_GetWordData();   // 8 reserved words
    easyDSP_GetWordData();                          // entry point
    easyDSP_GetWordData();                          // entry point
    easyDSP_CopyData(3);

    // secure the last echo
    DELAY_US(1000*1);

    // CM should be in reset
    ezDSP_bIsCMResetBeforeBooting = (CmConfRegs.CMRESCTL.bit.RESETSTS == 0);

    // Boot CM /w agent: copy max. size (1000 word)
    ezDSP_Device_bootCM(BOOTMODE_IPC_MSGRAM_COPY_LENGTH_1000W | BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_S0RAM);
    DELAY_US(1000*1);    // wait CM boot
    ezDSP_u32CMBootStatus = Cpu1toCmIpcRegs.CMTOCPU1IPCBOOTSTS;

    // download target CM code via SCI-A. This is done by agent program in CM
    if(easyDSP_GetWordData() != 0x08AA) while(1);   // KeyValue
    for(i = 0; i < 8; i++) easyDSP_GetWordData();   // 8 reserved words
    easyDSP_SendWordToCM(easyDSP_GetWordData());    // entry point
    easyDSP_SendWordToCM(easyDSP_GetWordData());    // entry point
    easyDSP_SendAppToCM();
    // secure the last echo byte
    DELAY_US(1000*1);

    // Synchronize both the cores
    Cpu1toCmIpcRegs.CPU1TOCMIPCSET.all = CPU1_CM_SYNC_FLAG;
    while((Cpu1toCmIpcRegs.CMTOCPU1IPCSTS.all & CPU1_CM_SYNC_FLAG) == 0);
    Cpu1toCmIpcRegs.CPU1TOCMIPCACK.all = CPU1_CM_SYNC_FLAG;
    while((Cpu1toCmIpcRegs.CPU1TOCMIPCFLG.all & CPU1_CM_SYNC_FLAG) != 0);
}
#endif  // F2838xX_CPU1_CM || F2838xD_CPU1_CPU2_CM
#endif  // CPU1

#ifdef CPU2
#if F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
// easyDSP_SCIBootCPU2 for F2838xD CPU2
void easyDSP_SCIBootCPU2(void)
{
    // booting echo feedback still ongoing even this moment
    // So no switch SCIA channel to CPU1 immediately
    DELAY_US(1000*10);

    // set IPC5 to inform to CPU1 that booting is done
    IPCLtoRFlagSet(CPU1_CPU2_SYNC_FLAG);          // Cpu2toCpu1IpcRegs.CPU2TOCPU1IPCSET.all |= (f)
    while(IPCLtoRFlagBusy(CPU1_CPU2_SYNC_FLAG));  // (((Cpu2toCpu1IpcRegs.CPU2TOCPU1IPCFLG.all & (f)) == 0) ? 0 : 1)

    asm(" NOP");
    asm(" NOP");

    // Wait until CPU1 is configuring SCI-B for CPU2 in easyDSP_SCI_Init()
    //DELAY_US(1000*10);
}
#endif  // F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM
#endif  // CPU2
#endif // !_FLASH
#endif  // F2838xS_CPU1_CM || F2838xD_CPU1_CM || F2838xD_CPU1_CPU2 || F2838xD_CPU1_CPU2_CM

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// F28Px multi core boot operation
// note : you can modify flash sector location if sector 0 is not preferred
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#if  F28P65xD_CPU1_CPU2
#if !defined(_FLASH)
#define CPU1_CPU2_SYNC_FLAG         IPC_FLAG5
void easyDSP_28p65x_Sci_Boot_Sync(void)
{
    easyDSP_SCIBootCPU2();
#ifdef CPU2
    DELAY_US(1000);    // Wait until CPU1 is configuring SCI-B for CPU2 in easyDSP_SCI_Init()
#endif
}
#endif

#ifdef CPU1
//#define BOOT_KEY                                0x5A000000U
//#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_1000W   0xA0000U
//#define BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM  0x01            // suspecting bug of boot rom. It should be 0x0CU
//#define CPU1TOCPU2MSGRAM0_BASE                  0x0003A000U    // CPU1 to CPU2 MSGRAM0
#define BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR0         0x03U
#define BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR127_END   0x23U
#define BOOTMODE_BOOT_TO_FLASH_BANK1_SECTOR0         0x43U
#define BOOTMODE_BOOT_TO_FLASH_BANK2_SECTOR0         0x63U
#define BOOTMODE_BOOT_TO_FLASH_BANK3_SECTOR0         0x83U
#define BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR0         0xA3U
#define BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR127_END   0xC3U
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK0_SECTOR0  0x0AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK1_SECTOR0  0x4AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK2_SECTOR0  0x6AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK3_SECTOR0  0x8AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK4_SECTOR0  0xAAU
//#include "f28p65x_ipc_defines.h"
void ezDSP_Device_bootCPU2(uint32_t u32BootMode)
{
    // Configure the CPU1TOCPU2IPCBOOTMODE register
    Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCBOOTMODE = (0x5A000000U | u32BootMode); // 0x5A000000U = book key
    // Set IPC Flag 0
    IPCLtoRFlagSet(IPC_FLAG0);
    // Bring CPU2 out of reset. Wait for CPU2 to go out of reset.
    EALLOW;
    DevCfgRegs.CPU2RESCTL.all = 0xA5A50000;
    EDIS;
    while(DevCfgRegs.RSTSTAT.bit.CPU2RES == 0);
}
#endif  // CPU1

void easyDSP_Boot_Sync(void)
{
#ifdef _FLASH
#ifdef CPU1
    ezDSP_Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_BANK3_SECTOR0);
    //ezDSP_Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_BANK1_SECTOR0);        // example for the other option for CPU2
    //ezDSP_Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_BANK2_SECTOR0);
#endif
#else // not flash
    easyDSP_28p65x_Sci_Boot_Sync();
#endif // flash
}

#if defined(CPU1) && !defined(_FLASH)
uint32_t ezDSP_u32CPU2BlockWordSize = 0;  // only for monitoring
uint32_t ezDSP_u32CPU2BootStatus = 0;
uint16_t easyDSP_GetWordData(void)
{
    uint16_t wordData, byteData;
    wordData = byteData = 0x0000;

    // Fetch the LSB and verify back to the host
    while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
    wordData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;
    SciaRegs.SCITXBUF.bit.TXDT = wordData;

    // Fetch the MSB and verify back to the host
    while(SciaRegs.SCIRXST.bit.RXRDY != 1) { }
    byteData =  (Uint16)SciaRegs.SCIRXBUF.bit.SAR;
    SciaRegs.SCITXBUF.bit.TXDT = byteData;

    // form the wordData from the MSB:LSB
    wordData |= (byteData << 8);

    return wordData;
}

uint32_t easyDSP_GetLongData(void)
{
    uint32_t longData;
    longData = ( (uint32_t)easyDSP_GetWordData() << 16);    // Fetch the upper 1/2 of the 32-bit value
    longData |= (uint32_t)easyDSP_GetWordData();            // Fetch the lower 1/2 of the 32-bit value
    return longData;
}

void easyDSP_CopyData(void)
{
    struct HEADER {
        uint32_t DestAddr;
        uint16_t BlockSize;
    } BlockHeader;

    uint16_t wordData;
    uint16_t i;

   BlockHeader.BlockSize = easyDSP_GetWordData();

   // monitoring
   ezDSP_u32CPU2BlockWordSize    += BlockHeader.BlockSize;

   while(BlockHeader.BlockSize != 0)
   {
      BlockHeader.DestAddr = easyDSP_GetLongData();
      BlockHeader.DestAddr -= 0x400;
      BlockHeader.DestAddr += 0x0003A000U;  // CPU1 to CPU2 MSGRAM0


      for(i = 0; i < BlockHeader.BlockSize; i++) {
          wordData = easyDSP_GetWordData();
          *(uint16_t *)BlockHeader.DestAddr = wordData;
          BlockHeader.DestAddr++;
      }

      BlockHeader.BlockSize = easyDSP_GetWordData();

      // monitoring
      ezDSP_u32CPU2BlockWordSize += BlockHeader.BlockSize;
   }
}

// auto baud implementation
void easyDSP_AutoBaud(void)
{
    uint16_t byteData;

    /////////////////////////////////////////////////////////////////////////////
    // Initialize the SCI-A port for communications with the host.
    /////////////////////////////////////////////////////////////////////////////
    // Enable the SCI-A clocks
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    ClkCfgRegs.LOSPCP.all = 0x0002;
    SciaRegs.SCIFFTX.all = 0x8000;
    SciaRegs.SCICCR.all = 0x0007;
    SciaRegs.SCICTL1.all = 0x0003;
    SciaRegs.SCICTL2.all = 0x0000;
    SciaRegs.SCICTL1.all = 0x0023;
    EDIS;

    /////////////////////////////////////////////////////////////////////////////
    // Perform autobaud lock with the host.
    /////////////////////////////////////////////////////////////////////////////
    SciaRegs.SCILBAUD.bit.BAUD = 1;
    SciaRegs.SCIFFCT.bit.CDC = 1;
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;
    while(SciaRegs.SCIFFCT.bit.ABD != 1);
    SciaRegs.SCIFFCT.bit.ABDCLR = 1;
    SciaRegs.SCIFFCT.bit.CDC = 0;
    while(SciaRegs.SCIRXST.bit.RXRDY != 1);
    byteData = SciaRegs.SCIRXBUF.bit.SAR;
    SciaRegs.SCITXBUF.bit.TXDT = byteData;
}

void easyDSP_SCIBootCPU2(void)
{
    ///////////////////////////////////////////
    // SCI-A to CPU1 : SCIRXDA = GPIO 13, SCITXDA = GPIO 12
    ///////////////////////////////////////////
    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(13, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_ASYNC);

    // copy SCI booting agent code to CPU1TOCPU2MSGRAM1
    uint16_t i;
    easyDSP_AutoBaud();
    if(easyDSP_GetWordData() != 0x08AA) while(1);   // KeyValue
    for(i = 0; i < 8; i++) easyDSP_GetWordData();   // 8 reserved words
    easyDSP_GetWordData();                          // entry point
    easyDSP_GetWordData();                          // entry point
    easyDSP_CopyData();

    // secure the last echo byte
    DELAY_US(1000*10);

    // SCI-A assigned to CPU2
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1;       // This register must be configured prior to enabling the peripheral clocks.
    EDIS;

    ///////////////////////////////////////////
    // SCI-A to CPU2 : SCIRXDA = GPIO 13, SCITXDA = GPIO 12
    ///////////////////////////////////////////
    GPIO_SetupPinMux(13, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinMux(12, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(13, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_ASYNC);

    // Boot CPU2 /w agent. max. (1000) copy
    Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCBOOTMODE = 0x5A0A0001;
    IPCLtoRFlagSet(IPC_FLAG0);  // Set IPC Flag 0
    // Bring CPU2 out of reset. Wait for CPU2 to go out of reset.
    EALLOW;
    DevCfgRegs.CPU2RESCTL.all = 0xA5A50000;
    EDIS;
    while(DevCfgRegs.RSTSTAT.bit.CPU2RES == 0);

    DELAY_US(1000*1);    // wait CPU2 boot
    ezDSP_u32CPU2BootStatus = Cpu1toCpu2IpcRegs.CPU2TOCPU1IPCBOOTSTS;

#if 0
    // if invalid boot mode due to boot rom bug fix, try with 0x5A0A000C
    if(ezDSP_u32CPU2BootStatus | 0x00040000) {
        Cpu1toCpu2IpcRegs.CPU1TOCPU2IPCBOOTMODE = 0x5A0A000C;
        IPCLtoRFlagSet(IPC_FLAG0);  // Set IPC Flag 0
        // Bring CPU2 out of reset. Wait for CPU2 to go out of reset.
        EALLOW;
        DevCfgRegs.CPU2RESCTL.all = 0xA5A50000;
        EDIS;
        while(DevCfgRegs.RSTSTAT.bit.CPU2RES == 0);

        DELAY_US(1000*1);    // wait CPU2 boot
        ezDSP_u32CPU2BootStatus = Cpu1toCpu2IpcRegs.CPU2TOCPU1IPCBOOTSTS;
    }
#endif

    ///////////////////////////////////////////////////////////////////////////////////
    // download target CPU2 code via SCI-A. This is done by agent program in CPU2
    ///////////////////////////////////////////////////////////////////////////////////
    //wait until CPU2 is booted and set CPU1_CPU2_SYNC_FLAG
    // Synchronize both the cores
    while(!IPCRtoLFlagBusy(CPU1_CPU2_SYNC_FLAG));
    IPCRtoLFlagAcknowledge(CPU1_CPU2_SYNC_FLAG);

    // SCI-A assigned to CPU1
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;       // SCI-A CLOCK Disabled
    DevCfgRegs.CPUSEL5.bit.SCI_A = 0;       // This register must be configured prior to enabling the peripheral clocks.
    EDIS;
}
#endif  // defined(CPU1) && !defined(_FLASH)

#if defined(CPU2) && !defined(_FLASH)
// easyDSP_SCIBootCPU2 for CPU2
void easyDSP_SCIBootCPU2(void)
{
    // booting echo feedback still ongoing even this moment
    // So no switch SCIA channel to CPU1 immediately
    DELAY_US(1000*10);

    // set IPC5 to inform to CPU1 that booting is done
    IPCLtoRFlagSet(CPU1_CPU2_SYNC_FLAG);          // Cpu2toCpu1IpcRegs.CPU2TOCPU1IPCSET.all |= (f)
    while(IPCLtoRFlagBusy(CPU1_CPU2_SYNC_FLAG));  // (((Cpu2toCpu1IpcRegs.CPU2TOCPU1IPCFLG.all & (f)) == 0) ? 0 : 1)

    asm(" NOP");
    asm(" NOP");

#if 0
    DELAY_US(1000*1);
    // wait until CPU1 set the memory configuration
    while(!IPCRtoLFlagBusy(CPU1_CPU2_SYNC_FLAG));
    IPCRtoLFlagAcknowledge(CPU1_CPU2_SYNC_FLAG);
#endif

}
#endif  // defined(CPU2) && !defined(_FLASH)
#endif  // F28P65xD_CPU1_CPU2




/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// easyDSP communication
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// easyDSP commands & states
#define STAT_INIT       0
#define STAT_ADDR       1
#define STAT_DATA2B     2
#define STAT_DATA4B     3
#define STAT_WRITE      4
#define STAT_DATA8B     5
#define STAT_CONFIRM    6
#define STAT_DATA_BLOCK 7

#define CMD_ADDR            0xE7
#define CMD_ADDR_32BIT      0x41    // 32bit address from v10.1
#define CMD_READ2B          0xDB
#define CMD_READ4B          0xC3
#define CMD_READ8B          0x8B
#define CMD_READ16B         0x28
#define CMD_DATA2B          0xBD
#define CMD_DATA4B          0x99
#define CMD_DATA8B          0x64
#define CMD_DATA_BLOCK      0x55
#define CMD_WRITE           0x7E
#define CMD_FB_READ         0x0D
#define CMD_FB_WRITE_OK     0x0D
#define CMD_FB_WRITE_NG     0x3C
#define CMD_CHANGE_CPU      0X5A
#define CMD_CONFIRM         0xA5

// error counter
unsigned int ezDSP_uBRKDTCount = 0, ezDSP_uFECount = 0, ezDSP_uOECount = 0, ezDSP_uPECount = 0;
unsigned int ezDSP_uWrongISRCount = 0;

// for easyDSP
unsigned char ezDSP_ucRx = 0;
unsigned int ezDSP_uState = STAT_INIT, ezDSP_uData = 0, ezDSP_uChksum = 0;
unsigned long ezDSP_ulData = 0, ezDSP_ulAddr = 0;
unsigned int ezDSP_uAddrRdCnt = 0, ezDSP_uDataRdCnt = 0;
unsigned long long ezDSP_ullData = 0;
unsigned int ezDSP_uBlockSize = 0, ezDSP_uBlockIndex = 0, ezDSP_uChkSumCalculated = 0;
unsigned int ezDSP_uISRRxCount = 0, ezDSP_uISRTxCount = 0;
unsigned int ezDSP_uMaxRxFifoCnt = 0, ezDSP_uTxFifoCnt = 0, ezDSP_uMaxTxFifoCnt = 0;
unsigned int ezDSP_uTxFifoFullCnt = 0;      // something wrong if not zero

static inline void easy_addRing(unsigned char y) {
    if(ScixRegs.SCIFFTX.bit.TXFFST != 16)     ScixRegs.SCITXBUF.bit.TXDT = y;
    else                                      ezDSP_uTxFifoFullCnt++;

    // counting after input to Tx
    ezDSP_uTxFifoCnt = ScixRegs.SCIFFTX.bit.TXFFST;
    if(ezDSP_uTxFifoCnt > ezDSP_uMaxTxFifoCnt) ezDSP_uMaxTxFifoCnt = ezDSP_uTxFifoCnt;
}

__interrupt void easy_RXINT_ISR()
{
    INT_NESTING_START;

    ezDSP_uISRRxCount++;
    Uint16 uIndex = 0;

    // check RX Error
    if(ScixRegs.SCIRXST.bit.RXERROR) {
        if(ScixRegs.SCIRXST.bit.BRKDT)  ezDSP_uBRKDTCount++;    // Break Down
        if(ScixRegs.SCIRXST.bit.FE)     ezDSP_uFECount++;       // FE
        if(ScixRegs.SCIRXST.bit.OE)     ezDSP_uOECount++;       // OE
        if(ScixRegs.SCIRXST.bit.PE)     ezDSP_uPECount++;       // PE

        // 'Break down' stops further Rx operation.
        // software reset is necessary to clear its status bit and proceed further rx operation
        ScixRegs.SCICTL1.bit.SWRESET = 0;
        ScixRegs.SCICTL1.bit.SWRESET = 1;

        //FIFO
        ScixRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   // Clear Overflow flag
        ScixRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag

        INT_NESTING_END;
        return;
    }

    if(!ScixRegs.SCIFFRX.bit.RXFFST) {
        ezDSP_uWrongISRCount++;
        ScixRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   // Clear Overflow flag
        ScixRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
        INT_NESTING_END;
        return;
    }

    // monitoring
    if(ScixRegs.SCIFFRX.bit.RXFFST > ezDSP_uMaxRxFifoCnt) ezDSP_uMaxRxFifoCnt = ScixRegs.SCIFFRX.bit.RXFFST;

    // FIFO
    while(ScixRegs.SCIFFRX.bit.RXFFST) {
        ezDSP_ucRx = ScixRegs.SCIRXBUF.all;

        ////////////////////////////////////////////
        // Parsing by state
        ////////////////////////////////////////////
        if(ezDSP_uState == STAT_INIT) {
            if(ezDSP_ucRx == CMD_ADDR || ezDSP_ucRx == CMD_ADDR_32BIT) {
                ezDSP_uState = STAT_ADDR;
                ezDSP_uAddrRdCnt = 0;
                ezDSP_b32bitAddress = (ezDSP_ucRx == CMD_ADDR_32BIT);
            }
            else if(ezDSP_ucRx == CMD_READ2B) {
                ezDSP_ulAddr++; // auto increment
                ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;

                easy_addRing(ezDSP_uData >> 8);  // MSB
                easy_addRing(ezDSP_uData);       // LSB
                easy_addRing(CMD_FB_READ);

                ezDSP_uState = STAT_INIT;
            }
            else if(ezDSP_ucRx == CMD_READ16B) {
                ezDSP_ulAddr += 8;
                for(uIndex = 0; uIndex < 8; uIndex++) {
                    // Since this is not for variable, addresss is increased sequentially
                    ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + uIndex);
                    easy_addRing(ezDSP_uData >> 8);      // MSB
                    easy_addRing(ezDSP_uData);           // LSB
                }
                easy_addRing(CMD_FB_READ);

                ezDSP_uState = STAT_INIT;
            }
            else if(ezDSP_ucRx == CMD_DATA2B) {
                ezDSP_ulAddr++; // auto increment

                ezDSP_uState = STAT_DATA2B;
                ezDSP_uDataRdCnt = 0;
            }
            else if(ezDSP_ucRx == CMD_DATA4B) {
                ezDSP_ulAddr += 2;  // auto increment

                ezDSP_uState = STAT_DATA4B;
                ezDSP_uDataRdCnt = 0;
            }
            // v8.5
            else if(ezDSP_ucRx == CMD_DATA8B) {
                ezDSP_ulAddr += 4;  // auto increment

                ezDSP_uState = STAT_DATA8B;
                ezDSP_uDataRdCnt = 0;
            }
        }
        else if(ezDSP_uState == STAT_ADDR) {
            ezDSP_uAddrRdCnt++;
            if(ezDSP_uAddrRdCnt == 1) {
                ezDSP_ulAddr = ezDSP_ucRx;          // MSB
            }
            else if(ezDSP_uAddrRdCnt == 2 || ezDSP_uAddrRdCnt == 3 || (ezDSP_b32bitAddress && ezDSP_uAddrRdCnt == 4 )) {
                ezDSP_ulAddr <<= 8;
                ezDSP_ulAddr |= ezDSP_ucRx;
           }
            else if((!ezDSP_b32bitAddress && ezDSP_uAddrRdCnt == 4) || (ezDSP_b32bitAddress && ezDSP_uAddrRdCnt == 5)) {
                if(ezDSP_ucRx == CMD_READ2B) {
                    ezDSP_uData = *(unsigned int*)ezDSP_ulAddr;

                    easy_addRing(ezDSP_uData >> 8);      // MSB
                    easy_addRing(ezDSP_uData);           // LSB
                    easy_addRing(CMD_FB_READ);

                    ezDSP_uState = STAT_INIT;
                }
                else if(ezDSP_ucRx == CMD_READ4B) { // modified in v9.4
                    ezDSP_ulData = *(unsigned long *)ezDSP_ulAddr;
                    easy_addRing(ezDSP_ulData >> 24);  // MSB
                    easy_addRing(ezDSP_ulData >> 16);
                    easy_addRing(ezDSP_ulData >> 8);
                    easy_addRing(ezDSP_ulData);        // LSB
                    easy_addRing(CMD_FB_READ);
                    ezDSP_uState = STAT_INIT;
                }
                else if(ezDSP_ucRx == CMD_READ8B) { // modified in v9.4
                    ezDSP_ullData = *(unsigned long long *)ezDSP_ulAddr;
                    easy_addRing(ezDSP_ullData >> (8*7));   // MSB
                    easy_addRing(ezDSP_ullData >> (8*6));
                    easy_addRing(ezDSP_ullData >> (8*5));
                    easy_addRing(ezDSP_ullData >> (8*4));
                    easy_addRing(ezDSP_ullData >> (8*3));
                    easy_addRing(ezDSP_ullData >> (8*2));
                    easy_addRing(ezDSP_ullData >> (8*1));
                    easy_addRing(ezDSP_ullData);            // LSB
                    easy_addRing(CMD_FB_READ);
                    ezDSP_uState = STAT_INIT;
                }
                else if(ezDSP_ucRx == CMD_READ16B) {
                    for(uIndex = 0; uIndex < 8; uIndex++) {
                        // Since this is not for variable, address is increased sequentially
                        ezDSP_uData = *(unsigned int*)(ezDSP_ulAddr + uIndex);
                        easy_addRing(ezDSP_uData >> 8);      // MSB
                        easy_addRing(ezDSP_uData);           // LSB
                    }
                    easy_addRing(CMD_FB_READ);
                    ezDSP_uState = STAT_INIT;
                }
                else if(ezDSP_ucRx == CMD_DATA2B) {
                    ezDSP_uState = STAT_DATA2B;
                    ezDSP_uDataRdCnt = 0;
                }
                else if(ezDSP_ucRx == CMD_DATA4B) {
                    ezDSP_uState = STAT_DATA4B;
                    ezDSP_uDataRdCnt = 0;
                }
                else if(ezDSP_ucRx == CMD_DATA8B) {
                    ezDSP_uState = STAT_DATA8B;
                    ezDSP_uDataRdCnt = 0;
                }
                else ezDSP_uState = STAT_INIT;
            }
            else
                ezDSP_uState = STAT_INIT;
        }
        else if(ezDSP_uState == STAT_DATA2B) {
            ezDSP_uDataRdCnt++;
            if(ezDSP_uDataRdCnt == 1)       ezDSP_uData = ezDSP_ucRx << 8;      // MSB
            else if(ezDSP_uDataRdCnt == 2)  ezDSP_uData |= ezDSP_ucRx;          // LSB
            else if(ezDSP_uDataRdCnt == 3)  ezDSP_uChksum = ezDSP_ucRx << 8;    // MSB
            else if(ezDSP_uDataRdCnt == 4)  ezDSP_uChksum |= ezDSP_ucRx;        // LSB
            else if(ezDSP_uDataRdCnt == 5) {
                if(ezDSP_ucRx == CMD_WRITE) {
                    if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_uData) & 0xFFFF)) {
                        *(unsigned int*)ezDSP_ulAddr = ezDSP_uData;
                        easy_addRing(CMD_FB_WRITE_OK);
                     }
                    else {
                        easy_addRing(CMD_FB_WRITE_NG);
                    }
                }
                ezDSP_uState = STAT_INIT;
            }
        }
        else if(ezDSP_uState == STAT_DATA4B) {
            ezDSP_uDataRdCnt++;
            if(ezDSP_uDataRdCnt == 1) {
                ezDSP_ulData = ezDSP_ucRx;      // MSB
                ezDSP_ulData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 2) {
                ezDSP_ulData |= ezDSP_ucRx;
                ezDSP_ulData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 3) {
                ezDSP_ulData |= ezDSP_ucRx;
                ezDSP_ulData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 4) {
                ezDSP_ulData |= ezDSP_ucRx;
            }
            else if(ezDSP_uDataRdCnt == 5)
                ezDSP_uChksum = ezDSP_ucRx << 8;    // MSB
            else if(ezDSP_uDataRdCnt == 6)
                ezDSP_uChksum |= ezDSP_ucRx;        // LSB
            else if(ezDSP_uDataRdCnt == 7) {
                if(ezDSP_ucRx == CMD_WRITE) {
                    if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_ulData) & 0xFFFF)) {
                        *(unsigned long*)ezDSP_ulAddr = ezDSP_ulData;
                        easy_addRing(CMD_FB_WRITE_OK);
                    }
                    else {
                        easy_addRing(CMD_FB_WRITE_NG);
                    }
                }
                ezDSP_uState = STAT_INIT;
            }
        }
        else if(ezDSP_uState == STAT_DATA8B) {
            ezDSP_uDataRdCnt++;
            if(ezDSP_uDataRdCnt == 1) {
                ezDSP_ullData = ezDSP_ucRx;         // MSB
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 2) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 3) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 4) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 5) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 6) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 7) {
                ezDSP_ullData |= ezDSP_ucRx;
                ezDSP_ullData <<= 8;
            }
            else if(ezDSP_uDataRdCnt == 8) {
                ezDSP_ullData |= ezDSP_ucRx;
            }
            else if(ezDSP_uDataRdCnt == 9)
                ezDSP_uChksum = ezDSP_ucRx << 8;    // MSB
            else if(ezDSP_uDataRdCnt == 10)
                ezDSP_uChksum |= ezDSP_ucRx;        // LSB
            else if(ezDSP_uDataRdCnt == 11) {
                if(ezDSP_ucRx == CMD_WRITE) {
                    if(ezDSP_uChksum == ((ezDSP_ulAddr + ezDSP_ullData) & 0xFFFF)) {
                        *(unsigned long long*)ezDSP_ulAddr = ezDSP_ullData;
                        easy_addRing(CMD_FB_WRITE_OK);
                    }
                    else {
                        easy_addRing(CMD_FB_WRITE_NG);
                    }
                }
                ezDSP_uState = STAT_INIT;
            }
        }
	
        else
            ezDSP_uState = STAT_INIT;
    }

    ScixRegs.SCIFFRX.bit.RXFFOVRCLR = 1;   // Clear Overflow flag
    ScixRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // Clear Interrupt flag
    INT_NESTING_END;
}

