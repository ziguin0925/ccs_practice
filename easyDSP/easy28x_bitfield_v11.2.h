/***************************************************************
    easy28x_bitfield.h
    v9.1 (Mar 2020) : First release based C2000Ware_2_01_00_00
    v9.2 (Apr 2020) : 1. supports F28002x
                      2. enables pull-up of 28004x Rx pins for better noise immunity
                      3. MCU name change : F2837xD -> F2837xD_CPU1_CPU2, F2837xD_CPU1_ONLY -> F2837xD_CPU1
					  4. F2832x, F2833x and C2834x are moved to easy28x_gen2_bitfield.h
    v9.3 (May 2020) : supports F2838xS and F2838xD
	v9.4 (Sep 2020) : supports one time reading for 4B/8B data
	v9.5 (Dec 2020) : 1. reduce uninitialized variables
                      2. CPU2 RAM booting for F2837xD and F2838xD
    v10.1 (Nov 2021) : 1. F28003x support
                       2. support 32bit address width
	v10.6 (Nov 2022) : 1. F280013x support
                       2. inline -> static inline for AddRing()
    v10.8 (Mar 2023) : 1. F280015x support
                       2. name changed from AddRing() to easy_addRing()
    v11.0 (Sep 2023) :  1. F28P65x support
                        2. F2837x/F2838x multi core RAM booting : GSxRAM ownership is not changed in easyDSP_Boot_Sync()
    v11.2 (May 2024) :  1. F28P55xS support
****************************************************************/
#ifndef _EASY28X_BITFIELD_H__
#define _EASY28X_BITFIELD_H__

extern void	easyDSP_SCI_Init(void);
extern __interrupt void easy_RXINT_ISR(void);
extern void easyDSP_SPI_Flashrom_Init(void);        // only for C2834x
extern void easyDSP_Boot_Sync(void);


// internal function declaration
static inline void easy_addRing(unsigned char y);

/////////////////////////////////////////////////////////////////////////////////////////
// NOTICE : Please select or modify below MCU type, CPU_CLK, LSP_CLK, BAUDRATE,
//          interrupt nesting, ram function activation according to your target system
///////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////
// Select target MCU series : Define 1 to target MCU. 0 to all others.
/////////////////////////////////////////////////////////////////////////////////////////////
#define F28001x                 0       // 280013x and 280015x
#define F28002x                 0
#define F28003x                 0
#define F28004x                 1
#define F28P55xS                0
#define F28P65xS                0
#define F28P65xD_CPU1           0       // use CPU1 only
#define F28P65xD_CPU1_CPU2      0       // use CPU1 and CPU2
#define F2807x                  0
#define F2837xS                 0
#define F2837xD_CPU1            0       // use CPU1 only. name changed from F2837xD_CPU1_ONLY
#define F2837xD_CPU1_CPU2       0       // use CPU1 and CPU2. name changed from F2837xD
#define F2838xS_CPU1            0       // use CPU1 only
#define F2838xS_CPU1_CM         0       // use CPU1 and CM
#define F2838xD_CPU1            0       // use CPU1 only
#define F2838xD_CPU1_CPU2       0       // use CPU1 and CPU2
#define F2838xD_CPU1_CM         0       // use CPU1 and CM
#define F2838xD_CPU1_CPU2_CM    0       // use CPU1 and CPU2 and CM

/////////////////////////////////////////////////////////////////////////////////////////////
// Select SYSCLK and LSPCLK
/////////////////////////////////////////////////////////////////////////////////////////////
#define CPU_CLK           100000000L      // 100MHz such as 28002x/4x
//#define CPU_CLK             120000000L      // 120MHz such as 2807x, 28003x, 28001x
//#define CPU_CLK           150000000L      // 150MHz such as 2837xS/2837xD INTOSC2, 28P55x
//#define CPU_CLK           200000000L      // 200MHz such as 2837x, 28P65x and F2838x

////////////////////////////////////////////////////////////////////////////////////////////
// Select LSPCLK
/////////////////////////////////////////////////////////////////////////////////////////////
#define LSP_CLK             (CPU_CLK/4)     // NOTE : LSP_CLK should be same to CPU_CLK in MotorWare¢â


/////////////////////////////////////////////////////////////////////////////////////////////
// Select baudrate of easyDSP communication.
// It should be same to baudrate of easyDSP project setting.
/////////////////////////////////////////////////////////////////////////////////////////////
//#define BAUDRATE            9600L
//#define BAUDRATE            19200L
#define BAUDRATE            38400L
//#define BAUDRATE            57600L
//#define BAUDRATE            86400L
//#define BAUDRATE            115200L
//#define BAUDRATE            153600L
//#define BAUDRATE            192000L
//#define BAUDRATE            230400L

////////////////////////////////////////////////////////////////////////////////////////////
// interrupt nesting assuming easyDSP ISR has the lowest priority.
// If not, please change the code accordingly
////////////////////////////////////////////////////////////////////////////////////////////
#define INT_NESTING_START   {           \
    IER |= 0x0100;                      \
    PieCtrlRegs.PIEACK.all = 0xFFFF;    \
    asm("       NOP");                  \
    EINT;                               \
    }
#define INT_NESTING_END         DINT
//#define INT_NESTING_START                                                  // reserved for no nesting
//#define INT_NESTING_END       (PieCtrlRegs.PIEACK.all = PIEACK_GROUP9)     // reserved for no nesting

////////////////////////////////////////////////////////////////////////////////////////////
// if _FLASH is not predefined by CCS configuration, you can do it here
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _FLASH
//#define _FLASH
#endif

////////////////////////////////////////////////////////////////////////////////////////////
// use #pragma if fast SCI ISR run on the ram is required
////////////////////////////////////////////////////////////////////////////////////////////
#ifdef _FLASH
#pragma CODE_SECTION(easy_RXINT_ISR, ".TI.ramfunc");
#pragma CODE_SECTION(easy_addRing, ".TI.ramfunc");
#endif

#endif	// of _EASY28X_BITFIELD_H__
