
typedef struct
{
    bool                master_bit;
    uint16_t            phaseShift;
    EPWM_SignalParams   epwm_signal_params;
} EPWM_Custom;




#define PWM_ON          1
#define PWM_OFF         2

void ePWM_Force12_Trip(void);
void ePWM_Force34_Trip(void);
void PWM_PRI_EN_DI(int enable);
void PWM_SEC_EN_DI(int enable);
void ePWM_TZ12_Reset();
void ePWM_TZ34_Reset();


void pwm_init (void);
void CreateEPwm(uint16_t, EPWM_Custom);
void InitEPwm1Gpio(void);
void InitEPwm2Gpio(void);
void InitEPwm4Gpio(void);
void InitEPwm3Gpio(void);


__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);



static inline volatile uint16_t TBPRD_BASE  = 1000;
static inline volatile uint16_t DEAD_BAND_CUSTOM  = 20;


static inline void TBCLKSYNC_disable(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
}

static inline void TBCLKSYNC_enable(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

static inline void Init_EPwm_1()      //PWM f = TBCLK / (2 * TBPRD) = 100MHz / (2 * 1000) = 50kHz
{
    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1A to low
    EPwm1Regs.TZCTL.bit.TZB         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1B to low
    EPwm1Regs.TZFRC.bit.OST         = 1;                    // One shot trip active -> Clear PWM
    EDIS;

    EPwm1Regs.TBPRD                 =   TBPRD_BASE;      // Set timer period, PRD = 1000
    EPwm1Regs.TBCTR                 =   0x0000;             // Clear counter
    EPwm1Regs.TBPHS.bit.TBPHS       =   0;             // Clear phase

    //TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) = SYSCLK = 100MHz
    EPwm1Regs.TBCTL.bit.HSPCLKDIV   =   TB_DIV1;            // divider = /1
    EPwm1Regs.TBCTL.bit.CLKDIV      =   TB_DIV1;            // divider = /1
    EPwm1Regs.TBCTL.bit.CTRMODE     =   TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.PHSEN       =   TB_DISABLE;          // Phase Loading 활성화 -> 다른 PWM 모듈과 동기화 시 필요
    EPwm1Regs.TBCTL.bit.PRDLD       =   TB_SHADOW;          // 주기(TBPRD) 값을 shadow 레지스터에서 읽어옴 (카운터 = 0 일 때)
    EPwm1Regs.TBCTL.bit.SYNCOSEL    =   TB_CTR_ZERO;        // 다른 PMW 모듈과 동기화 위해, 카운터 0일 때 Sync out 신호 발생 -> Master

    EPwm1Regs.CMPCTL.bit.SHDWAMODE  =   CC_SHADOW;          // CMPA Shadow mode Enable
    EPwm1Regs.CMPCTL.bit.SHDWBMODE  =   CC_SHADOW;          // CMPB Shadow mode Enable
    EPwm1Regs.CMPCTL.bit.LOADAMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)
    EPwm1Regs.CMPCTL.bit.LOADBMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)

    EPwm1Regs.AQCTLA.bit.CAU        =   AQ_CLEAR;           // Action when counter = CMPA on up-count is Clear
    EPwm1Regs.AQCTLA.bit.CAD        =   AQ_SET;             // Action when counter = CMPA on down-count is Set

    EPwm1Regs.CMPA.bit.CMPA         =   (0.5)*EPwm1Regs.TBPRD;             //Clear Compare Regs

    EPwm1Regs.DBCTL.bit.OUT_MODE    =   DB_FULL_ENABLE;     // EPWMxA/B에 대해 상승(RED)/하강(FED) 에지 모두 데드 밴드 활성화
    EPwm1Regs.DBCTL.bit.POLSEL      =   DB_ACTV_HIC;        // EPWMxB는 EPWMxA의 반전(inverted) 신호로 출력 (상보적인 신호)
    EPwm1Regs.DBCTL.bit.IN_MODE     =   DBA_ALL;            // EPWMxA 신호를 데드 밴드 로직의 입력 소스로 사용
    EPwm1Regs.DBRED.bit.DBRED       =   DEAD_BAND_CUSTOM;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay
    EPwm1Regs.DBFED.bit.DBFED       =   DEAD_BAND_CUSTOM;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay

    EPwm1Regs.ETSEL.bit.SOCAEN      = 1;                    // SOCA 이벤트 생성 기능 활성화
    EPwm1Regs.ETSEL.bit.SOCASEL     = 2;                    // 트리거 시점: 카운터가 '주기값(TBPRD)'에 도달했을 때.
    EPwm1Regs.ETPS.bit.SOCAPRD      = 1;                    // Generate pulse on 1st event

    EPwm1Regs.ETSEL.bit.INTSEL      = ET_CTR_ZERO;          // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN       = 1;                    // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD       = ET_1ST;               // Generate INT on 1st event
}


static inline void Init_EPwm_2()      //PWM f = TBCLK / (2 * TBPRD) = 100MHz / (2 * 1000) = 50kHz
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
    EPwm2Regs.TBCTL.bit.PHSDIR      =   TB_UP;

    EPwm2Regs.TBCTL.bit.PRDLD       =   TB_SHADOW;          // 주기(TBPRD) 값을 shadow 레지스터에서 읽어옴 (카운터 = 0 일 때)
    EPwm2Regs.TBCTL.bit.SYNCOSEL    =   TB_SYNC_IN;        // 다른 PMW 모듈과 동기화 위해, 카운터 0일 때 Sync out 신호 발생 -> Master

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
    EPwm2Regs.DBRED.bit.DBRED       =   DEAD_BAND_CUSTOM;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay
    EPwm2Regs.DBFED.bit.DBFED       =   DEAD_BAND_CUSTOM;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay

    EPwm2Regs.ETSEL.bit.SOCAEN      = 1;                    // SOCA 이벤트 생성 기능 활성화
    EPwm2Regs.ETSEL.bit.SOCASEL     = 2;                    // 트리거 시점: 카운터가 '주기값(TBPRD)'에 도달했을 때.
    EPwm2Regs.ETPS.bit.SOCAPRD      = 1;                    // Generate pulse on 1st event

    EPwm2Regs.ETSEL.bit.INTSEL      = ET_CTR_ZERO;          // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN       = 1;                    // Enable INT
    EPwm2Regs.ETPS.bit.INTPRD       = ET_1ST;               // Generate INT on 1st event
}
           // EPWM1_SYNC_OUT


static inline void Init_EPwm_4()      //PWM f = TBCLK / (2 * TBPRD) = 100MHz / (2 * 1000) = 50kHz
{
    EALLOW;
    SyncSocRegs.SYNCSELECT.bit.EPWM4SYNCIN = 0;  // EPWM1_SYNC_OUT
    EPwm4Regs.TZCTL.bit.TZA         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1A to low
    EPwm4Regs.TZCTL.bit.TZB         = TZ_FORCE_LO;          // Trip zone on -> Force EPWM1B to low
    EPwm4Regs.TZFRC.bit.OST         = 1;                    // One shot trip active -> Clear PWM
    EDIS;

    EPwm4Regs.TBPRD                 =   TBPRD_BASE;      // Set timer period, PRD = 1000
    EPwm4Regs.TBCTR                 =   0x0000;             // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS       =   1000;             // Clear phase

    //TBCLK = SYSCLK / (HSPCLKDIV * CLKDIV) = SYSCLK = 100MHz
    EPwm4Regs.TBCTL.bit.HSPCLKDIV   =   TB_DIV1;            // divider = /1
    EPwm4Regs.TBCTL.bit.CLKDIV      =   TB_DIV1;            // divider = /1
    EPwm4Regs.TBCTL.bit.CTRMODE     =   TB_COUNT_UPDOWN;
    EPwm4Regs.TBCTL.bit.PHSEN       =   TB_ENABLE;          // Phase Loading 활성화 -> 다른 PWM 모듈과 동기화 시 필요
    EPwm4Regs.TBCTL.bit.PHSDIR      =   TB_UP;

    EPwm4Regs.TBCTL.bit.PRDLD       =   TB_SHADOW;          // 주기(TBPRD) 값을 shadow 레지스터에서 읽어옴 (카운터 = 0 일 때)
    EPwm4Regs.TBCTL.bit.SYNCOSEL    =   TB_SYNC_IN;        // 다른 PMW 모듈과 동기화 위해, 카운터 0일 때 Sync out 신호 발생 -> Master

    EPwm4Regs.CMPCTL.bit.SHDWAMODE  =   CC_SHADOW;          // CMPA Shadow mode Enable
    EPwm4Regs.CMPCTL.bit.SHDWBMODE  =   CC_SHADOW;          // CMPB Shadow mode Enable
    EPwm4Regs.CMPCTL.bit.LOADAMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)
    EPwm4Regs.CMPCTL.bit.LOADBMODE  =   CC_CTR_ZERO;        // Shadow -> Active 로딩 시점: 카운터가 0일 때 (CTR=ZERO)

    EPwm4Regs.AQCTLA.bit.CAU        =   AQ_CLEAR;           // Action when counter = CMPA on up-count is Clear
    EPwm4Regs.AQCTLA.bit.CAD        =   AQ_SET;             // Action when counter = CMPA on down-count is Set

    EPwm4Regs.CMPA.bit.CMPA         =   (0.5)*EPwm4Regs.TBPRD;             //Clear Compare Regs

    EPwm4Regs.DBCTL.bit.OUT_MODE    =   DB_FULL_ENABLE;     // EPWMxA/B에 대해 상승(RED)/하강(FED) 에지 모두 데드 밴드 활성화
    EPwm4Regs.DBCTL.bit.POLSEL      =   DB_ACTV_HIC;        // EPWMxB는 EPWMxA의 반전(inverted) 신호로 출력 (상보적인 신호)
    EPwm4Regs.DBCTL.bit.IN_MODE     =   DBA_ALL;            // EPWMxA 신호를 데드 밴드 로직의 입력 소스로 사용
    EPwm4Regs.DBRED.bit.DBRED       =   DEAD_BAND_CUSTOM;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay
    EPwm4Regs.DBFED.bit.DBFED       =   DEAD_BAND_CUSTOM;       // 상승 에지 데드 밴드 시간 (20 * TBCLK cycles) 1us Delay

    EPwm4Regs.ETSEL.bit.SOCAEN      = 1;                    // SOCA 이벤트 생성 기능 활성화
    EPwm4Regs.ETSEL.bit.SOCASEL     = 2;                    // 트리거 시점: 카운터가 '주기값(TBPRD)'에 도달했을 때.
    EPwm4Regs.ETPS.bit.SOCAPRD      = 1;                    // Generate pulse on 1st event

    EPwm4Regs.ETSEL.bit.INTSEL      = ET_CTR_ZERO;          // Select INT on Zero event
    EPwm4Regs.ETSEL.bit.INTEN       = 1;                    // Enable INT
    EPwm4Regs.ETPS.bit.INTPRD       = ET_1ST;               // Generate INT on 1st event
}
