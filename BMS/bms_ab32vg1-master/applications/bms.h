/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-29     yjm       the first version
 */
#ifndef APPLICATIONS_BMS_H_
#define APPLICATIONS_BMS_H_

#define TIMCYCLE 20                /* 定时器中断周期，单位毫秒 */
#define UARTTIMEOUTCNT 5           /* 定义串口中断超时计数，也就是连续20*5=100ms没有接收到串口数据，视为一帧数据完成 */

#define TOTAL_IC  1                /* !<number of ICs in the daisy chain */
#define DEFAULT_VOLT_LOW_LIMIT_WARNING   3000  /* 欠压值 */
#define DEFAULT_VOLT_HIGH_LIMIT_WARNING  4200  /* 过压值  */

/* 电池组的定义 */
#define CELLNUMONEBAT    12        /* 一个电池组中单体电芯的数量 */
#define TEMPNUMONEBAT    5         /* 一个电池组中温度传感器数量 */
#define BATTNUMONPACK    TOTAL_IC  /* 一个电池包中电池组的数量 */
/* -------------------------------- 宏定义 ------------------------------- */
#define CURRTIMES 5000         /* 5000 * 0.012 = 60秒 */

#define CHARGERITE           1         /* .1f     充电效率 */
#define UNCHARGERITE         1         /* 0.99f   放电效率 */
#define SOEADDR              32        /* SOE在FRAM中存储的地址，十进制 */
#define SOESAVETIMEADDDR     36        /* SOE保存时间在FRAM中的存储地址 */
#define USE_SOE_SAVE_TIME
#define ADCOFCURRBASEVAL     2250      /* 电流为0是的电压基准值。计算方法为4500/2 */
#define CELLAVGVOLT          3.6f      /* 定义电芯的平均电压值 */
//#define XSPACING             0.1f     /* 步长 */
#define MAXFSOCVAL           1.0

#define BATTTYPELIN          1         /* 定义电芯类型，磷酸铁锂为1 */
#define BATTTYPESAN          2         /* 三元材料为2 */
#define OCVTYPEDISCH         1         /* 定义放电状态 */
#define OCVTYPESTATIC        2         /* 定义静置状态 */
#define OCVTYPECHARGE        3         /* 定义充电状态 */
#define SOCMAXVAL            1000      /* SOC的最大值，也就是SOC可以分为多少份 */
#define SOCSTURCTLONG        11        /* 结构体的长度，也就是OCV结构体中的数据量 */

/* 物理边界值 */
#define MAXCELLV       4.20f           /* 电芯最高电压 */
#define MINCELLV       3.0f            /* 电芯最低电压 */
#define MAXPACKETCURR  50              /* 最大电流，放电 */
#define MINPACKETCURR  -50             /* 最小电流，充电 */
#define MAXSOCVAL      1.0f            /* SOC最大值 */
#define MINSOCVAL      0.0f            /* SOC最小值 */

#define MINR0VALUE     0.020f          /* 定义R0的最小值 */
#define MINR1VALUE     0.012f          /* 定义R1的最小值 */
#define MINR2VALUE     0.012f          /* 定义R2的最小值 */

/* 模型边界值 */
#define MAXU1VAL  0.1f                 /* U1最大值 */
#define MAXU2VAL  0.1f                 /* U2最大值 */
#define MAXRVAL   0.3f                 /*  R最大值 */

#define MAXSX0   1                     /* Sx0最大值 */
#define MAXSX1   1                     /* Sx1最大值 */
#define MAXSX2   1                     /* Sx2最大值 */
#define MAXPXY0  1                     /* Pxy0最大值 */
#define MAXPXY1  1                     /* Pxy1最大值 */
#define MAXPXY2  1                     /* Pxy2最大值 */

#define MAXSY    1                     /*  Sy最大值 */
#define MAXSYT   1                     /* SyT最大值 */
#define MAXK0    1                     /*  K0最大值 */
#define MAXK1    1                     /*  K1最大值 */
#define MAXK2    1                     /*  K2最大值 */

/*  SocUkf.c */

/* SohUkf.c */
#define SOH_S           1              /* 状态变量的维度 */
#define SOH_M           1              /* 观测测量变量的维度 */
#define SOH_V           1              /* 过程噪声的维度 */
#define SOH_N           1              /* 测量噪声的维度 */

#define SOH_STATE_SOH   0              /* 状态变量序号(SOC值)序号 */
#define SOH_STATE_UR1   1              /* UR1，极化内阻1的端电压值序号 */
#define SOH_STATE_UR2   2              /* UR2，极化内阻2的端电压值序号 */

#define SOH_NOISE_UR2   0              /* 极化内阻R2端电压的噪声序号 */
#define SOH_NOISE_UR1   1              /* 极化内阻R1端电压的噪声序号 */

#define LIMIT_MIN_VAL   1e-16f         /* 在做浮点数的接近0值判断时的最小浮点值 */

#define R0_MAX_VAL       0.085f
#define R0_MIN_VAL       0.070f


#define UPDATESOCTIME         60       /* 静置一定时间后通过单体电压更新SOC值，单位秒 */
/* #define UPDATERXTIME         30 */
#define UPDATESOCSW           1        /* 这里定义一个开关，是否启用静置一定时间后通过单体电压更新SOC值这个功能 */
#define CURRNOTZEROTIME       5        /* 在电流不为0开始x秒后改变状态(滤波的滞后问题)检测R0时的延时 */
#define GETR1ANDR2TIME        5        /* 这个是检测R1和R2的时间点，是在检测完R0后的一个延时，单位秒 */

#define CURRNOTZEROTIME_CH    1        /* 在电流不为0开始x秒后改变状态(滤波的滞后问题)检测R0时的延时 */
#define GETR1ANDR2TIME_CH     3        /* 这个是检测R1和R2的时间点，是在检测完R0后的一个延时，单位秒 */
#define NEWR0TIME             90       /* 60*60   3000 计算新的R0所需等待的值，单位秒 */
#define FU0RATE  0.98f
#define FU1RATE  0.98f
#define FU2RATE  0.98f

#define PARKTIME              120      /* 停车多长时间(分钟)后启用OCV测量SOC，单位分钟 */



/*  FRAM中存储的数据地址定义 */
#define SOC_INEEPROMADDR      16       /* 定义SOC在EEPROM中的首地址，占用两个字节 */
#define TIME_INEEPROMADDR     20       /* 定义SOC在存储到EEPROM中的时候所在的时间的首地址，占用6个字节 */
#define CURR_BASEVAULEADDR    26       /* 电流基准保存的地址 */
#define R0_INEEPROMADDR       30       /* 定义R0在存储到EEPROM中的地址 */
#define SOH_INEEPROMADDR      48       /* 定义SOH在EEPROM中的首地址，占用两个字节 */
#define SAVEDATAP             80       /* R0相关数据保存在FRAM中的地址，十进制 */
#define     R0_CHG_VAL_ADDR   96       /* R0——充电保存在FRAM中的地址为96，占用两个字节，高位在前 */
#define     R1_CHG_VAL_ADDR   98
#define     R2_CHG_VAL_ADDR   100
#define     R0_DCHG_VAL_ADDR  102      /* R0放电 */
#define     R1_DCHG_VAL_ADDR  104      /* R1放电 */
#define     R2_DCHG_VAL_ADDR  106      /* R2放电 */

#define R0_NEW                64       /* 新电池内阻 */
#define CHARGE_R0             66       /* 充电时测量到的R0 */
#define DISCHARGE_R0          68       /* 放电时测量到的R0 */
#define RCOUNTNUMADDR         192      /* R0统计次数在FRAM中的地址 */
#define RCOUNTVALUEADDR       194      /* R0统计累计值在FRAM中的地址 */
#define R0_AVG_CHAR_ADDR      70       /* R0_AVG充电计算值保存地址 */

#define R0_AVG_CNT_ADDR       74       /* R0_AVG统计次数 */
#define R0_AVG_M_ADDR         208      /* R0_AVG多次充电数据的首地址 */
#define R0_AVG_T_ADDR         72       /* R0_AVG首次数据时的温度值 */
#define R0_AVG_M_T_ADDR       608      /* R0_AVG后续数据的温度值 */
#define CHARGETIMES           44       /* 充电次数在FRAM中的地址 */
#define CHARGEINTEGRAL        36       /* 充电积分在FRAM中的地址 */
#define CHARGEUNCHINTE        40       /* 充放电积分在FRAM中的地址 */

#define DISCHARGEINTADDR      128      /* 放电积分保存在FRAM中位置 */
#define CHARGEINTADDR         132      /* 充电积分保存在FRAM中位置 */

// #define CURRZEROTOCH          5     /* 电流不为0开始x秒后检测R0值，充电过程，上面这个变量时放电过程 */
// #define UKFDELAYTIME          CURRNOTZEROTIME + GETR1ANDR2TIME + 1 /* 12   /* 延时x秒后进入UKF */
#define MAXDELAYTIME          5        /* 电流变为0后延时一定时间 */

#define CHG_USE_RX_TIMES      316      /* 充电统计次数，到达该次数后转换为充电Rx */
#define DCHG_USE_RX_TIMES     83       /* 632   放电统计次数，到达该次数后转换为放电Rx */
#define USED_CHG_RX           1        /* 当前使用充电Rx中 */
#define USED_DCHG_RX          0        /* 当前使用放电Rx中 */

/* 比较并获取最大值最小值 */
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* 电池组数据结构体，一个电池组由12个Cell组成，同时一个电池包有8个电池组组成 */
typedef struct
{
    uint16_t usCellV[CELLNUMONEBAT]; /* 一个电池组中12节电池单体的电压值 */
    int8_t  ucCellT[TEMPNUMONEBAT];  /* 一个电池组中5个温度传感器的值 */
}Batteries_t;

typedef struct
{
    Batteries_t MyBat[BATTNUMONPACK];/* 电池包中有N个电池组 */
    uint16_t usMaxCellV;             /* 一个电池组中单体电压最高值 */
    uint16_t usMinCellV;             /* 一个电池组中单体电压最低值 */
    uint16_t usEvgCellV;             /* 一个电池组中单体电压平均值 */
    int8_t  ucMaxCellT;              /* 一个电池组中N个温度传感器的最高值 */
    int8_t  ucMinCellT;              /* 一个电池组中N个温度传感器的最低值 */
    int8_t  ucEvgCellT;              /* 一个电池组中N个温度传感器的平均值 */
    int8_t  ucMaxVLocal;
    int8_t  ucMinVLocal;
}Packet_t;                           /* 电池包数据结构，有N个电池组和6个统计值组成 */

/* 电池包各个物理参数的预警参数 */
typedef struct Packet_Alarm_Val_t{
    float fCellVoltH;              /* 单体电压过高预警值 */
    float fCellVoltL;              /* 单体电压过低预警值 */
    float fCellVoltDiff;           /* 单体压差过高预警值 */
    float fPackVoltH;              /* 总体电压过高预警值 */
    float fPackVoltL;              /* 总体电压过低预警值 */
    float fPackDisCurrH;           /* 总体放电电流过高预警值 */
    float fPackCharCurrH;          /* 总体充电电流过高预警值 */
    int iCellTempH;                /* 单体温度过高预警值 */
    int iCellTempL;                /* 单体温度过低预警值 */
    int iCellTempDiff;             /* 单体温差过高预警值 */
}Packet_Alarm_Val_t;

extern Packet_Alarm_Val_t stAlarm;
extern uint32_t uDataOverFlag ;   /* 是否有超范围发生标记位 */
extern Packet_t MyPacket;
extern float g_fPackCurrA;
extern float g_fPackVoltV;
extern int   g_ROMSoc;                 /* EEPROM中保存的SOC值，范围0~1 */
extern unsigned char ucRTC[6];
extern unsigned char ucR0Time[8];      /* 从FRAM中读取到的R0保存时的时间信息 */
extern float fSOC_R0_VALUE;            /* 二阶RC模型的欧姆内阻 */

/* -------------------------------- 全局变量声明 ------------------------------- */
extern struct ocvUpdata MeasData;
extern struct timeUpdata TimeData;
extern struct sohUpdata SohData;

extern float BattQAs;
extern float fLambda;                  /* SOC修正系数,包含温度修正，充放电倍率修正，充放电次数修正 */
extern float T_StopState_OCV_TableF32[SOCSTURCTLONG];
extern float fSocQCurrVal_1;
extern float fSohQDefVal_1;
extern float fSohQCurrVal_1;
extern float fSoh_Vcell_Noise;         /* 二阶RC模型的测量噪声-单体电压 */
extern float fSoh_UR2_noise;           /* 二阶RC模型的过程噪声-UR2 */
extern float fSoh_UR1_noise;           /* 二阶RC模型的过程噪声-UR1 */
extern float fCurrSOH ;                /* 当前的SOH值 */
extern float g_PacketInmR;
/* extern float g_CellVNormalize; */
extern uint32_t uDataOverFlag;

extern float invalid_ratio ;
/* 定义失效容量变量  */
extern float invalid_Q;

extern float g_PacketDisChargeCnt;     /* 统计电池包放电电量,单位Ah */
extern float g_PacketChargeCnt;        /* 统计电池包充电电量,单位Ah */
/* BMS变量 */
extern float g_Soc;                    /* SOC全局变量，范围0~1 */
extern float g_Soh;
extern float g_SOEVal;                 /* 当前SOE的值 */
extern int   g_ROMSoc;                 /* EEPROM中保存的SOC值，范围0~1 */
extern float g_PacketVoltV;            /* 电池包的总电压，单位V */
extern float g_PacketCurrA;            /* 电池包的总电流，单位A */
extern char  g_PacketMaxTem;           /* 电池包的最高温度，单位摄氏度 */
extern char  g_PacketMinTem;           /* 电池包的最低温度，单位摄氏度 */
extern char  g_PacketTem;              /* 电池包的温度，单位摄氏度。默认设置为25度 */
extern float g_MinCellV;               /* 电池包最低单体电压 */
extern float g_MaxCellV;               /* 电池包的最高单体电压 */
extern float g_AvgCellV;               /* 电池包的平均单体电压 */
extern float g_WeighCellV;             /* 加权的单体电压值，这个值与最大值，最小值和SOC值有关，只用作SOC计算时的测量更新用 */
extern float g_FirstWei;               /* BMS启动后再不放电的情形下的稳定单体电压值 */
extern float g_SecondWei;
extern float g_ThirWei;

extern float g_FirstWeiMax;            /* BMS启动后再不放电的情形下的稳定单体电压值 */
extern float g_SecondWeiMax;
extern float g_ThirWeiMax;
extern float g_fMaxWeiVal;

extern float g_FirstPackV;
extern float g_SecondPackV;
extern float g_ThirPackV;
extern char  g_PackState;              /* 电池包状态，2代表充电中，0代表静置中，1代表放电中 */
/* 各种电流滤波变量 */
extern float g_originalCurr;           /* 原始的电流值，未经滤波的电流值 */
extern float g_WeightCurr;             /* 带权重的电流滤波值 */
extern float g_smoothCurr;             /* 滑动滤波，三选一，三十平均 */
extern float g_firfilCurr;             /* fir滤波电流值 */
extern int   g_PackChargeTimes;        /* 记录电池包的充电次数 */
extern float g_PackCharge_AS;          /* 电池包充电容量统计，统计够电池包额定容量后充电次数加1 */
extern float g_PackAhCnt_AS;           /* 电池包充放电容量积分值，单位安秒 */
extern float fWeiAndUxSOC;
extern float g_fR0CurrVal;             /* R0的当前值 */
extern float g_fR0CntVal;              /* R0 累计值 */
extern float g_fR0AvgVal;              /* R0 平均值 */
extern int   g_iR0CntNum;              /* R0值的统计次数 */
extern int   g_iR0AvgUpdataFlag;       /* R0Avg更新标记 */
extern float g_fRxFilter;
extern uint16_t u16R0_Avg_Cnt;
/* 电流状态判断变量 */
extern float lastCurrVal;              /* 记录上一次的电流情况，这里也是以A为单位的浮点数 */
extern int   iCurrTimeCnt;             /* 电流计时变量 */
extern float StaticCellV;              /* 静置时的单体电压 */
extern int   iCurrStat;                /* 记录当前电流状态。如果电流为0，该值为0，否则为1 */
extern int   iUpdateSocTimeCnt;        /* 更新SOC计时变量，这里默认值给1，如果给0会导致系统启动后如果电流一直为0，就无法统计静置时间 */

extern unsigned char ucRTC[6];
extern unsigned char ucR0Time[8];      /* 从FRAM中读取到的R0保存时的时间信息 */
extern unsigned char eepromBuffer[20];

extern int updateR0;

extern float fEtaDis;                  /* 放电时的库伦效率 */
extern float fEtaChg;                  /* 充电时的库伦效率 */
extern float fLambda;                  /* 温度对电池容量的影响系数 */
extern float fCurrSOC;                 /* 当前的SOC值 */
extern float fDtSohDis;                /* 放电时，当前SOC减小到最小允许SOC时的时间 */
extern float fDtSohChg;                /* 放电时，当前SOC减小到最小允许SOC时的时间 */
extern float BattQAs;
extern float fSOC_Q;
extern char  cNoCurr1Hour;

/* 放电变量 */
extern float fSocDisMaxI;              /* SOC因素获取到的最大电流 */
extern float fVoltDisMaxI;             /* 电压因素获取到的最大电流 */
extern float fBattDisMaxI;             /* 电池本身因素得到的最大电流 */
extern float fFulsDisMaxI;             /* 保险丝因素得到的最大电流 */
extern float fRelayDisMaxI;            /* 接触器因素得到的最大电流 */
extern float fLastDisMaxI;             /* 综合所有因素计算获取到的最终电流值 */
extern float fSOPDis;                  /* 最后需要输出的SOP值 */

/* 充电变量 */
extern float fSocChgMaxI;              /* SOC因素获取到的最大电流 */
extern float fVoltChgMaxI;             /* 电压因素获取到的最大电流 */
extern float fBattChgMaxI;             /* 电池本身因素得到的最大电流 */
extern float fFulsChgMaxI;             /* 保险丝因素得到的最大电流 */
extern float fRelayChgMaxI;            /* 接触器因素得到的最大电流 */
extern float fLastChgMaxI;             /* 综合所有因素计算获取到的最终电流值 */
extern float fSOPChg;                  /* 最后需要输出的SOP值 */

extern float fMaxSoc;                  /* 最大允许SOC */
extern float fMinSoc;                  /* 最小允许SOC */

/* 计算SOP可能用到的其他算法算出的结果 */
extern float fCurrU1;
extern float fCurrU2;
extern float fCurrU0;

extern uint16_t u16_R0_Chg_Val;
extern uint16_t u16_R1_Chg_Val;
extern uint16_t u16_R2_Chg_Val;
extern uint16_t u16_R0_DChg_Val;
extern uint16_t u16_R1_DChg_Val;
extern uint16_t u16_R2_DChg_Val;

extern float fSOC_Step1_NOISE;         /* 二阶RC模型的过程噪声-UR2 */
extern float fSOC_Step2_NOISE;         /* 二阶RC模型的测量噪声-单体电压 */
extern float fSOC_UR1_NOISE;           /* 二阶RC模型的过程噪声-UR1 */

extern float fSOC_R0_VALUE;            /* 二阶RC模型的欧姆内阻 */
extern float fSOC_R1_VALUE;            /* 二阶RC模型的极化内阻1 */
extern float fSOC_C1_VALUE;            /* 二阶RC模型的极化电容1 */
extern float fSOC_R2_VALUE;            /* 二阶RC模型的极化内阻2 */
extern float fSOC_C2_VALUE;            /* 二阶RC模型的极化电容2 */

extern float fSocQDefVal_1;
extern float fSocQDefVal_2;
extern float fSocQDefVal_3;
extern float fSocQDefVal_4;

extern float fSocQCurrVal_1;
extern float fSocQCurrVal_2;
extern float fSocQCurrVal_3;
extern float fSocQCurrVal_4;
extern float BattQAs;
extern float fDeltTime;


/* SOP公共变量 */

extern float fSopDt     ;              /* 估计步长,单位秒S */
extern float fSopUmax   ;              /* 最大充电电压，单位V */
extern float fSopUmin   ;              /* 最小放电电压，单位V */
extern float fSopImaxChg;              /* 最大充电电流，单位A(1C) */
extern float fSopImaxDch;              /* 最大放电电流，单位A(3C) */
extern float fSopPchg   ;              /* 额定充电功率，单位W(4.2*2.7 = 11.34) */
extern float fSopPdch   ;              /* 额定放电功率，单位W(4.2*8.1 = 34.02) */
extern float fSopSoCmin ;              /* 最小荷电状态，单位% */
extern float fSopSoCmax ;              /* 最大荷电状态，单位% */
extern float g_fL;                     /* 预测SOP时的周期个数 */
extern float g_fE1PowerL;
extern float g_fOneSubE1;
extern float g_fSumE1PowerJ;
extern float g_fNiLdTCmax;
extern float g_fE2PowerL;
extern float g_fOneSubE2;
extern float g_fSumE2PowerJ;

extern float fR0New;                   /* 新电池的内阻值(fSOC_R1_VALUE + fSOC_R2_VALUE) */
extern float fR0EolBit;                /* 电池寿命终结时的内阻与新电池内阻的比值(2.0f) */
extern float fR0Eol;                   /* 电池寿命终结时的内阻(fR0New * fR0EolBit) */

extern float fQBit;                    /* 电池寿命终结时的可用容量与新电池可用容量的比值(0.8f) */
extern float fQEol;                    /* 电池寿命终结时的可用容量(fSOC_Q * fQBit) */
extern float fDisChR0;                 /* 放电R0值 */
extern float fChargeR0;                /* 充电R0值 */

/* extern float xi[]; */
/* extern float yi[]; */
/* extern float dyi[]; */

extern int   iUsedUKF;                 /* 使用UKF开关 */
extern float fsocval;
extern int   g_PackChargeTimes;        /* 记录电池包的充电次数 */

extern float e_dtr1c1val ;             /* e^(-dt/(R1C1)) */
extern float e_dtr2c2val ;             /* e^(-dt/(R2C2)) */

extern char  g_PacketEvgTem;           /* 电池包的平均温度，单位摄氏度 */

extern char  ucBattType;               /* 磷酸铁锂为1，三元材料为2 */

extern float g_CurrNoZeroFlag;
/* 引自 rtos.c */

extern int   iCurrZeroDelay;
extern int   iCurrStat;
extern float g_firfilCurr;             /* fir滤波电流值 */

extern  unsigned long  g_ParkTimeMin;  /* 停车时间，单位分钟 */
extern  unsigned char  g_OCVFlag;      /* OCV开启标记 */
extern unsigned short  g_MaxCellLocal; /* 最高电压电芯的位置，高字节为模组位置，低字节为Cell位置 */
extern unsigned short  g_MinCellLocal; /* 最低电压电芯的位置，高字节为模组位置，低字节为Cell位置 */
extern unsigned short  g_ChargeVolt;   /* 充电机发送过来的当前充电电压 */
extern unsigned short  g_ChargeCurr;   /* 充电机发送过来的当前充电电流 */
extern unsigned char   g_ChargeState;  /* 充电机的当前状态 */
extern unsigned char   g_OcvUpdateFlag;
extern unsigned char   g_EEPUpdateFlag;
extern unsigned char   g_CurrOffsetFlag;/* 定义一个变量用来在启动的时候校准电流传感器 */
extern unsigned short  g_CurrAdBase;    /* 定义一个变量，用来标记电流为0时的电压基准值，这里用变量时因为在启动的时候需要自动校正 */
extern unsigned long   ulMaxChargeV;    /* 电池包最大充电电压，单位mV */
extern unsigned long   ulMaxChargeC;    /* 电池包最大充电电流，单位mA */

/* 三元电芯充电过程的单体电压与SOC对应表 */
extern unsigned short T_Charge_OCV_TableS[SOCSTURCTLONG];
/* 三元电芯放电过程的单体电压与SOC对应表 */
extern unsigned short T_DisCharge_OCV_TableS[SOCSTURCTLONG];
/* 三元电芯充分静置(3小时)后的单体电压与SOC对应表 */
extern unsigned short T_StopState_OCV_TableS[SOCSTURCTLONG];
/* 磷酸铁锂电芯充电过程的单体电压与SOC对应表 */
extern unsigned short T_Charge_OCV_TableT[SOCSTURCTLONG];
/* 磷酸铁锂电芯放电过程的单体电压与SOC对应表 */
extern unsigned short T_DisCharge_OCV_TableT[SOCSTURCTLONG];
/* 磷酸铁锂电芯充分静置(3小时)后的单体电压与SOC对应表 */
extern unsigned short T_StopState_OCV_TableT[SOCSTURCTLONG];
/* 电池最大可用容量的温度修正参数列表 */
extern unsigned short T_Temperature_Q_Parameter_TableT[SOCSTURCTLONG];
/* 电池充电时SOC对应的欧姆内阻列表 */
extern unsigned short T_CH_R0_SOC_TableT[SOCSTURCTLONG];
/* 电池放电时SOC对应的欧姆内阻列表 */
extern unsigned short T_DIS_R0_SOC_TableT[SOCSTURCTLONG];

/* -------------------------------- 全局数据结构 ------------------------------- */
/* SOP计算算法结果输出的数据结构 */
typedef struct SopRet
{
    float fDisMaxSop;
    float fChgMaxSop;
}SOPRETSTRUCT;

/* -------------------------------- 全局数函数 ------------------------------- */

extern void *aqDataCalloc(uint16_t count, uint16_t size);
extern float GetFixValForQ(char Temp);
extern float GetFixValForH(float soh);
extern float value_limiter_f32(const float state,const float min,const float max);
//extern float __sqrtf(float val);
unsigned short * Get_Data_Pointer(char cBatType,char PackStat);
/* 根据给定的数值在结构体中查找对应的数值，通过电压查找SOC，相当于正向查找。参数1结构体指针，参数2结构体长度，参数3给定的数值。 */
short Get_SOC_FromCellV(unsigned short uCellmV,char cBatType,char PackStat);
float Get_SOC_FromCellV_f32(float fCellV);
/* 通过SOC查找单体电压，相当于逆向查找。*/
short Get_CellV_FromSOC(int isoc,char cBatType,char PackStat);
float Get_CellV_FromSoc_f32(float fsoc);
float GetSocFromCellV(float fCellV);
#endif /* APPLICATIONS_BMS_H_ */
