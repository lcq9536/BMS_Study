/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-29     yjm       the first version
 */
#include "stdint.h"
#include "bms.h"
#include <stddef.h>
/* 定义预警值结构体变量并赋值 */
Packet_Alarm_Val_t stAlarm ={
    .fCellVoltH     = 4.25f, /* 单体电压过高预警值 */
    .fCellVoltL     = 2.75f, /* 单体电压过低预警值 */
    .fCellVoltDiff  = 0.1f,  /* 单体压差过高预警值 */
    .fPackVoltH     = 52.0f, /* 总体电压过高预警值 */
    .fPackVoltL     = 25.0f, /* 总体电压过低预警值 */
    .fPackDisCurrH  = 10.0f, /* 总体放电电流过高预警值 */
    .fPackCharCurrH = -5.0f, /* 总体充电电流过高预警值,充电电流为负值 */
    .iCellTempH     = 60,    /* 单体温度过高预警值 */
    .iCellTempL     = -20,   /* 单体温度过低预警值 */
    .iCellTempDiff  = 10,    /* 单体温差过大预警值 */
};

/* BMS超限预警标记变量,每个bit标记一种超限，最多标记32类
 * 变量bit位示意图
 * [31][30][29][28][27][26][25][24] // 预留
 * [23][22][21][20][19][18][17][16] // 总体预警
 * [15][14][13][12][11][10][09][08] // 单体预警
 * [07][06][05][04][03][02][01][00] // 预留
 *
 * 23：1为总电压过压，0为不过压
 * 22：1为总电压欠压，0为不欠压
 * 21：1为总电流充电过流，0为不过流
 * 20：1为总电流放电过流，0为不过流
 *
 * 15：1为单体过压，0为不过压
 * 14：1为单体欠压，0为不欠压
 * 13：1为压差过大，
 * 12：1为单体温度过高，
 * 11：1为单体温度过低，
 * 10：1为温差过大，
*/
uint32_t uDataOverFlag = 0;  /* 是否有超范围发生标记位 */

Packet_t MyPacket;           /* 定义电池包数据结构体 */
float g_fPackCurrA;          /* 定义电池包的总电流 */
float g_fPackVoltV;          /* 定义电池包的总电压 */


/*
SOC：state of charge, 可以理解为电池剩余电量百分比；
SOE：state of energy，可以理解为电池剩余电量，或者说对于整车来说，剩余里程；
SOH：state of health, 电池健康度，可以理解为电池当前的容量与出厂容量的百分比；
SOF：state of function, 电池的功能状态，可以理解为控制策略中的一个参数
SOP: state of power ,可以理解为电池当前的功率承受能力。
*/

float fLambda = 1.0f;   /* 总容量修正系数，包含温度修正，充放电倍率修正，充放电次数修正 */
float fEtaDis = 1.0f;   /* 放电时的库伦效率 */
float fEtaChg = 1.0f;   /* 充电时的库伦效率 */
float BattQAs = 0.0f;   /* 电池的容量，单位安秒 */
char  g_PacketEvgTem;   /* 电池包的平均温度，单位摄氏度 */
/* BMS变量 */
float g_Soc = 0.0f;     /* SOC全局变量，范围0~1 */
//float g_R0Val;        /* 当前的欧姆内阻值 */
float g_SOEVal;         /* 当前SOE的值 */
int   g_ROMSoc = 0;     /* EEPROM中保存的SOC值，范围0~1 */
float g_PacketVoltV;    /* 电池包的总电压，单位V */
float g_PacketCurrA;    /* 电池包的总电流，单位A */
char  g_PacketMaxTem;   /* 电池包的最高温度，单位摄氏度 */
char  g_PacketMinTem;   /* 电池包的最低温度，单位摄氏度 */
char  g_PacketTem = 25; /* 电池包的温度，单位摄氏度。默认设置为25度 */
float g_MinCellV;       /* 电池包最低单体电压 */
float g_MaxCellV;       /* 电池包的最高单体电压 */
float g_AvgCellV;       /* 电池包的平均单体电压 */
float g_WeighCellV;     /* 加权的单体电压值，这个值与最大值，最小值和SOC值有关，只用作SOC计算时的测量更新用 */

float g_FirstWei     = 0.0f; /* BMS启动后再不放电的情形下的稳定单体电压值 */
float g_SecondWei    = 0.0f; /* 从电流不为0开始的第一时间节点(比如放电9秒)的单体电压值 */
float g_ThirWei      = 0.0f; /* 从电流不为0开始的第二时间节点(比如9+5秒)的单体电压值 */
//
float g_FirstWeiMax  = 0.0f; /* BMS启动后再不放电的情形下的稳定单体电压值 */
float g_SecondWeiMax = 0.0f; /* 从电流不为0开始的第一时间节点(比如放电9秒)的单体电压值 */
float g_ThirWeiMax   = 0.0f; /* 从电流不为0开始的第二时间节点(比如9+5秒)的单体电压值 */
//
float g_FirstPackV   = 0.0f; /* 电流为0时的总电压值 */
float g_SecondPackV  = 0.0f; /* 从电流不为0开始的第一时间节点(比如放电9秒)的总电压 */
float g_ThirPackV    = 0.0f; /* 从电流不为0开始的第二时间节点(比如9+5秒)的总电压 */

char  g_PackState    = 0;    /* 电池包状态，2代表充电中，0代表静置中，1代表放电中 */
/* 各种电流滤波变量 */
float g_originalCurr = 0.0f; /* 原始的电流值，未经滤波的电流值 */
float g_WeightCurr   = 0.0f; /* 带权重的电流滤波值 */
float g_smoothCurr   = 0.0f; /* 滑动滤波，三选一，三十平均 */
float g_firfilCurr   = 0.0f; /* fir滤波电流值 */
int   g_PackChargeTimes;     /* 记录电池包的充电次数 */
float g_PackCharge_AS= 0.0f; /* 电池包充电容量统计，统计够电池包额定容量后充电次数加1 */
float g_PackAhCnt_AS = 0.0f; /* 电池包充放电容量积分值，单位安秒 */
long  lParkTimeMin   = 0;    /* 停车时间，单位分钟 */
float fDeltTime      = 0.0f; /* 定义采样周期的浮点型变量 */
float g_PacketInmR   = 0.0f; /* 电池包总体内阻（R0,R1,R2的合计值) */

float g_PacketDisChargeCnt = 0.0f;  /* 统计电池包放电电量,单位Ah */
float g_PacketChargeCnt = 0.0f;     /* 统计电池包充电电量,单位Ah */

float g_fMaxWeiVal = 0.0f;

float fWeiAndUxSOC;
float g_fR0CurrVal= 0.0f;  /* R0的当前值 */
float g_fR0CntVal = 0.0f;  /* R0 累计值 */
float g_fR0AvgVal = 0.0f;  /* R0 平均值 */
int   g_iR0CntNum = 0;     /* R0值的统计次数 */
int   g_iR0AvgUpdataFlag = 0;  //R0Avg更新标记 */
float g_fRxFilter=0.0f;

/* 电流状态判断变量 */
float lastCurrVal      = 0.0f;  /* 记录上一次的电流情况，这里也是以A为单位的浮点数 */
int  iCurrTimeCnt      = 0;     /* 电流计时变量 */
float StaticCellV      = 0.0f;  /* 静置时的单体电压 */
int  iCurrStat         = 0;     /* 记录当前电流状态。如果电流为0，该值为0，否则为1 */
int  iUpdateSocTimeCnt = 1;     /* 更新SOC计时变量，这里默认值给1，如果给0会导致系统启动后如果电流一直为0，就无法统计静置时间*/

unsigned char ucRTC[6];
unsigned char ucR0Time[8];      /* 从FRAM中读取到的R0保存时的时间信息 */
int updateR0=0;
unsigned char eepromBuffer[20];
/* ----------------- SOC参数-------------------------- */
float fCurrSOC = 0.0f;          /* 当前的SOC值 */
float fCurrU1  = 0.0f;
float fCurrU2  = 0.0f;
float fCurrU0  = 0.0f;
float fCurrSOH = 0.0f;          /* 当前的SOH值 */
/* SOC_UKF中Q的默认值   这里是误差协方差，没有被开方的值 */
float fSocQDefVal_1  = 0.000000000000001f;/* 0.000000000000001f; 1e-14 yroot = 0.0000001 */
float fSocQDefVal_2  = 0.0000000001f;     /*  1e-10 yroot = 0.00001 */
float fSocQDefVal_3  = 0.0000000001f;     /*  1e-10 yroot = 0.00001 */
float fSocQDefVal_4  = 0.0000000001f;     /*  1e-10 yroot = 0.00001 */

/* SOC_UKF中Q的当前值  这里是状态变量的初始默认值，上面是状态变量误差协方差的默认值。两个不是一个概念 */
float fSocQCurrVal_1 = 0.68f;
float fSocQCurrVal_2 = 0.0008f;/* 0.0002f;  0.000f;    这三个值才是观测变量初值，单位V */
float fSocQCurrVal_3 = 0.0008f;/* 0.0002f;//0.000f; */
float fSocQCurrVal_4 = 0.0008f;/* 0.0002f;//0.000f; */

/* OcvSoc的当前状态，即放电，充电，还是静置 */
char ucOCVSocType = OCVTYPEDISCH;    /* 放电状态为1,静置状态为2,充电状态为3 */
char ucBattType   = BATTTYPESAN;     /* 磷酸铁锂为1，三元材料为2 */

/* 一步预测用到的噪声 */
float fSOC_Step1_NOISE = 0.000001f;  /* 二阶RC模型的过程噪声-UR2      这个观测噪声是在SocUkfInit中传递给了srcdkfSetVariance */
float fSOC_UR1_NOISE   = 0.0001f;    /* 二阶RC模型的过程噪声-UR1      这个噪声值只在原来的两个观测噪声时有用，如果观测噪声改为一个，这个没有用到 */
/* 二步用到的噪声 */
float fSOC_Step2_NOISE = 0.0000005f; /* 二阶RC模型的测量噪声-单体电压  这个测量噪声在SocDoOCVUpdate函数中传递给了srcdkfMeasurementUpdate */

float fSOC_R0_VALUE    = MINR0VALUE; /* 二阶RC模型的欧姆内阻 */
float fSOC_R1_VALUE    = MINR1VALUE; /* 二阶RC模型的极化内阻1  20 */
float fSOC_C1_VALUE    = 1000.0f;    /* 二阶RC模型的极化电容1 */
float fSOC_R2_VALUE    = MINR2VALUE; /* 二阶RC模型的极化内阻2  35 */
float fSOC_C2_VALUE    = 1000.0f;    /* 二阶RC模型的极化电容2 */

uint16_t u16_R0_Chg_Val;
uint16_t u16_R1_Chg_Val;
uint16_t u16_R2_Chg_Val;
uint16_t u16_R0_DChg_Val;
uint16_t u16_R1_DChg_Val;
uint16_t u16_R2_DChg_Val;

float e_dtr1c1val      = 0.0f;       /* e^(-dt/(R1C1)) */
float e_dtr2c2val      = 0.0f;       /* e^(-dt/(R2C2)) */
char  cNoCurr1Hour     = 0;          /* 静置时间是否达到一个小时 */
float g_CurrNoZeroFlag = 0.0f;

//float g_CellVNormalize = 0.0f;

/* ----------------- SOH参数-------------------------- */
//int   g_Soh            = 1000;     /* SOH全局变量，范围0~1000 */
float g_Soh     = 0.988f;
//SOH_UKF中Q的默认值
float fSohQDefVal_1    = 0.99f;
//SOH_UKF中Q的当前值
float fSohQCurrVal_1   = 0.98f;
float fDtSohDis        = 10.0f;      /* 放电时，当前SOC减小到最小允许SOC时的时间 */
float fDtSohChg        = 10.0f;      /* 放电时，当前SOC减小到最小允许SOC时的时间 */
float fSoh_Vcell_Noise = 0.1f;       /* 二阶RC模型的测量噪声-单体电压 */
float fSoh_UR2_noise   = 0.1f;       /* 二阶RC模型的过程噪声-UR2 */
float fSoh_UR1_noise   = 0.1f;       /* 二阶RC模型的过程噪声-UR1 */

float fR0New    = 0.0f;              /* 新电池的内阻值(fSOC_R1_VALUE + fSOC_R2_VALUE) */
float fR0EolBit = 0.0f;              /* 电池寿命终结时的内阻与新电池内阻的比值(2.0f) */
float fR0Eol    = 0.0f;              /* 电池寿命终结时的内阻(fR0New * fR0EolBit) */

float fQBit = 0.0f;                  /* 电池寿命终结时的可用容量与新电池可用容量的比值(0.8f) */
float fQEol = 0.0f;                  /* 电池寿命终结时的可用容量(fSOC_Q * fQBit) */

float fDisChR0 = 0;                  /* 放电R0值 */
float fChargeR0 = 0;                 /* 充电R0值 */
uint16_t u16R0_Avg_Cnt=0;            /* 保存在FRAM中的R0_AVG的次数(个数) */

uint16_t uRCntNum = 0;               /* 内阻统计的次数 */
uint16_t uRCntVal = 0;               /* 内阻统计累加值 */
/* ----------------- SOP参数-------------------------- */
/* 放电变量 */
float fSocDisMaxI   = 0.0f;          /* SOC因素获取到的最大电流 */
float fVoltDisMaxI  = 0.0f;          /* 电压因素获取到的最大电流 */
float fBattDisMaxI  = 0.0f;          /* 电池本身因素得到的最大电流 */
float fFulsDisMaxI  = 0.0f;          /* 保险丝因素得到的最大电流 */
float fRelayDisMaxI = 0.0f;          /* 接触器因素得到的最大电流 */
float fLastDisMaxI  = 0.0f;          /* 综合所有因素计算获取到的最终电流值 */
float fSOPDis       = 0.0f;          /* 最后需要输出的SOP值 */

/* 充电变量 */
float fSocChgMaxI   = 0.0f;          /* SOC因素获取到的最大电流 */
float fVoltChgMaxI  = 0.0f;          /* 电压因素获取到的最大电流 */
float fBattChgMaxI  = 0.0f;          /* 电池本身因素得到的最大电流 */
float fFulsChgMaxI  = 0.0f;          /* 保险丝因素得到的最大电流 */
float fRelayChgMaxI = 0.0f;          /* 接触器因素得到的最大电流 */
float fLastChgMaxI  = 0.0f;          /* 综合所有因素计算获取到的最终电流值 */
float fSOPChg       = 0.0f;          /* 最后需要输出的SOP值 */
float fMaxSoc       = 0.95f;         /* 最大允许SOC */
float fMinSoc       = 0.05f;         /* 最小允许SOC */

/* SOP公共变量 */

float fSopDt        = 10.0f;         /* 估计步长,单位秒S */
float fSopUmax      = 4.2f;          /* 最大充电电压，单位V */
float fSopUmin      = 3.0f;          /* 最小放电电压，单位V */
float fSopImaxChg   = -2.7f;         /* 最大充电电流，单位A(1C) */
float fSopImaxDch   = 8.1f;          /* 最大放电电流，单位A(3C) */
float fSopPchg      = -11.34f;       /* 额定充电功率，单位W(4.2*2.7 = 11.34) */
float fSopPdch      = 34.02f;        /* 额定放电功率，单位W(4.2*8.1 = 34.02) */
float fSopSoCmin    = 0.3f;          /* 最小荷电状态，单位% */
float fSopSoCmax    = 0.9f;          /* 最大荷电状态，单位% */
float g_fL          = 0;             /* 预测SOP时的周期个数 */

float g_fE1PowerL=0;
float g_fOneSubE1=0;
float g_fSumE1PowerJ=0;
float g_fE2PowerL=0;
float g_fOneSubE2=0;
float g_fSumE2PowerJ=0;
float g_fNiLdTCmax=0;
/* -------------------电池组包的整体参数----------------- */
float fSOC_Q         = 2.7f;  /* 定义电池组的额定容量，单位安时 */
int   iCellNumber    = 12;    /* 电池串联数量 */
float fPacketMaxVolt = 50.4f;
float fPacketMinVolt = 36.0f;

/* 定义失效比例变量  */
float invalid_ratio = 0.7f;
/* 定义失效容量变量  */
float invalid_Q;

/* -------------------电池组单体电压参数----------------- */
/* 电池组电压阈值设置 */
float fCellMaxOverVolt    = 4.20f;  /* 电池充电过压阈值 */
float fCellMaxReleaseVolt = 4.15f;  /* 电池充电过压释放阈值 */
float fCellMinOverVolt    = 3.00f;  /* 电池放电欠压阈值 */
float fCellMinReleaseVolt = 3.2f;   /* 电池放电欠压释放阈值 */
/* 电池组单体电压统计 */
float fCellMaxVolt = 4.001f;
float fCellMinVolt = 3.210f;
float fCellAveVolt = 3.702f;

/* -------------------电池组单体温度参数----------------- */
int TempCnt=16;                      /* 电池包温度传感器数量 */

/*  高温三级预警 */
int iTempOverLeve1 = 35;
int iTempOverLeve2 = 45;
int iTempOverLeve3 = 50;
/* 低温三级预警 */
int iTempUnderLeve1 = 0;
int iTempUnderLeve2 = -5;
int iTempUnderLeve3 = -20;

/* 电池组温度统计 */
int MaxTemp   = 30;    /* 最高温度 */
int MinTemp   = -10;   /* 最低温度 */
int AveTemp   = 20;    /* 平均温度 */
int iUsedUKF  = 0;     /* 是否使用UKF算法 */
int g_PackChargeTimes; /* 记录电池包的充电次数 */
float fsocval = 0.0f;

unsigned long  ulMaxChargeV = 100800;/* 电池包最大充电电压，单位mV */
unsigned long  ulMaxChargeC = 1000;  /* 电池包最大充电电流，单位mA */
unsigned long  g_ParkTimeMin;        /* 停车时间，单位分钟 */
uint16_t  g_MaxCellLocal;            /* 最高电压电芯的位置，高字节为模组位置，低字节为Cell位置 */
uint16_t  g_MinCellLocal;            /* 最低电压电芯的位置，高字节为模组位置，低字节为Cell位置 */
uint16_t  g_ChargeVolt;              /* 充电机发送过来的当前充电电压 */
uint16_t  g_ChargeCurr;              /* 充电机发送过来的当前充电电流 */
uint16_t  g_CurrAdBase = ADCOFCURRBASEVAL;   /* 定义一个变量，用来标记电流为0时的电压基准值，这里用变量时因为在启动的时候需要自动校正 */
uint8_t   g_ChargeState;             /* 充电机的当前状态 */
uint8_t   g_OCVFlag        = 0;      /* OCV测量SOC开关 */
uint8_t   g_OcvUpdateFlag  = 0;
uint8_t   g_EEPUpdateFlag  = 0;
uint8_t   g_CurrOffsetFlag = 0;      /* 定义一个变量用来在启动的时候校准电流传感器 */



/*------------------------------------------------------------------------------
*  External table used for linear interpolation
*------------------------------------------------------------------------------*/
float T_StopState_OCV_TableF32[SOCSTURCTLONG] =  {
/* 2019.09.06   2019.08.13 */
    3.320f,      /* 3.266,      //3.320,    //3387,     // SOC=0 */
    3.436f,      /* 3.392,      //3.426,    //3684,     // SOC=10 */
    3.537f,      /* 3.517,      //3.527,    //3737,     // SOC=20 */
    3.593f,      /* 3.593,      //3.578,    //3770,     // SOC=30 */
    3.629f,      /* 3.629,      //3.629,    //3780,     // SOC=40 */
    3.714f,      /* 3.714,      //3.714,    //3806,     // SOC=50 */
    3.811f,      /* 3.811,      //3.811,    //3865,     // SOC=60 */
    3.890f,      /* 3.890,      //3.890,    //3912,     // SOC=70 */
    3.974f,      /* 3.974,      //3.974,    //3971,     // SOC=80 */
    4.071f,      /* 4.071,      //4.071,    //4046,     // SOC=90 */
    4.168f       /* 4.189       //4.189     //4185      // SOC=100 */
};

/* Q_Cnt = 2.7Ah */
/* 循环次数对电池容量修正二维表 */
float T_ChargeTime_Q_TableF32[11] = {
/*  充电次数               电池最大容量比例 */
   0.0f,                  /* 100.0f * Q_Cnt */
   100.0f,                /* 97.0f * Q_Cnt */
   200.0f,                /* 94.0f * Q_Cnt */
   300.0f,                /* 91.0f * Q_Cnt */
   400.0f,                /* 88.0f * Q_Cnt */
   500.0f,                /* 85.0f * Q_Cnt */
   600.0f,                /* 82.0f * Q_Cnt */
   700.0f,                /* 79.0f * Q_Cnt */
   800.0f,                /* 76.0f * Q_Cnt */
   900.0f,                /* 73.0f * Q_Cnt */
   1000.0f                /* 70.0f * Q_Cnt */
};
/* 内阻值与电池容量修正二维表 */
float T_R_Q_TableF32[11] = {
/*  内阻值，单位毫欧       电池最大容量比例 */
   100.0f,                /* 100.0f * Q_Cnt */
   110.0f,                /* 98.0f * Q_Cnt */
   120.0f,                /* 96.0f * Q_Cnt */
   130.0f,                /* 94.0f * Q_Cnt */
   140.0f,                /* 92.0f * Q_Cnt */
   150.0f,                /* 90.0f * Q_Cnt */
   160.0f,                /* 88.0f * Q_Cnt */
   170.0f,                /* 86.0f * Q_Cnt */
   180.0f,                /* 84.0f * Q_Cnt */
   190.0f,                /* 82.0f * Q_Cnt */
   200.0f                 /* 80.0f * Q_Cnt */
};
/* 内阻变化对应SOH */
/* 容量减少变化对应SOH */

/* -------------------- 三元电芯数据 ------------------ */
/* 三元电芯充电过程的单体电压与SOC对应表 */
/*const*/  unsigned short T_Charge_OCV_TableS[SOCSTURCTLONG] =
{
    3376,             /* SOC=0 */
    3698,             /* SOC=10 */
    3758,             /* SOC=20 */
    3783,             /* SOC=30 */
    3793,             /* SOC=40 */
    3819,             /* SOC=50 */
    3879,             /* SOC=60 */
    3929,             /* SOC=70 */
    3989,             /* SOC=80 */
    4067,             /* SOC=90 */
    4169              /* SOC=100 */
};
/* 三元电芯放电过程的单体电压与SOC对应表 */
/*const*/  unsigned short T_DisCharge_OCV_TableS[SOCSTURCTLONG] =
{
    3370,             /* SOC=0 */
    3692,             /* SOC=10 */
    3741,             /* SOC=20 */
    3767,             /* SOC=30 */
    3781,             /* SOC=40 */
    3804,             /* SOC=50 */
    3853,             /* SOC=60 */
    3914,             /* SOC=70 */
    3974,             /* SOC=80 */
    4051,             /* SOC=90 */
    4175              /* SOC=100 */
};
/* 三元电芯充分静置(3小时)后的单体电压与SOC对应表 */
/*const*/  unsigned short T_StopState_OCV_TableS[SOCSTURCTLONG] =
{
/* 实测数据            论文数据        SOC真实值  松下NCR18650PF 2.7Ah三元电芯OCV（1A放电，静置40分钟后测量，静置8小时以上与静置40分钟测量电压差值约为8毫伏) */
    3320,            /* 3387,         // SOC=0 */
    3426,            /* 3684,         // SOC=10 */
    3527,            /* 3737,         // SOC=20 */
    3578,            /* 3770,         // SOC=30 */
    3629,            /* 3780,         // SOC=40 */
    3714,            /* 3806,         // SOC=50 */
    3811,            /* 3865,         // SOC=60 */
    3890,            /* 3912,         // SOC=70 */
    3974,            /* 3971,         // SOC=80 */
    4071,            /* 4046,         // SOC=90 */
    4189             /* 4185          // SOC=100 */
};

/* -------------------- 磷酸铁锂电芯数据 ------------------ */
/* 磷酸铁锂电芯充电过程的单体电压与SOC对应表 */
/*const*/ unsigned short T_Charge_OCV_TableT[SOCSTURCTLONG] =
{
    3046,             /* SOC=0 */
    3172,             /* SOC=10 */
    3241,             /* SOC=20 */
    3262,             /* SOC=30 */
    3272,             /* SOC=40 */
    3279,             /* SOC=50 */
    3286,             /* SOC=60 */
    3302,             /* SOC=70 */
    3329,             /* SOC=80 */
    3386,             /* SOC=90 */
    3431              /* SOC=100 */
};
/* 磷酸铁锂电芯放电过程的单体电压与SOC对应表 */
/*const*/  unsigned short T_DisCharge_OCV_TableT[SOCSTURCTLONG] =
{
    3020,             /* SOC=0 */
    3123,             /* SOC=10 */
    3215,             /* SOC=20 */
    3245,             /* SOC=30 */
    3254,             /* SOC=40 */
    3268,             /* SOC=50 */
    3276,             /* SOC=60 */
    3285,             /* SOC=70 */
    3312,             /* SOC=80 */
    3361,             /* SOC=90 */
    3402              /* SOC=100 */
};
/* 根据以上两组平局计算所得 */
/* 磷酸铁锂电芯充分静置(3小时)后的单体电压与SOC对应表 */
/*const*/  unsigned short T_StopState_OCV_TableT[SOCSTURCTLONG] =
{
    3033,             /* SOC=0 */
    3148,             /* SOC=10 */
    3228,             /* SOC=20 */
    3254,             /* SOC=30 */
    3263,             /* SOC=40 */
    3274,             /* SOC=50 */
    3281,             /* SOC=60 */
    3307,             /* SOC=70 */
    3321,             /* SOC=80 */
    3374,             /* SOC=90 */
    3416              /* SOC=100 */
};

/* 磷酸铁锂电芯温度修正数据(锂离子电池的允许工作温度范围在-40~60度) */
/*const*/  unsigned short T_Temperature_Q_Parameter_TableT[SOCSTURCTLONG] =
{
    1030,      /* 60度时最大容量对于额定容量的千分比(103/100) */
    1030,      /* 50度时最大容量对于额定容量的千分比(103/100) */
    1030,      /* 40度时最大容量对于额定容量的千分比(103/100) */
    1030,      /* 30度时最大容量对于额定容量的千分比(103/100) */
    1030,      /* 20度时最大容量对于额定容量的千分比(103/100) */
    980,       /* 10    (98/100) */
    880,       /*  0    (88/100) */
    580,       /* -10    (58/100) */
    550,       /* -20    (55/100) */
    480,       /* -30    (48/100) */
    400        /* -40    (40/100) */
};

/* -------------------- 电芯欧姆内阻数据 ------------------ */
/* 数据来源：测试 */
/* 电芯欧姆内阻与SOC值关系表--充电，单位mΩ */
/*const*/ unsigned short T_CH_R0_SOC_TableT[SOCSTURCTLONG] =
{
    40,              /* SOC=0 */
    140,             /* SOC=10 */
    138,             /* SOC=20 */
    154,             /* SOC=30 */
    156,             /* SOC=40 */
    179,             /* SOC=50 */
    172,             /* SOC=60 */
    188,             /* SOC=70 */
    120,             /* SOC=80 */
    122,             /* SOC=90 */
    124              /* SOC=100 */
};

/* 数据来源：测试 */
/* 电芯欧姆内阻与SOC值关系表--放电，单位mΩ */
/*const*/ unsigned short T_DIS_R0_SOC_TableT[SOCSTURCTLONG] =
{
    250,             /* SOC=0 */
    220,             /* SOC=10 */
    212,             /* SOC=20 */
    184,             /* SOC=30 */
    170,             /* SOC=40 */
    168,             /* SOC=50 */
    162,             /* SOC=60 */
    154,             /* SOC=70 */
    120,             /* SOC=80 */
    122,             /* SOC=90 */
    124              /* SOC=100 */
};/* 使用时需要把毫欧转换为浮点的欧姆数 */

/**********************************************************************************************************
*函 数 名: GetFixValForQ
*功能说明: 根据当前温度值，充放电效率，老化程度等，计算电池包总容量的修正系数
*形    参: Temp 电池包温度，-40~125
*返 回 值: 修正系数，范围0~2(注意这个值与额定容量的乘积得到当前真实容量)
**********************************************************************************************************/
float GetFixValForQ(char Temp)
{
    float fTemFix;
    fTemFix =  (- 0.004488f * Temp) + 1.113f;
    fTemFix = 1/fTemFix;
    return fTemFix;
}
float GetFixValForH(float soh)
{
    return soh;
}

//float __sqrtf(float val){
//    float tmp;
//    arm_sqrt_f32(val, &tmp);
//    return tmp;
//}

/* 根据电池类型，电池状态找到具体的数据，并传回数据指针 */
/**********************************************************************************************************
*函 数 名: Get_Data_Pointer
*功能说明: 获取数据表指针，根据给定的电池类型和状态，查找对应的SOC与OCV对应表的地址指针
*形    参: cBatType为电池包类型，可以是磷酸铁锂或者三元材料，PackStat为充电，放电，静置
*返 回 值: 数据表指针
**********************************************************************************************************/
unsigned short * Get_Data_Pointer(char cBatType,char PackStat)
{
    /* 定义一个指针，根据不同的参数查找不同的表格 */
    unsigned short * usPOcv = NULL;

    /* 如果电池类型为磷酸铁锂 */
    if(cBatType == (char)BATTTYPELIN)
    {
        if(PackStat == (char)OCVTYPEDISCH)
        {
            usPOcv = (unsigned short *)T_Charge_OCV_TableT;
        }else if(PackStat == (char)OCVTYPESTATIC)
        {
            usPOcv = T_StopState_OCV_TableT;
        }else if(PackStat == (char)OCVTYPECHARGE)
        {
            usPOcv = T_DisCharge_OCV_TableT;
        }else
        {
            usPOcv = NULL;
        }
    }else if(cBatType == (char)BATTTYPESAN) /* 如果电池类型为三元材料 */
    {
        if(PackStat == (char)OCVTYPEDISCH)
        {
            usPOcv = T_Charge_OCV_TableS;
        }else if(PackStat == (char)OCVTYPESTATIC)
        {
            usPOcv = T_StopState_OCV_TableS;
        }else if(PackStat == (char)OCVTYPECHARGE)
        {
            usPOcv = T_DisCharge_OCV_TableS;
        }else
        {
            usPOcv = NULL;
        }
    }else  /* 电池类型未定义 */
    {
        usPOcv = NULL;
    }
    return usPOcv;
}

/* 在结构体中查找数据值的算法，即通过电压查找SOC */
/**********************************************************************************************************
*函 数 名: FindSOC
*功能说明: 通过给定的电压，查找对应的SOC值
*形    参: 1：数据结构指针；参数2，数组长度；参数3,电压值
*返 回 值: SOC值
**********************************************************************************************************/
short FindSOC(const unsigned short* usData_t,unsigned short n,unsigned short uCellmV)/*参数1：数据结构指针；参数2，数组长度；参数3,电压值 */
{
    unsigned short usRet=0;
    unsigned short i=0;
    /* 判断指针是否为空，如果不判断，后面的代码会溢出 */
    if(usData_t == NULL)
    {
        return -1;
    }
    /* 如果单体最低电压大于SOC为100%的电压 */
    if((uCellmV > usData_t[n - (unsigned short)1])) {
        usRet = 1000;
    }else if(uCellmV == usData_t[n - (unsigned short)1])
    {
        usRet = 1000;
    }else if(uCellmV <= usData_t[0]){  /* 如果单体最低电压小于SOC为0%的电压 */
        usRet = 0;
    }else{    /* 单体电压处于正常的电压范围内 */
        /* 寻找最接近的电压 */
        for(i = 0;i < (n - (unsigned short)1);i++)
        {
            /* 找到对应的区间 */
          if((uCellmV > usData_t[i])&&(uCellmV <= usData_t[i + (unsigned short)1]))
          {
              /* 在这里使用线性插值算法查找相应的值 */
              usRet= (i * (SOCMAXVAL / (SOCSTURCTLONG - 1)) + ((uCellmV - usData_t[i]) * (SOCMAXVAL / (SOCSTURCTLONG - 1))) / (usData_t[i + 1] - usData_t[i]));  //这个区间序号代表了SOC0_1000的值
              break;
          }
        }
    }
    return (short)usRet;
}
/* 在结构体中查找数据值的算法，即通过SOC查找电压(soc变量的范围为0~1000) */
/**********************************************************************************************************
*函 数 名: FindCellmV
*功能说明: 通过给定的SOC，查找对应的单体电压值
*形    参: 1：数据结构指针；参数2，数组长度；参数3,SOC值
*返 回 值: 单体电压值
**********************************************************************************************************/
short FindCellmV(const unsigned short* usData_t,unsigned short n,unsigned short Soc)
{
    /*反向算法 */
    unsigned short usRet = 0;
    unsigned short i,m;
    /* 判断指针是否为空，如果不判断，后面的代码会溢出 */
    if(usData_t == NULL)
    {
        return -1;
    }
    /* 此时i的值就是该SOC对应的单体电压所在结构体的位置，比如soc为85，i为0，位置在结构体的0和1之间 */
    i = Soc / ((unsigned short)SOCMAXVAL / ((unsigned short)SOCSTURCTLONG - (unsigned short)1));
    /* 此时m的值是在结构体的两个位置之间的百分比，比如soc为85，m为85，soc就是结构体1-结构体0的值乘以m，然后除以100，在加上结构体0 */
    m = Soc % ((unsigned short)SOCMAXVAL / ((unsigned short)SOCSTURCTLONG - (unsigned short)1));
    /* 如果i等于10说明SOC等于1000，即满电状态，直接返回数据结构中的最高电压值即可。*/
    if(i == (n - (unsigned short)1))
    {
        return (short)usData_t[n - (unsigned short)1];
    }else if(i > (n - (unsigned short)1))  /*如果i大于10，数据异常 */
    {
        return -2;
    }else    /* 如果i小于10并且大于等于0 */
    {
        usRet = ((usData_t[i + (unsigned short)1] - usData_t[i]) * m / (  (unsigned short)SOCMAXVAL/((unsigned short)SOCSTURCTLONG - (unsigned short)1))) + usData_t[i];  //soc所处结构体位置的高值减去低值乘以soc在这个段的比例，在加上低值，结果就是电压值。
    }

    return (short)usRet;
}

/* 在结构体中查找数据值的算法，即通过SOC查找欧姆内阻(soc变量的范围为0~1000)*/
/**********************************************************************************************************
*函 数 名: FindCellmOM
*功能说明: 通过给定的SOC，查找对应的单体欧姆内阻值
*形    参: 1：数据结构指针；参数2，数组长度；参数3,SOC值
*返 回 值: 欧姆内阻值
**********************************************************************************************************/
short FindCellmOM(const unsigned short* usData_t,unsigned short n,unsigned short Soc)
{
    return FindCellmV(usData_t,n,Soc);
}
/* ----------------------------------------------------------------------------
* 通过SOC查询单体电压值，浮点类型
* ---------------------------------------------------------------------------- */
float Get_CellV_FromSoc_f32(float fsoc)
{
    /*注意！！！在fsoc的值大于1.0的时候，由于arm_linear_interp_f32函数本身的问题，可能导致查询到的电压异常  */
    /* 所以这里需要对于fsoc的值进行判断并处理 */
    if(fsoc > MAXFSOCVAL)
    {
        return T_StopState_OCV_TableF32[SOCSTURCTLONG-1];
    }
    //arm_linear_interp_instance_f32 S = {SOCSTURCTLONG, 0, XSPACING, &T_StopState_OCV_TableF32[0]};
    /* SOC查询单体电压可行 */
    //return arm_linear_interp_f32(&S, fsoc);
}
/* ----------------------------------------------------------------------------
* 通过单体电压查询SOC值，浮点类型
* ---------------------------------------------------------------------------- */
float Get_SOC_FromCellV_f32(float fCellV)
{
    int i;
    float fret=0.0f;
    /* 单体电压差寻SOC方法：
     * 根据arm cmsis dsp函数arm_linear_interp_f32源代码中的实现方式，该函数的差值只能在x值为固定步长的情况下使用，比如SOC的取值固定为[0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
     * 这样只需要给定最小值0，步长值0.1即可。
     * 如果通过单体电压查询SOC，对应关系中x没有固定步长，所以不能实现该方式查询。*/

    /* 如果单体电压值小于等于soc为0时的电压，那么SOC返回最小值，也就是0.0 */
    if(fCellV <= T_StopState_OCV_TableF32[0])
    {
        return MINSOCVAL;
    }else if(fCellV >= T_StopState_OCV_TableF32[SOCSTURCTLONG-1])/* 如果单体电压值大于等于SOC为最高值所对应的电压，那么SOC返回最大值，也就是1.0 */
    {
        return MAXFSOCVAL;
    }else  /* 如果单体电压大于OCV对应SOC表格的最小值并且小于最大值，也就是在正常范围内。 */
    {
        for(i=0;i<(SOCSTURCTLONG-1);i++)            /* 寻找最接近的电压 */
        {
          if((fCellV > T_StopState_OCV_TableF32[i])&&(fCellV <= T_StopState_OCV_TableF32[i+1]))    /* 一定能找到对应的区间 */
          {
              /* 在这里使用线性插值算法查找相应的值 ,这个区间序号代表了SOC0_1000的值  */
              fret = (i*(MAXFSOCVAL/(SOCSTURCTLONG-1))+((fCellV-T_StopState_OCV_TableF32[i])*(MAXFSOCVAL/(SOCSTURCTLONG-1)))/(T_StopState_OCV_TableF32[i+1]-T_StopState_OCV_TableF32[i]));

              break;
          }
        }
    }
    return fret;
}

/* 为开路电压和SOC互相查询定义数据结构
 * 数据结构采用一个N行两列的二维数组。
 * 第一列保存SOC数列
 * 第二列保存SOC对应的开路电压值。
 * */
float fsoc_ocv[SOCSTURCTLONG][2]={
    {0.0f, 3.320f},
    {0.1f, 3.436f},
    {0.2f, 3.537f},
    {0.3f, 3.593f},
    {0.4f, 3.629f},
    {0.5f, 3.714f},
    {0.6f, 3.811f},
    {0.7f, 3.890f},
    {0.8f, 3.974f},
    {0.9f, 4.071f},
    {1.0f, 4.168f}};
    /*soc<==>电压*/

/* 功能：通过给定的SOC值，在数据结构中查询对应的开路电压值
 *
 *       a-----------c-----b
 * 结构：|           |     |
 *       A-----------C-----B
 *       大         当前   小
 *
 *            c - b
 * 公式：C = ------- × (A - B) + B
 *            a - b
 *
 * 含义：C的值等于c在a~b中的比例乘以A~B的大小，在加上B的基础值。
 * */
float GetCellVFromSOC(float fsoc) {
    int i;
    float fret;
    //①判断给定的SOC值是否小于数组中最小的SOC值，如果是则返回最小SOC对应的电压值或异常值。
    if (fsoc < fsoc_ocv[0][0]) {
        return  -1.0f;//fsoc_ocv[0][1];
    }
    /* ②判断给定的SOC值是否大于数组中最大的SOC值，如果是则返回最大SOC对应的电压值或异常值*/
    if (fsoc > fsoc_ocv[SOCSTURCTLONG -1][0]) {
        return  -2.0f;//fsoc_ocv[10][1];
    }
    /* ③通过循环查找，找到该SOC处于数据结构中的那一段区间 */
    for (i = 0; i < (SOCSTURCTLONG - 1); i++) {
        if ((fsoc >= fsoc_ocv[i][0])&&(fsoc <= fsoc_ocv[i+1][0])) {
            /* ④在该区间内根据公式计算SOC对应的电压值 */
            fret =  (fsoc - fsoc_ocv[i][0])/(fsoc_ocv[i+1][0] - fsoc_ocv[i][0]) * (fsoc_ocv[i+1][1] - fsoc_ocv[i][1]) + fsoc_ocv[i][1];
            break;
        }
    }

    return fret;
}

/* 功能：通过给定的开路电压值，在数据结构中查询对应的SOC值
 *
 *       a-----------c-----b
 * 结构：|           |     |
 *       A-----------C-----B
 *       大         当前   小
 *
 *            C - B
 * 公式：c = ------- × (a - b) + b
 *            A - B
 * 含义：c的值等于C在A~B中的比例乘以a~b的大小，在加上b的基础值。
 * */
float GetSocFromCellV(float fCellV) {
    int i;
    float fret;
    /* ①判断给定的开路电压值是否小于数组中最小的电压值，如果是则返回最小电压对应的SOC值 */
    if (fCellV < fsoc_ocv[0][1]) {
        return  fsoc_ocv[0][0];
    }
    /* ②判断给定的开路电压值是否大于数组中最大的电压值，如果是则返回最大电压对应的SOC值 */
    if (fCellV > fsoc_ocv[SOCSTURCTLONG -1][1]) {
        return  fsoc_ocv[10][0];
    }
    /* ③通过循环查找，找到该开路电压处于数据结构中的那一段区间 */
    for (i = 0; i < (SOCSTURCTLONG - 1); i++) {
        if ((fCellV >= fsoc_ocv[i][1])&&(fCellV <= fsoc_ocv[i+1][1])) {
            /* ④在该区间内根据公式计算电压对应的SOC值 */
            fret =  (fCellV - fsoc_ocv[i][1])/(fsoc_ocv[i+1][1] - fsoc_ocv[i][1]) * (fsoc_ocv[i+1][0] - fsoc_ocv[i][0]) + fsoc_ocv[i][0];
            break;
        }
    }

    return fret;
}
