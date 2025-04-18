
#include "include.h"

//---------------------------------------------------------
volatile SysErrDef		g_SystemError;			// 记录系统错误信息
volatile SysWarningDef 	g_SystemWarning;		// 记录系统警告信息


CurrWarnClsDef 	g_BattCOCThr;		// 电池包充电过流
CurrWarnClsDef 	g_BattDOCThr;		// 放电过流
VoltWarnClsDef 	g_PackOVThr;
VoltWarnClsDef 	g_PackUVThr;

VoltWarnClsDef 	g_IsoThr;   	// 绝缘等级参数设置

//---------------------------------------------------------
VoltWarnClsDef 	g_CellOVThr;
VoltWarnClsDef 	g_CellUVThr;
VoltWarnClsDef 	g_CellIBMThr;	// 单体一致性

TempWarnClsDef 	g_PACKCOTThr;	// 电池包充电高温
TempWarnClsDef 	g_PACKCUTThr;	// 电池包充电低温
TempWarnClsDef 	g_PACKDOTThr;	// 电池包放电高温
TempWarnClsDef 	g_PACKDUTThr;	// 电池包放电低温
TempWarnClsDef 	g_PACKDLTThr;	// 电池包温差大告警

// 电池是否处于充电检测相关参数
static uint8_t chgrChkTimer = 0;
static uint8_t keyChkTimer = 0;

// 0-位表示是否有充电器接入
// 1-位表示是否有keyrun信号
static uint8_t keyChgrState = 0;

//============================================================================
// Function    : DetectPackOverCurrent
// Description : 电池包过流检测（包括放电过流和充电过流），
// 				 执行周期2Ms
// Parameters  : none
// Returns     : 
//============================================================================
void DetectPackOverCurrent(void)
{
	static uint16_t ocErrCnt = 0;
	// 已经设置过二级报警的就直接返回，等待保护时间到断开继电器即可
	// 充电过流
	if (WARNING_SECOND_LEVEL == g_SystemWarning.COC)
	{
		return;
	}
	
	// 放电过流
	if (WARNING_SECOND_LEVEL == g_SystemWarning.DOC)
	{
		return;
	}
	
	if (CHARGE == g_BatteryMode)
	{
		if (g_BatteryParameter.current < g_BattCOCThr.cls_2)
		{
			if (ocErrCnt > PACK_COC_FAULT_DLY) // 持续5S
			{
				g_SystemWarning.COC = WARNING_SECOND_LEVEL;
				if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_5S)
				{
					g_ProtectDelayCnt = RELAY_ACTION_DELAY_5S;
				}
			}
			else
			{
				ocErrCnt++;
			}
		} 
		else if (g_BatteryParameter.current < g_BattCOCThr.cls_1)
		{
			if (ocErrCnt > PACK_COC_WARNING_DLY) // 持续5s
			{
				g_SystemWarning.COC = WARNING_FIRST_LEVEL;
				ocErrCnt = 0;
			}
			else
			{
				ocErrCnt++;
			}
		}
		else
		{
			g_SystemWarning.COC = 0;
			ocErrCnt = 0;
		}
	}
	else if(DISCHARGE == g_BatteryMode )// 放电情况
	{
		if (g_BatteryParameter.current > g_BattDOCThr.cls_2)
		{
			if (ocErrCnt > PACK_DOC_FAULT_DLY) // 持续500Ms
			{
				g_SystemWarning.DOC = WARNING_SECOND_LEVEL;
				if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_3S)
				{
					g_ProtectDelayCnt = RELAY_ACTION_DELAY_3S;
				}
			}
			else
			{
				ocErrCnt++;
			}
		}
		else if (g_BatteryParameter.current > g_BattDOCThr.cls_1)
		{
			if (ocErrCnt > PACK_DOC_WARNING_DLY) // 持续500Ms
			{
				ocErrCnt = 0;
				g_SystemWarning.DOC = WARNING_FIRST_LEVEL;
			}
			else
			{
				ocErrCnt++;
			}
		}
		else
		{
			g_SystemWarning.DOC = 0;
			ocErrCnt = 0;
		}
	}
}


//============================================================================
// Function    : DetectMaxMinCellTemp
// Description : 该函数检测最大最小单体温度值，并获取平均温度和最大最小温度位置
// Parameters  : none
// Returns     : 
//============================================================================
void DetectMaxMinCellTemp(void)
{
	uint8_t i;
	int8_t maxTemp = -40;
	int8_t minTemp = 125;
	int16_t avgTemp = 0;
    

	for (i = 0; i < MAX_TEMP_SENSOR; i++)
	{
		if (maxTemp < g_BatteryParameter.CellTemp[i])
		{
			maxTemp = g_BatteryParameter.CellTemp[i];
			g_BatteryParameter.MaxTempChnIdx = i+1;
		}

		if (minTemp > g_BatteryParameter.CellTemp[i])
		{
			minTemp = g_BatteryParameter.CellTemp[i];
			g_BatteryParameter.MinTempChnIdx = i+1;   
		}
		avgTemp += g_BatteryParameter.CellTemp[i];
	}

	g_BatteryParameter.CellTempMax = maxTemp;
	g_BatteryParameter.CellTempMin = minTemp;
	g_BatteryParameter.CellTempAvg = avgTemp / MAX_TEMP_SENSOR;
}

//============================================================================
// Function    : DetectCellsOverTemp
// Description : 检测温度过高,执行周期40Ms
// Parameters  : none
// Returns     : 
//============================================================================
void DetectCellsOverTemp(void)
{
	static uint8_t otErrCnt = 0;

	if(WARNING_SECOND_LEVEL == g_SystemWarning.COT)
	{
		if(g_BatteryParameter.CellTempMax < g_PACKCOTThr.cls_1)
		{
			otErrCnt++;
			if (otErrCnt > CELL_COT_FAULT_DLY) // 持续5S
			{
				otErrCnt = 0;
				g_SystemWarning.COT = 0;
			}	
		}
		else
		{
			otErrCnt = 0;
		}
		return;
	}
	if(WARNING_SECOND_LEVEL == g_SystemWarning.DOT)
	{
		if (g_BatteryParameter.CellTempMax < g_PACKDOTThr.cls_1)
		{ 
			otErrCnt++;
			if (otErrCnt > CELL_DOT_FAULT_DLY) // 持续5S
			{
				otErrCnt = 0;
				g_SystemWarning.DOT = 0;
			}	
		}
		else
		{
			otErrCnt = 0;
		}
		return;
	}

	if(CHARGE == g_BatteryMode)
	{
		if(g_BatteryParameter.CellTempMax > g_PACKCOTThr.cls_2)
		{
			if(otErrCnt > CELL_COT_FAULT_DLY) // 持续5S
			{
				g_SystemWarning.COT = WARNING_SECOND_LEVEL;
				if(g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
				{
					g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
				}
			}
			else
			{
				otErrCnt++;
			}
		}
		else if(g_BatteryParameter.CellTempMax > g_PACKCOTThr.cls_1)
		{
			if (otErrCnt > CELL_COT_WARNING_DLY)
			{
				g_SystemWarning.COT = WARNING_FIRST_LEVEL;
				otErrCnt = 0;
			}
			else
			{
				otErrCnt++;
			}
		}
		else
		{
			otErrCnt = 0;
			g_SystemWarning.COT = 0;
		}
	}
	else if(DISCHARGE == g_BatteryMode)
	{
		if(g_BatteryParameter.CellTempMax > g_PACKDOTThr.cls_2)
		{   
			if(otErrCnt > CELL_DOT_FAULT_DLY) // 持续5S
			{
				g_SystemWarning.DOT = WARNING_SECOND_LEVEL;
				if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
				{
					g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
				}
			}
			else
			{
				otErrCnt++;
			}
		}
		else if(g_BatteryParameter.CellTempMax > g_PACKDOTThr.cls_1)
		{
			if(otErrCnt > CELL_DOT_WARNING_DLY) // 持续5S
			{
				g_SystemWarning.DOT = WARNING_FIRST_LEVEL;
				otErrCnt = 0;
			}
			else
			{
				otErrCnt++;
			}
		}
		else
		{
			otErrCnt = 0;
			g_SystemWarning.DOT = 0;
		}
	}
}

//============================================================================
// Function    : DetectCellsUnderTemp
// Description : 检测温度是否过低 执行周期40Ms
// Parameters  : none
// Returns     : 
//============================================================================
void DetectCellsUnderTemp(void)
{
	static uint8_t utErrCnt = 0;

	if(WARNING_SECOND_LEVEL == g_SystemWarning.CUT)
	{
		if(g_BatteryParameter.CellTempMin > g_PACKCUTThr.cls_1)
		{       
			if(utErrCnt > CELL_CUT_FAULT_DLY) // 持续5S
			{
				utErrCnt = 0;
				g_SystemWarning.CUT = 0;
			}
			else
			{
				utErrCnt++;
			}
		}
		return;
	}
	if(WARNING_SECOND_LEVEL == g_SystemWarning.DUT)
	{
		if(g_BatteryParameter.CellTempMin > g_PACKDUTThr.cls_1)
		{
			if(utErrCnt > CELL_DUT_FAULT_DLY) // 持续5S
			{
				utErrCnt = 0;
				g_SystemWarning.DUT = 0;
			}
			else
			{
				utErrCnt++;
			}
		}
		return;
	}

	if(CHARGE == g_BatteryMode)
	{
		if(g_BatteryParameter.CellTempMin < g_PACKCUTThr.cls_2)
		{       
			if(utErrCnt > CELL_CUT_FAULT_DLY) // 持续5S
			{
				g_SystemWarning.CUT = WARNING_SECOND_LEVEL;
				if(g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
				{
					g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
				}
			}
			else
			{
				utErrCnt++;
			}
		}
		else if(g_BatteryParameter.CellTempMin < g_PACKCUTThr.cls_1)
		{
			if(utErrCnt > CELL_CUT_WARNING_DLY) // 持续5S
			{
				utErrCnt= 0;
				g_SystemWarning.CUT = WARNING_FIRST_LEVEL;
			}
			else
			{
				utErrCnt++;
			}
		}
		else
		{
			utErrCnt = 0;
			g_SystemWarning.CUT = 0;
		}
	}
	else if(DISCHARGE == g_BatteryMode)
	{
		if(g_BatteryParameter.CellTempMin < g_PACKDUTThr.cls_2)
		{

			if(utErrCnt > CELL_DUT_FAULT_DLY) // 持续5S
			{
				g_SystemWarning.DUT = WARNING_SECOND_LEVEL;
				if(g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
				{
					g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
				}
			}
			else
			{
				utErrCnt++;
			}
		}
		else if(g_BatteryParameter.CellTempMin < g_PACKDUTThr.cls_1)
		{
			if(utErrCnt > CELL_DUT_WARNING_DLY)
			{
				utErrCnt = 0;
				g_SystemWarning.DUT = WARNING_FIRST_LEVEL;
			}
			else
			{
				utErrCnt++;
			}
		}
		else
		{
			utErrCnt = 0;
			g_SystemWarning.DUT = 0;
		}
	}
}

#if 0
// 检测PCB板子的温度
void DetectPCBOverTemp(void)
{
	static uint8_t otErrCnt = 0;

	if(g_SystemWarning.PCBOT == WARNING_SECOND_LEVEL)
	{
		if(g_BatteryParameter.AmbientTemp < PcbOverTempValue)
		{
			otErrCnt++;
			if(otErrCnt >= 5)
			{
				otErrCnt = 0;
				g_SystemWarning.PCBOT = 0;
			}
		}
	}
	else
	{
		if(g_BatteryParameter.AmbientTemp >= PcbOverTempValue)
		{
			otErrCnt++;
			if(otErrCnt >= 5)
			{
				otErrCnt = 0;
				g_SystemWarning.PCBOT = WARNING_SECOND_LEVEL;
				if(g_ProtectDelayCnt > RELAY_ACTION_DELAY_1S)
				{
					g_ProtectDelayCnt = RELAY_ACTION_DELAY_1S;
				}
			}
			else
			{
				otErrCnt++;
			}
		}
		else
		{
			otErrCnt = 0;
			g_SystemWarning.PCBOT = 0;
		}
	}
}
#endif

//============================================================================
// Function    : DetectCellsUnderTemp
// Description : 检测温度差 执行周期40Ms
// Parameters  : none
// Returns     : 
//============================================================================
void DetectCellTempDlt(void)
{  
	static uint8_t tImbErrCnt = 0;
	int16_t temp = 0;

	temp = g_BatteryParameter.CellTempMax - g_BatteryParameter.CellTempMin;
	if (WARNING_SECOND_LEVEL == g_SystemWarning.TDIF)
	{
		if(temp < g_PACKDLTThr.cls_1)
		{
			if(tImbErrCnt > CELL_TIB_FAULT_DLY) //持续5S
			{
				tImbErrCnt = 0;
				g_SystemWarning.TDIF = 0;
			}
			else
			{
				tImbErrCnt ++;
			}
		}
		else
		{
			tImbErrCnt = 0;
		}
		return;
	}

	if(temp > g_PACKDLTThr.cls_2)
	{
		if (tImbErrCnt > CELL_TIB_FAULT_DLY)//持续5S 
		{
			g_SystemWarning.TDIF = WARNING_SECOND_LEVEL;
			if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
			{
				g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
			}
		}
		else
		{
			tImbErrCnt++;
		}
	}
	else if(temp > g_PACKDLTThr.cls_1)
	{
		if (tImbErrCnt > CELL_TIB_WARNING_DLY) //持续5S
		{
			tImbErrCnt = 0;
			g_SystemWarning.TDIF = WARNING_FIRST_LEVEL;
		}
		else
		{
			tImbErrCnt++;
		}
	}
	else
	{
		g_SystemWarning.TDIF = 0;
		tImbErrCnt = 0;
	}
}



//============================================================================
// Function    : DetectCellsOverVolt
// Description : 检测单体电池过压 执行周期60Ms
// Parameters  : none
// Returns     : none
//============================================================================
void DetectCellsOverVolt(void)
{
	static uint8_t ovErrCnt = 0;

	if (WARNING_SECOND_LEVEL == g_SystemWarning.COV)
	{    
		return;
	}

	if(g_BatteryParameter.CellVoltMax > g_CellOVThr.cls_2)
	{
		if (ovErrCnt > CELL_OV_FAULT_DLY) // 持续5S
		{
			g_SystemWarning.COV = WARNING_SECOND_LEVEL;
			if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_5S)
			{
				g_ProtectDelayCnt = RELAY_ACTION_DELAY_5S;
			}
		}
		else
		{
			ovErrCnt++;
		}
	}
	else if(g_BatteryParameter.CellVoltMax > g_CellOVThr.cls_1)
	{
		if (ovErrCnt > CELL_OV_WARNING_DLY) // 持续5S
		{
			ovErrCnt = 0;
			g_SystemWarning.COV = WARNING_FIRST_LEVEL;
		}
		else
		{
			ovErrCnt++;
		}
	}
	else
	{
		g_SystemWarning.COV = 0;
		ovErrCnt = 0;
	}
}


//============================================================================
// Function    : DetectCellsUnderVolt
// Description : 检测单体电池欠压 执行周期60Ms
// Parameters  : none
// Returns     : none
//============================================================================
void DetectCellsUnderVolt(void)
{
	static uint8_t uvErrCnt = 0;

	if (WARNING_SECOND_LEVEL == g_SystemWarning.CUV)
	{
		return;
	}

	if(g_BatteryParameter.CellVoltMin < g_CellUVThr.cls_2)
	{
		if (uvErrCnt > CELL_UV_FAULT_DLY) // 持续5S
		{
			if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_20S)
			{
			    g_ProtectDelayCnt = RELAY_ACTION_DELAY_20S;
			}
			g_SystemWarning.CUV = WARNING_SECOND_LEVEL;
		}
		else
		{
			uvErrCnt++;
		}
	}
		
	else if(g_BatteryParameter.CellVoltMin < g_CellUVThr.cls_1)
	{
		if (uvErrCnt > CELL_UV_WARNING_DLY) // 持续5S
		{
			g_SystemWarning.CUV = WARNING_FIRST_LEVEL;
			uvErrCnt = 0;
		}
		else
		{
			uvErrCnt++;
		}
	}
	else
	{
		g_SystemWarning.CUV = 0;
		uvErrCnt = 0;
	}
}

//============================================================================
// Function    : DetectCellsVoltImba
// Description : 检测单体电压不一致性 执行周期60Ms
// Parameters  : none
// Returns     : none
//============================================================================
void DetectCellsVoltImba(void)
{
	static uint8_t imbErrCnt = 0;
	uint16_t temp;

   	temp = g_BatteryParameter.CellVoltMax - g_BatteryParameter.CellVoltMin;

	if (WARNING_SECOND_LEVEL == g_SystemWarning.VDIF)
	{
		return;
	}
	
	if(temp > g_CellIBMThr.cls_2)
	{
		if (imbErrCnt > CELL_IB_FAULT_DLY)  // 持续5S
		{
			g_SystemWarning.VDIF = WARNING_SECOND_LEVEL;
			if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
			{
				g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
			}
		}
		else
		{
			imbErrCnt++;
		}
	}
	else if(temp > g_CellIBMThr.cls_1)
	{
		if (imbErrCnt > CELL_IB_WARNING_DLY) // 持续5S
		{
			imbErrCnt = 0;
			g_SystemWarning.VDIF = WARNING_FIRST_LEVEL;
		}
		else
		{
			imbErrCnt++;
		}
	}
	else
	{
		g_SystemWarning.VDIF = 0;
		imbErrCnt = 0;
	}
}

//电池包的过压检测 执行周期60Ms
void DetectPackOv(void)
{
	static uint8_t pOvErrCnt = 0;

	if (WARNING_SECOND_LEVEL == g_SystemWarning.POV)
	{
		return;
	}

	if(g_BatteryParameter.voltage > g_PackOVThr.cls_2)
	{
		if (pOvErrCnt > PACK_OV_FAULT_DLY) // 持续5S
		{
			g_SystemWarning.POV = WARNING_SECOND_LEVEL;
			if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
			{
				g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
			}
		}
		else
		{
			pOvErrCnt++;
		}
	}
	else if(g_BatteryParameter.voltage > g_PackOVThr.cls_1)
	{
		if (pOvErrCnt > PACK_OV_WARNING_DLY) // 持续5S
		{
			pOvErrCnt = 0;
			g_SystemWarning.POV = WARNING_FIRST_LEVEL;
		}
		else
		{
			pOvErrCnt++;
		}
	}
	else
	{
		g_SystemWarning.POV = 0;
		pOvErrCnt = 0;
	}    
}


//电池包的欠压检测 执行周期60Ms
void DetectPackUv(void)
{
	static uint8_t pUvErrCnt = 0;

	if (WARNING_SECOND_LEVEL == g_SystemWarning.PUV)
	{
		return;
	}

	if(g_BatteryParameter.voltage < g_PackUVThr.cls_2)
	{
		if (pUvErrCnt > PACK_UV_FAULT_DLY) // 持续5S
		{
			g_SystemWarning.PUV = WARNING_SECOND_LEVEL;
			if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_10S)
			{
				g_ProtectDelayCnt = RELAY_ACTION_DELAY_10S;
			}
		}
		else
		{
			pUvErrCnt++;
		}
	}
	else if(g_BatteryParameter.voltage < g_PackUVThr.cls_1)
	{
		if (pUvErrCnt > PACK_UV_WARNING_DLY) // 持5S
		{
			pUvErrCnt = 0;
			g_SystemWarning.PUV = WARNING_FIRST_LEVEL;
		}
		else
		{
			pUvErrCnt++;
		}
	}
	else
	{
		g_SystemWarning.PUV = 0;
		pUvErrCnt = 0;
	}    
}


//============================================================================
// Function    : PackChargeFinish
// Description : 检测电池包是否达到充电截止电压
// Parameters  : none
// Returns     : 1:充电完成，0:未完成
//============================================================================
uint8_t DetectPackChargeFinish(void)
{
	static uint16_t chgEndTimer = 0;

	// 注意,选用的充电器如果电流不稳定需要修改此行代码
	if ((g_BatteryParameter.CellVoltMax > g_CellOVThr.cls_2)
		|| (g_BatteryParameter.voltage > g_PackOVThr.cls_2))
	{
		if ( chgEndTimer++ >= 10 ) 
		{
			return 1;
		}
	}
	else 
	{
		chgEndTimer = 0;
		return 0;    
	}
}


//============================================================================
// Function    : GetMaxMinAvgCellVolt
// Description : 获取电芯最大最小电压值，电芯平均电压值
// Parameters  : none
// Returns     : none
//============================================================================
void DetectMaxMinAvgCellVolt(void)
{
	uint8_t i,j;
	uint16_t maxVolt = 0;
	uint16_t minVolt = 5000;
	uint32_t sumVolt = 0;

	for(i=0; i<ModuleAmount; i++)
	{
		for(j=0; j<(CellsAmount/ModuleAmount); j++)
		{
			if(maxVolt < g_ArrayLtc6803Unit[i].CellVolt[j])
			{
				maxVolt = g_ArrayLtc6803Unit[i].CellVolt[j];
				g_BatteryParameter.MaxCellNum = i * CellsAmount / ModuleAmount + j;
			}

			if(minVolt > g_ArrayLtc6803Unit[i].CellVolt[j])
			{
				minVolt = g_ArrayLtc6803Unit[i].CellVolt[j];
				g_BatteryParameter.MinCellNum = i * CellsAmount / ModuleAmount + j;
			}

			sumVolt += g_ArrayLtc6803Unit[i].CellVolt[j];
		}
	}

	g_BatteryParameter.CellVoltMax = maxVolt;
	g_BatteryParameter.CellVoltMin = minVolt;
	g_BatteryParameter.voltage = sumVolt / 100;
	g_BatteryParameter.CellVoltAvg = (uint16_t)(sumVolt / CellsAmount);   
}


//============================================================================
// Function    ：TskChargerCheck
// Description ：检测电池包是否处于充电状态
// Parameters  ：none 
// Returns     ：
//============================================================================
void DetectCharger(void)
{
	if (!PORTDbits.RD7) // 有充电器接入
	{
		if (++chgrChkTimer > CHGR_CHK_CYCLE)
		{
			keyChgrState |= 0x01;
			chgrChkTimer = CHGR_CHK_CYCLE;
		}
	}
	else  // 没有充电器接入
	{
		if (keyChgrState & 0x01)    
		{
			if( !chgrChkTimer--)
			{
				keyChgrState &= ~0x01;
			}
		} 
		else
		{
			keyChgrState &= ~0x01;
		}
	} 
}

#if 0
//============================================================================
// Function    ：TskRunkeyCheck
// Description ：检测点火钥匙状态
// Parameters  ：none 
// Returns     ：
//============================================================================
void DetectRunkey(void)
{
	if (PORTDbits.RD4) // keyrun接通
	{
		if (++keyChkTimer > CHGR_CHK_CYCLE)
		{
			keyChgrState |= 0x02;
			keyChkTimer = CHGR_CHK_CYCLE;
		}
	}
	else
	{
		if (keyChgrState & 0x02)    
		{
			if( !keyChkTimer--)
			{
				keyChgrState &= ~0x02;
			}
		} 
		else
		{
			keyChgrState &= ~0x02;
		}
	}       
}
#endif

//============================================================================
// Function    ：GetChargeState
// Description ：获取电池包是否处于充电状态
// Parameters  ：none 
// Returns     ：1,charge signal effective; 0, no charge
//============================================================================
uint8_t GetChargeState(void)
{
	return (keyChgrState & 0x01);
    //return (!PORTDbits.RD7);
}

#if 0
//============================================================================
// Function    ：GetKeyrunState
// Description ：获取点火钥匙状态
// Parameters  ：none 
// Returns     ：
//============================================================================
uint8_t GetKeyrunState(void)
{
   return (keyChgrState & 0x02);
}
#endif

