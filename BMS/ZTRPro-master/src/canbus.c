//============================================================================
//
//              Copyright (c) 2013, Globetools. All rights reserved.
//
//
// FileName    ：TaskCan.c
// Description ：BMS与外部设备（如上位机、driver等）的通信处理。 
// Versions    ：V1.0
// Author      ：
// Date        ：
// History     ：
// Comment     ：
//============================================================================

#include "include.h"

static volatile CanMsgBufTypedef g_CanMsgBuf; 

static uint8_t 	tduCanId = 0;		// 温度模块CAN ID
static uint16_t canMcsTimeout = 0;	// MCS 电机控制系统时间超时变量
static uint16_t canCcsTimeout = 0;	// CCS 充电器控制系统时间超时变量
static uint16_t canTduTimeout = 0;	// TDU 温度模块时间超时变量
static uint16_t brdTxTimer = 0;		// 广播时间参数

uint16_t g_CCS_MaxVoltage = 0;		// 控制充电器的最大电压 0.1V
uint16_t g_CCS_MaxCurrent = 0;		// 控制充电器的最大电流 0.1A


//----------------------------------------------------------------------------
// Function    : TskCan_Init
// Description : 上层 CAN 参数初始化
// Parameters  : none
// Returns     : none
//----------------------------------------------------------------------------
void TskCan_Init(void)
{
	g_CanMsgBuf.TxBuf_Wptr = 0;
	g_CanMsgBuf.TxBuf_Rptr = 0;
	g_CanMsgBuf.RxBuf_Wptr = 0;
	g_CanMsgBuf.RxBuf_Rptr = 0;

 	// modify by zhuyanliang20160822
	tduCanId = TDU;
	
	brdTxTimer = 0; 
	canMcsTimeout = 0;
	canCcsTimeout = 0;
	canTduTimeout = 0;
}



//----------------------------------------------------------------------------
// Function    ：CAN_ExtractSourceAddress
// Description ：从CAN ID中提取发送节点的源地址.
// Parameters  ：can_id - can的ID
// Returns     ：源地址
//----------------------------------------------------------------------------
uint8_t CAN_GetSourceAddr(uint32_t can_id)
{
   return ((uint8_t)can_id);
}

//----------------------------------------------------------------------------
// Function    ：CAN_ExtractFunctionCode
// Description ：从CAN ID中提取功能码.
// Parameters  ：can_id - can的ID
// Returns     ：功能码
//----------------------------------------------------------------------------
uint8_t CAN_GetFuncCode(uint32_t can_id)
{
	uint8_t temp;

	temp = (uint8_t)(can_id >> 16);
	return temp;
}

//----------------------------------------------------------------------------
// Function    ：CAN_GenerateID
// Description ：根据目标地址和功能码产生CAN ID
// Parameters  ：
// Returns     ：
//----------------------------------------------------------------------------
uint32_t CAN_GenerateID(uint8_t des_addr, uint8_t msg_fc)
{
	uint32_t temp = 0;

	temp = ((uint32_t)0x18 << 24) + ((uint32_t)msg_fc << 16) + 
		((uint32_t)des_addr << 8) + BMS;
	return temp;
}

//----------------------------------------------------------------------------
// Function    : CAN_IsTxBufFull
// Description : 检查分配的CAN发送缓冲区是否满
// Parameters  : 
// Returns     : 1 -- 缓冲区满  0 -- 缓冲区未满
//----------------------------------------------------------------------------
BOOL CAN_IsTxBufFull(void)
{
#if 0
	if (((g_CanMsgBuf.TxBuf_Wptr == (CAN_BUF_DEEP - 1))&&(g_CanMsgBuf.TxBuf_Rptr == 0))
	|| (g_CanMsgBuf.TxBuf_Rptr == (g_CanMsgBuf.TxBuf_Wptr + 1)))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
#endif
	if((g_CanMsgBuf.TxBuf_Wptr+1)%CAN_BUF_DEEP == g_CanMsgBuf.TxBuf_Rptr)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}

//----------------------------------------------------------------------------
// Function    : CAN_IsTxBufEmpty
// Description : 检查CAN发送缓冲区是否为空
// Parameters  : 
// Returns     : 1 -- 缓冲区空  0 -- 缓冲区非空
//----------------------------------------------------------------------------
BOOL CAN_IsTxBufEmpty(void)
{
	if (g_CanMsgBuf.TxBuf_Rptr == g_CanMsgBuf.TxBuf_Wptr)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

//----------------------------------------------------------------------------
// Function    : CAN_IsRxBufFull
// Description : 检查分配的CAN 接收缓冲区是否满 
// Parameters  : 
// Returns     : 1 -- 满   0 -- 未满
//----------------------------------------------------------------------------
BOOL CAN_IsRxBufFull(void)
{
#if 0
	if (((g_CanMsgBuf.RxBuf_Wptr == (CAN_BUF_DEEP - 1))&&(g_CanMsgBuf.RxBuf_Rptr == 0))
		|| (g_CanMsgBuf.RxBuf_Rptr == (g_CanMsgBuf.RxBuf_Wptr + 1)))
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
#endif 
	if((g_CanMsgBuf.RxBuf_Wptr+1)%CAN_BUF_DEEP == g_CanMsgBuf.RxBuf_Rptr)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

//----------------------------------------------------------------------------
// Function    : CAN_IsRxBufEmpty
// Description : 检查分配的CAN 接收缓冲区是否空
// Parameters  : 
// Returns     : 1 -- 缓冲区空  0 -- 缓冲区非空
//----------------------------------------------------------------------------
BOOL CAN_IsRxBufEmpty(void)
{
	if (g_CanMsgBuf.RxBuf_Rptr == g_CanMsgBuf.RxBuf_Wptr)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/* 
 * 将电池包的警告信息放入发送缓冲区
 * 
 */
void CAN_PutBattWarnErrorToTxBuf(void)
{
	if(CAN_IsTxBufFull())
		return;
		
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_BATT_WARN);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint32_t *)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_SystemWarning.all;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0;

	if ( g_SystemError.ltc_st || g_SystemError.ltc_com || g_SystemError.det_oc)
	{
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] |= 0x01;
	}
	if (g_SystemError.mcs_comm) 
	{
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] |= (0x01 << 1);
	}
	if (g_SystemError.ccs_comm)
	{
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] |= (0x01 << 2);
	}
	if (g_SystemError.tdu_comm)
	{
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] |= (0x01 << 3);
	}
	if (g_SystemError.nbr_comm)
	{
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] |= (0x01 << 4);
	}
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}	

/*
 * 将电池包的电压，电流，SOC，主继电器信息放入发送缓冲区
 * 
 */
void CAN_PutBattInfoToBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_BATT_INFO);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.voltage;
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint16_t)g_BatteryParameter.current;
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_BatteryParameter.SOC;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = g_RelayActFlg.positive;
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

/*
 * 将电芯最大最小电压值，位置放入发送缓冲区
 */
void CAN_PutCellVoltPosToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_CELL_VOLT);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.CellVoltMax;
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_BatteryParameter.CellVoltMin;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_BatteryParameter.MaxCellNum+1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = g_BatteryParameter.MinCellNum+1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}	

/*
 * 将电芯最大最小温度值，位置放入发送缓冲区
 */
void CAN_PutCellTempPosToTxBuf(void)
{
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_CELL_TEMP);
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.CellTempMax;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = g_BatteryParameter.CellTempMin;;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_BatteryParameter.MaxTempChnIdx;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = g_BatteryParameter.MinTempChnIdx;

    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

   if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
   {
      g_CanMsgBuf.TxBuf_Wptr = 0;
   }
}	


//----------------------------------------------------------------------------
// Function    : CAN_CellVoltage1ToTxBuf
// Description : 将单体电芯电压值 1~4 放入发送缓冲区
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_CellVoltage1ToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_VOLT_GRP1);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_ArrayLtc6803Unit[0].CellVolt[0];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_ArrayLtc6803Unit[0].CellVolt[1];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_ArrayLtc6803Unit[0].CellVolt[2];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_ArrayLtc6803Unit[0].CellVolt[3];

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//----------------------------------------------------------------------------
// Function    : CAN_CellVoltage2ToTxBuf
// Description :  将单体电芯电压值 5~8 放入发送缓冲区
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_CellVoltage2ToTxBuf(void)
{
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_VOLT_GRP2);
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_ArrayLtc6803Unit[0].CellVolt[4];
   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_ArrayLtc6803Unit[0].CellVolt[5];
   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_ArrayLtc6803Unit[0].CellVolt[6];
   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_ArrayLtc6803Unit[0].CellVolt[7];

   if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
   {
      g_CanMsgBuf.TxBuf_Wptr = 0;
   }
}

//----------------------------------------------------------------------------
// Function    : CAN_CellVoltage3ToTxBuf
// Description :  将单体电芯电压值 9~12 放入发送缓冲区
// Parameters  : none
// Returns     : none
//----------------------------------------------------------------------------
void CAN_CellVoltage3ToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_VOLT_GRP3);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_ArrayLtc6803Unit[0].CellVolt[8];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_ArrayLtc6803Unit[0].CellVolt[9];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_ArrayLtc6803Unit[1].CellVolt[0];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_ArrayLtc6803Unit[1].CellVolt[1];

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//----------------------------------------------------------------------------
// Function    : CAN_CellVoltage4ToTxBuf
// Description :  将单体电芯电压值 13~16 放入发送缓冲区
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_CellVoltage4ToTxBuf(void)
{
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_VOLT_GRP4);
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_ArrayLtc6803Unit[1].CellVolt[2];
   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_ArrayLtc6803Unit[1].CellVolt[3];
   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_ArrayLtc6803Unit[1].CellVolt[4];
   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_ArrayLtc6803Unit[1].CellVolt[5];
	
   if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
   {
      g_CanMsgBuf.TxBuf_Wptr = 0;
   }
}

//----------------------------------------------------------------------------
// Function    : CAN_CellVoltage5ToTxBuf
// Description :  将单体电芯电压值 17~20 放入发送缓冲区
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_CellVoltage5ToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_VOLT_GRP5);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_ArrayLtc6803Unit[1].CellVolt[6];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_ArrayLtc6803Unit[1].CellVolt[7];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_ArrayLtc6803Unit[1].CellVolt[8];
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_ArrayLtc6803Unit[1].CellVolt[9];

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//============================================================================
// Function    : CAN_CellTempToTxBuf
// Description : 将电芯温度值放入发送缓冲区
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_CellTempToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(BCA,CAN_MSG_BMS_BCA_TEMP_GRR1);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.CellTemp[0];
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = g_BatteryParameter.CellTemp[1];
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_BatteryParameter.CellTemp[2];
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = g_BatteryParameter.CellTemp[3];
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0x00;


	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}





//----------------------------------------------------------------------------
// Function    : CAN_BatteryStateToTxBuf
// Description : 电池包基本信息放入发送缓冲区
// Parameters  : 
// Returns     : 
//----------------------------------------------------------------------------
void CAN_BatteryStateToTxBuf(void)
{
	static uint8_t WatchDog = 0;	// 用于丢帧判断

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_BATTERY_STATE);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.voltage;
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint16_t)g_BatteryParameter.current;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_BatteryParameter.SOC;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = WatchDog++;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//----------------------------------------------------------------------------
// Function    : CAN_CellStateToTxBuf
// Description : 将电芯基本状态信息放入发送缓冲区
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_CellStateToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_CELL_STATE);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.CellVoltMax;
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_BatteryParameter.CellVoltMin;
	            g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)g_BatteryParameter.CellTempMax;
	            g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = (uint8_t)g_BatteryParameter.CellTempMin;
	            g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_BatteryParameter.MaxCellNum;
	            g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = g_BatteryParameter.MinCellNum;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


//----------------------------------------------------------------------------
// Function    : CAN_PackWarningToTxBuf
// Description : 将电池包警告信息放入发送缓冲区
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_PackWarningToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PACK_WARNING);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint32_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_SystemWarning.all;

	if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}



//============================================================================
// Function    : CAN_PackSohToTxBuf
// Description : 将pack的SOH相关参数放到发送缓冲区
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_PackSohToTxBuf(void)
{
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PACK_SOH);
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
   g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.sohCycleTimes;
   *(uint32_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_BatteryParameter.ChargedAh;
   *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0;

   if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
   {
      g_CanMsgBuf.TxBuf_Wptr = 0;
   }
}

//============================================================================
// Function    : CAN_SysVerToTxBuf
// Description : 将系统软件和硬件的版本号放入发送缓冲区
// Parameters  : none
// Returns     : none
//----------------------------------------------------------------------------
void CAN_SysVerToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_BMS_VERSION);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x05;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)_HARDWARE_MAJOR_VERSION;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)_HARDWARE_MINOR_VERSION;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)_FIRMWARE_MAJOR_VERSION;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)_FIRMWARE_MINOR_VERSION;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)_FIRMWARE_REVISE_VERSION;

	if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

void CAN_AckMcsToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = SEND_HEART_ID;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_STD;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = 0x05;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0x00;

	if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

void CAN_SendHeartToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = SEND_HEART_ID;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_STD;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = 0x7F;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0x00;

	if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

void CAN_SendSTDBattInfoToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = SendBatterInfoID;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_STD;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	uint8_t levelone = 0x00;
	uint8_t leveltwo = 0x00;

	if(g_SystemWarning.DOT == WARNING_FIRST_LEVEL)
	{
		levelone |= 0x01;
	}

	if(g_SystemWarning.DUT == WARNING_FIRST_LEVEL)
	{
		levelone |= 0x02;
	}

	if(g_SystemWarning.TDIF == WARNING_FIRST_LEVEL)
	{
		levelone |= 0x04;
	}

	if(g_SystemWarning.COV == WARNING_FIRST_LEVEL)
	{
		levelone |= 0x08;
	}

	if(g_SystemWarning.CUV == WARNING_FIRST_LEVEL)
	{
		levelone |= 0x10;
	}

	if(g_SystemWarning.DOT == WARNING_SECOND_LEVEL)
	{
		leveltwo |= 0x01;
	}

	if(g_SystemWarning.DUT == WARNING_SECOND_LEVEL)
	{
		leveltwo |= 0x02;
	}

	if(g_SystemWarning.TDIF == WARNING_SECOND_LEVEL)
	{
		leveltwo |= 0x04;
	}

	if(g_SystemWarning.COV == WARNING_SECOND_LEVEL)
	{
		leveltwo |= 0x08;
	}

	if(g_SystemWarning.CUV == WARNING_SECOND_LEVEL)
	{
		leveltwo |= 0x10;
	}
	// 以下集团给的协议有误，
	if(g_BatteryParameter.voltage == 0)// 电池不存在
	{
		leveltwo |= 0x40;
	}
	if(g_SystemWarning.DOC == WARNING_SECOND_LEVEL)
	{
		leveltwo |= 0x20;
	}

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = levelone;
			
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = leveltwo;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_BatteryParameter.SOC;
	// 充电为正，放电为负
	uint16_t current = 0 - g_BatteryParameter.current;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)current;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)(current>>8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = g_BatteryParameter.CellTempMax;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_BatteryParameter.CellTempMin;

	if(g_BatteryMode == CHARGE)
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xaa;
	else if(g_BatteryMode == DISCHARGE)
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;
	
	if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


//============================================================================
// Function    : CAN_SendMsg_PackPra
// Description : 发送Pack信息 电池包的充放电状态
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadPackPra(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PACK_PRA);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)CELL_SERIES_NUM;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)CELL_PARALLEL_NUM;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)BATTERY_CAPACITY_RATED;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(BATTERY_CAPACITY_RATED >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_BatteryMode;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//============================================================================
// Function    : CAN_GUI_ReadSetOUV
// Description : 发送高低压故障信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadSetOUV(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_OUC);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_CellOVThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_CellOVThr.cls_1 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_CellOVThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_CellOVThr.cls_2 >> 8U);
	
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)g_CellUVThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = (uint8_t)(g_CellUVThr.cls_1 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = (uint8_t)g_CellUVThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = (uint8_t)(g_CellUVThr.cls_2 >> 8U);
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}

//============================================================================
// Function    : CAN_GUI_ReadSetCOT
// Description : 发送充电高温保护信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadSetCOT(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_COT);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_PACKCOTThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_PACKCOTThr.cls_1 >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_PACKCOTThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_PACKCOTThr.cls_2 >> 8);

	
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//============================================================================
// Function    : CAN_GUI_ReadSetCUT
// Description : 发送充电低温保护信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadSetCUT(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_CUT);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_PACKCUTThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_PACKCUTThr.cls_1 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_PACKCUTThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_PACKCUTThr.cls_2 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//============================================================================
// Function    : CAN_GUI_ReadSetDOT
// Description : 发送放电高温保护信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadSetDOT(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_DOT);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_PACKDOTThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_PACKDOTThr.cls_1 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_PACKDOTThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_PACKDOTThr.cls_2 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


//============================================================================
// Function    : CAN_GUI_ReadSetDUT
// Description : 发送放电低温保护信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadSetDUT(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_DUT);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (int8_t)(g_PACKDUTThr.cls_1);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (int8_t)(g_PACKDUTThr.cls_1 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (int8_t)(g_PACKDUTThr.cls_2);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (int8_t)(g_PACKDUTThr.cls_2 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}



//============================================================================
// Function    : CAN_GUI_ReadSetCOC
// Description : 发送充电过流保护信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadSetCOC(void)
{
	uint16_t temp;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_COC);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_BattCOCThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_BattCOCThr.cls_1 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_BattCOCThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_BattCOCThr.cls_2 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;
	
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

//============================================================================
// Function    : CAN_GUI_ReadSetDOC
// Description : 发送放电过流保护信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadSetDOC(void)
{
	uint16_t temp;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_DOC);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_BattDOCThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_BattDOCThr.cls_1 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_BattDOCThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_BattDOCThr.cls_2 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = PACK_DOC_WARNING_DLY;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = PACK_DOC_FAULT_DLY;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}


// 电池包的温度差
void CAN_GUI_READSetPDLT(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_PDLT);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_PACKDLTThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_PACKDLTThr.cls_1 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_PACKDLTThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_PACKDLTThr.cls_2 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;
	
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}

// 电池包的过压
void CAN_GUI_READSetPOV(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_POV);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_PackOVThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_PackOVThr.cls_1 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_PackOVThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_PackOVThr.cls_2 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;
	
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}

// 电池包的欠压
void CAN_GUI_READSetPUV(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_PUV);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_PackUVThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_PackUVThr.cls_1 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_PackUVThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_PackUVThr.cls_2 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;
	
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}

// 电池包的单体一致性
void CAN_GUI_ReadSetIBM(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PRA_SET_IBM);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_CellIBMThr.cls_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_CellIBMThr.cls_1 >> 8U);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_CellIBMThr.cls_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_CellIBMThr.cls_2 >> 8U);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;
	
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}



// 将 GUI 要读取的soh的循环次数，最近的三次错误信息放入发送缓冲区
//
void CAN_GUI_ReadNormalRec(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_RUN_NORMAL_REC);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_BatteryParameter.sohCycleTimes;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_BatteryParameter.sohCycleTimes >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_FaultRecentRec.code_0;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = g_FaultRecentRec.code_1;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_FaultRecentRec.code_2;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if(++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}	
}


// 将 GUI 要读取的充放电过流错误信息放入发送缓冲区
void CAN_GUI_ReadFaultCnt1(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_FAULT_REC_1);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_FaultRecord.coc;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_FaultRecord.coc >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = 0xFF;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = 0xff;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)g_FaultRecord.doc;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = (uint8_t)(g_FaultRecord.doc >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


// 将 GUI 要读取的充电电压错误信息放入发送缓冲区
void CAN_GUI_ReadFaultCnt2(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_FAULT_REC_2);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_FaultRecord.cov;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_FaultRecord.cov >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_FaultRecord.pov;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_FaultRecord.pov >> 8);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)g_FaultRecord.cuv;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = (uint8_t)(g_FaultRecord.cuv >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = (uint8_t)g_FaultRecord.puv;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = (uint8_t)(g_FaultRecord.puv >> 8);

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


// 将 GUI 要读取的充电温度错误信息放入发送缓冲区
void CAN_GUI_ReadFaultCnt3(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_FAULT_REC_3);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_FaultRecord.cot;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_FaultRecord.cot >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = 0xFF;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = 0xff;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)g_FaultRecord.cut;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = (uint8_t)(g_FaultRecord.cut >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


// 将 GUI 要读取的放电温度错误信息放入发送缓冲区
void CAN_GUI_ReadFaultCnt4(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_FAULT_REC_4);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_FaultRecord.dot;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_FaultRecord.dot >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)g_FaultRecord.dut;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_FaultRecord.dut >> 8);

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = (uint8_t)g_FaultRecord.vdif;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = (uint8_t)(g_FaultRecord.vdif >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = (uint8_t)g_FaultRecord.tdif;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = (uint8_t)(g_FaultRecord.tdif >> 8);

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


// 将 GUI 要读取的ltc6803错误信息放入发送缓冲区
void CAN_GUI_ReadFaultCnt5(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_FAULT_REC_5);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)g_FaultRecord.ltc_com;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_FaultRecord.ltc_com >> 8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = 0x00;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0x00;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0x00;

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


//**************************************************************************//
//
//**************************************************************************//

//============================================================================
// Function    : CAN_GUI_WriteBufToE2PRom
// Description : 将缓冲区中的数据写入eeprom中  目前暂定生产上的一些信息
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_WriteBufToE2PRom(uint8_t *ptr)
{
	uint8_t offset, len;

	if (*ptr > 127)
	{
		return;
	}
	else
	{
		offset = *ptr;
	}

	if ((*(ptr+1) == 0)
		|| (*(ptr+1) > 6))
	{
		return;
	}
	else
	{
		len = *(ptr+1);
	}

	EEPROM_WriteBlock((EEPROM_ADDR_FOR_GUI+offset), (ptr+2), len);
}

//============================================================================
// Function    : CAN_GUI_ReadBuf
// Description : 从eeprom中读取数据放入发送缓冲区
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ReadBuf(uint8_t *ptr)
{
	uint8_t offset, len;

	if (*ptr > 127)
	{
		return;
	}
	else
	{
		offset = *ptr;
	}

	if ((*(ptr+1) == 0)
	|| (*(ptr+1) > 6))
	{
		return;
	}
	else
	{
		len = *(ptr+1);
	}

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(GUI, CAN_MSG_PACK_SOH);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = offset;
	*(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = len;
	EEPROM_ReadBlock((EEPROM_ADDR_FOR_GUI+offset), &g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2], len);

	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}


//============================================================================
// Function    : CAN_GUI_ConfigCovThr
// Description : 配置电芯充电过压等级界限值参数
// Parameters  : none
// Returns     : none
//============================================================================
void CAN_GUI_ConfigCovThr(uint8_t *ptrData)
{
	g_CellOVThr.cls_1 = *(uint16_t *)ptrData;
	g_CellOVThr.cls_2 = *(uint16_t *)(ptrData+2);    
	g_CellOVThr.crc   = calculate_crc8(ptrData, 4);
	EEPROM_WriteBlock(EEPROM_ADDR_COV_THRHOLD, (uint8_t *)&g_CellOVThr.cls_1, 5);
}

// 配置电芯充电低压等级界限值参数
void CAN_GUI_ConfigCuvThr(uint8_t *ptrData)
{
    g_CellUVThr.cls_1 = *(uint16_t *)ptrData;
    g_CellUVThr.cls_2 = *(uint16_t *)(ptrData+2);
    g_CellUVThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_CUV_THRHOLD, (uint8_t *)&g_CellUVThr.cls_1, 5);
}

// 配置电芯充电高温等级界限值参数
void CAN_GUI_ConfigCotThr(uint8_t *ptrData)
{
	g_PACKCOTThr.cls_1 = *(uint16_t *)ptrData;
	g_PACKCOTThr.cls_2 = *(uint16_t *)(ptrData+2);

	g_PACKCOTThr.crc = calculate_crc8(ptrData, 4);
	EEPROM_WriteBlock(EEPROM_ADDR_COT_THRHOLD, (uint8_t *)&g_PACKCOTThr.cls_1, 5);
}

// 配置电芯充电低温等级界限值参数
void CAN_GUI_ConfigCutThr(uint8_t *ptrData)
{
	g_PACKCUTThr.cls_1 = *(uint16_t *)ptrData;
	g_PACKCUTThr.cls_2 = *(uint16_t *)(ptrData+2);
	g_PACKCUTThr.crc = calculate_crc8(ptrData, 4);
	EEPROM_WriteBlock(EEPROM_ADDR_CUT_THRHOLD, (uint8_t *)&g_PACKCUTThr.cls_1, 5);
}

// 配置电芯放电高温等级界限值参数
void CAN_GUI_ConfigDotThr(uint8_t *ptrData)
{
    g_PACKDOTThr.cls_1 = *(uint16_t *)ptrData;
    g_PACKDOTThr.cls_2 = *(uint16_t *)(ptrData+2);
    g_PACKDOTThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_DOT_THRHOLD, (uint8_t *)&g_PACKDOTThr.cls_1, 5);
}

// 配置放电低温等级界限值参数
void CAN_GUI_ConfigDutThr(uint8_t *ptrData)
{
	g_PACKDUTThr.cls_1 = *(uint16_t *)ptrData;
	g_PACKDUTThr.cls_2 = *(uint16_t *)(ptrData+2);
	g_PACKDUTThr.crc = calculate_crc8(ptrData, 4);
	EEPROM_WriteBlock(EEPROM_ADDR_DUT_THRHOLD, (uint8_t *)&g_PACKDUTThr.cls_1, 5);
}


// 配置电池包充电过流等级值参数
void CAN_GUI_ConfigCocThr(uint8_t *ptrData)
{
    g_BattCOCThr.cls_1 = *(uint16_t *)ptrData;
    g_BattCOCThr.cls_2 = *(uint16_t *)(ptrData+2);
    g_BattCOCThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_COC_THRHOLD, (uint8_t *)&g_BattCOCThr.cls_1, 5);
}

// 配置电池包放电过流等级值参数
void CAN_GUI_ConfigDocThr(uint8_t *ptrData)
{
    g_BattDOCThr.cls_1 = *(uint16_t *)ptrData;
    g_BattDOCThr.cls_2 = *(uint16_t *)(ptrData+2);
    g_BattDOCThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_DOC_THRHOLD, (uint8_t *)&g_BattDOCThr.cls_1, 5);
}

// 配置电池温差告警等级值
void CAN_GUI_ConfigDltThr(uint8_t *ptrData)
{
    g_PACKDLTThr.cls_1 = *(uint16_t *)ptrData;
    g_PACKDLTThr.cls_2 = *(uint16_t *)(ptrData+2);
    g_PACKDLTThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_DLT_THRHOLD, (uint8_t *)&g_PACKDLTThr.cls_1, 5);
}


// 配置单体一致性等级值参数
void CAN_GUI_ConfigDlvThr(uint8_t *ptrData)
{
    g_CellIBMThr.cls_1 = *(uint16_t *)ptrData;
    g_CellIBMThr.cls_2 = *(uint16_t *)(ptrData+2);
    g_CellIBMThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_DLV_THRHOLD, (uint8_t *)&g_CellIBMThr.cls_1, 5);
}


// 配置电池包过压等级参数  
void CAN_GUI_ConfigPovThr(uint8_t *ptrData)
{
    g_PackOVThr.cls_1 = *(uint16_t *)ptrData;
    g_PackOVThr.cls_2 = *(uint16_t *)(ptrData+2);
    
    g_PackOVThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_POV_THRHOLD, (uint8_t *)&g_PackOVThr.cls_1, 5);
}


// 配置电池包低压等级参数  
void CAN_GUI_ConfigPuvThr(uint8_t *ptrData)
{
    g_PackUVThr.cls_1 = *(uint16_t *)ptrData;
    g_PackUVThr.cls_2 = *(uint16_t *)(ptrData+2);
    
    g_PackUVThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_PUV_THRHOLD, (uint8_t *)&g_PackUVThr.cls_1, 5);
}


// 配置绝缘性参数
void CAN_GUI_ConfigIsoThr(uint8_t *ptrData)
{
    g_IsoThr.cls_1 = *(uint16_t *)ptrData;
    g_IsoThr.cls_2 = *(uint16_t *)(ptrData+2);
    
    g_IsoThr.crc = calculate_crc8(ptrData, 4);
    EEPROM_WriteBlock(EEPROM_ADDR_ISO_THRHOLD, (uint8_t *)&g_IsoThr.cls_1, 5);
}


//============================================================================
// Function    : CAN_ClearImage
// Description : 上位机GUI向单片机更新用户app时发送的命令，擦除flash中apphead块
//               ，然后reset，运行bootloader,接收GUI发送的新的用户app image。
// Parameters  : 
// Returns     : none
//----------------------------------------------------------------------------
void CAN_ClearImage(uint8_t *can_msg)
{
    LedRedOff();
    
	if (can_msg[0] == 0x01 
		&& can_msg[1] == 0x23               
		&& can_msg[2] == 0x45 
		&& can_msg[3] == 0x67 
		&& can_msg[4] == 0x89 
		&& can_msg[5] == 0xAB 
		&& can_msg[6] == 0xCD 
		&& can_msg[7] == 0xEF)  // 8个字节的密码，主要防治误操作
	{
		INTCON = 0x00;                            // Disable all interrupts
		FLASH_Erase64Bytes( _IMG_APPHDR_ADDR );     // Invalidate AppImage
		LedGreOn();
		Reset();                                 
	}
}


//  解析充电器的消息
void CAN_PaserChargerMsg(void)
{
	g_SystemError.ccs_comm = 0;	
	canCcsTimeout = 0;    
}

// 将要发送给充电机的信息放入发送缓冲区
void CAN_CharggerMsgToTxBuf(void)
{
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = ControlCCSID;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_STD;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = (uint8_t)(g_CCS_MaxVoltage>>8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = (uint8_t)(g_CCS_MaxVoltage);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = (uint8_t)(g_CCS_MaxCurrent>>8);
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = (uint8_t)(g_CCS_MaxCurrent);
	
	if(g_BatteryMode == CHARGE)
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0x00;
	else 
		g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = 0x01;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 0x0;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0x0;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0x0;
		
	if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
	{
		g_CanMsgBuf.TxBuf_Wptr = 0;
	}
}

// 将与电机控制器系统的握手信息放入发送缓冲区
void CAN_SendMcsHandshakeFrame(void)
{
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(MCS,CAN_MSG_BMS_MCS_HNDSK);
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = 'G';
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[1] = 'l';
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = 'o';
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[3] = 'b';
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = 'e';
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = 0xff;
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;

   if (++g_CanMsgBuf.TxBuf_Wptr >= CAN_BUF_DEEP)
   {
      g_CanMsgBuf.TxBuf_Wptr = 0;
   }	
}

// 将发给电机控制器系统的信息放入发送缓冲区
void CAN_SendMcsBattInfoFrame(void)
{
	uint8_t warn = 0;

   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].COB_ID = CAN_GenerateID(MCS,CAN_MSG_BMS_MCS_INFO);
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].IDE = CAN_ID_EXT;
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].RTR = CAN_RTR_DATA;
   	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].DLC = 0x08;

    *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[0] = g_BatteryParameter.CellVoltMax;
    *(uint16_t*)&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[2] = g_BatteryParameter.CellVoltMin;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[4] = g_BatteryParameter.CellTempMax;
    g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[5] = g_BatteryParameter.CellTempMin;

	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[6] = g_BatteryParameter.SOC;//Soc_GetSoc();
	g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Wptr].Data[7] = 0xff;
}

// 解析电气控制器的信息
void CAN_PaserMcsInfo(uint8_t *can_msg)
{
	if ( (can_msg[0]=='G') 
			&& (can_msg[1]=='l') 
			&& (can_msg[2]=='o') 
			&& (can_msg[3]=='b') 
			&& (can_msg[4]=='e') )
	{
		g_SystemError.mcs_comm = 0;	
		canMcsTimeout = 1;		
	}
}

// 解析温度模块发来的信息
void CAN_PaserTduGnrInfo(uint8_t *can_msg)
{
    
    g_BatteryParameter.CellTempMax = can_msg[0];
    g_BatteryParameter.CellTempMin = can_msg[1];
    g_BatteryParameter.CellTempAvg = can_msg[2];
    g_BatteryParameter.MaxTempModuIdx = can_msg[3]+1;
    g_BatteryParameter.MaxTempChnIdx = can_msg[4]+1;
    g_BatteryParameter.MinTempModuIdx = can_msg[5]+1;
    g_BatteryParameter.MinTempChnIdx = can_msg[6]+1;

    canTduTimeout = 0;
    g_SystemError.tdu_comm = 0;
}


// 更新要广播信息的发送缓冲区
void CAN_BroadcastBufUpdate(void)
{ 
    switch (brdTxTimer++)
    {
        case 5:
            CAN_PutBattWarnErrorToTxBuf();   
        break;
        case 10:
            CAN_PutBattInfoToBuf();
        break;
        case 15:
            CAN_PutCellVoltPosToTxBuf();       
        break;
        case 20:
            CAN_PutCellTempPosToTxBuf();          
        break;
        case 25:
            CAN_CellVoltage1ToTxBuf(); 
        break;
        case 30:
            CAN_CellVoltage2ToTxBuf();    
        break;
        case 35:
            CAN_CellVoltage3ToTxBuf();
        break;
        case 40:
            CAN_CellVoltage4ToTxBuf();
        break;
        case 45:   
            CAN_CellVoltage5ToTxBuf();
            break;  
        case 50:
            CAN_CellTempToTxBuf();  
            brdTxTimer = 0;          
        break;
       default: 
            if (brdTxTimer > 50)
            {
                brdTxTimer = 0;  
            }        
        break;
    }
}


// 充电器信息更新
void CAN_ChargerTskUpdate(void)
{
	static uint8_t chgr_can_timer = 0;

	if (chgr_can_timer++ > 100)
	{
		chgr_can_timer = 0;
		CAN_CharggerMsgToTxBuf();	
	}			
}

// 与电机控制器系统信息更新
void CAN_McsTskUpdate(void)
{
	static uint16_t mcs_can_timer = 0;

	if ( !(mcs_can_timer % 200) )
	{
		CAN_SendMcsHandshakeFrame();
	}

	mcs_can_timer++;	

	if (g_SystemError.mcs_comm) 
		return;

	if ( !(mcs_can_timer % 100) )
	{
		CAN_SendMcsBattInfoFrame();
	}

	if (mcs_can_timer > 200)
	{
		mcs_can_timer = 0;
	}
}


// 与电机控制器通信超时检测
void CAN_McsTimeoutCheck(void)
{
	if (canMcsTimeout++ > CAN_MCS_COMM_TIMEOUT)
	{
		g_SystemError.mcs_comm = 1;	
		canMcsTimeout = 0;
		if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_1S)
		{
			g_ProtectDelayCnt = RELAY_ACTION_DELAY_1S;
		}        
	}
}


// 与温度模块通信超时检测
void CAN_TmpBoardTimeoutCheck(void)
{
	if (canTduTimeout++ > CAN_TDU_COMM_TIMEOUT)
	{
		g_SystemError.tdu_comm = 1;	
		canTduTimeout = 0;
	}
}

// 与充电器通信超时检测
void CAN_ChargerTimeoutCheck(void)
{
	if (canCcsTimeout++ > CAN_CCS_COMM_TIMEOUT)
	{
		g_SystemError.ccs_comm = 0b1;	
		canCcsTimeout = 0;
        if (g_ProtectDelayCnt > RELAY_ACTION_DELAY_1S)
        {
            g_ProtectDelayCnt = RELAY_ACTION_DELAY_1S;
        }  
	}    
}


//============================================================================
// Function    ：ToggleTestPin
// Description ：切换测试管脚高低电平输出，产生脉冲波形
// Parameters  ：none 
// Returns     ：none
//============================================================================  
void ToggleCanSendTest(void)
{
	LATBbits.LATB6 ^= 1;
}


//----------------------------------------------------------------------------
// Function    ：CAN_SendTxBufMsg
// Description ：将发送缓冲区的数据发送出去
//               
// Parameters  ：none
// Returns     ：none
//----------------------------------------------------------------------------
void TskCanSendTxBufMsg(void)
{
	/* 缓冲区中是否有待发送的帧 */
	while(!CAN_IsTxBufEmpty())
	{
		ECAN_TransmitMsg(&g_CanMsgBuf.TxBuf[g_CanMsgBuf.TxBuf_Rptr]);
        if (++g_CanMsgBuf.TxBuf_Rptr >= CAN_BUF_DEEP)
        {
            g_CanMsgBuf.TxBuf_Rptr = 0;
        }
	}
}
//----------------------------------------------------------------------------
// Function    ：CAN_RecMsgToBuf
// Description ：将接收缓冲区RX0中的CAN帧放入接收缓冲区中，并做初步的解析
// Parameters  ：none
// Returns     ：none
//----------------------------------------------------------------------------
void TskCanRecMsgToBuf(void)
{
	if (CAN_IsRxBufFull())
	{
		RXB0CONbits.RXFUL = 0; // 如果接受缓冲区满，仍然接受数据，但丢弃
		RXB1CONbits.RXFUL = 0;
		return;
	}

	if (ECAN_ReceiveMsg(&g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Wptr]))
	{
		if (++g_CanMsgBuf.RxBuf_Wptr >= CAN_BUF_DEEP)
		{
			g_CanMsgBuf.RxBuf_Wptr = 0;
		}
	}
}


//----------------------------------------------------------------------------
// Function    ：CAN_ProcessRxMsg
// Description ：处理接收到的CAN数据帧
// Parameters  ：none
// Returns     ：none
//----------------------------------------------------------------------------
void TskCanProcessRxMsg(void)
{
	uint8_t can_sa, can_fc;

	while (!CAN_IsRxBufEmpty())
	{
		if (CAN_IsTxBufFull())  // 判断发送缓冲区是否满，以确定是否有空间给响应帧发送，
		{                      // 若没有空间发送响应帧，则暂不处理接收到的数据帧。
			return;             
		}

		if( CAN_ID_EXT == g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].IDE)  // 消息帧为扩展帧（暂时只有与GUI通信采用扩展帧）
		{
			can_sa = CAN_GetSourceAddr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].COB_ID);
			can_fc = CAN_GetFuncCode(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].COB_ID);
  
			if(GUI == can_sa)
			{
				switch(can_fc)
				{
				case CAN_MSG_BATTERY_STATE:
					CAN_BatteryStateToTxBuf();
					break;

				case CAN_MSG_CELL_STATE:
					CAN_CellStateToTxBuf();
					break;
					
				case CAN_MSG_PACK_WARNING	:
					CAN_PackWarningToTxBuf();
					break;

				case CAN_MSG_BATTERY_TEMPERATURE:
					CAN_CellTempToTxBuf();
					break;

				case CAN_MSG_PACK_SOH:
					CAN_PackSohToTxBuf();
					break;
				case CAN_MSG_BMS_VERSION:
					CAN_SysVerToTxBuf();
					break;

				// 以下为GUI读参数命令
				case CAN_MSG_PACK_PRA :
					CAN_GUI_ReadPackPra();
					break;

				case CAN_MSG_PRA_SET_COC :
					CAN_GUI_ReadSetCOC();
					break;
				case CAN_MSG_PRA_SET_DOC :
					CAN_GUI_ReadSetDOC();
					break;
				case CAN_MSG_PRA_SET_COT :
					CAN_GUI_ReadSetCOT();
					break;
				case CAN_MSG_PRA_SET_DOT :
					CAN_GUI_ReadSetDOT();	
					break;
				case CAN_MSG_PRA_SET_CUT :
					CAN_GUI_ReadSetCUT();
					break;
				case CAN_MSG_PRA_SET_DUT :
					CAN_GUI_ReadSetDUT();
					break;
				case CAN_MSG_PRA_SET_OUC :
					CAN_GUI_ReadSetOUV();
					break;
				case CAN_MSG_PRA_SET_IBM:
					CAN_GUI_ReadSetIBM();
					break;
				case CAN_MSG_PRA_SET_PDLT:
					CAN_GUI_READSetPDLT();
					break;
				case CAN_MSG_PRA_SET_POV:
					CAN_GUI_READSetPOV();
					break;
				case CAN_MSG_PRA_SET_PUV:
					CAN_GUI_READSetPUV();
					break;

				// 以下为GUI读写内部BUF命令
				case CAN_MSG_GUI_BUF_WRITE:
					CAN_GUI_WriteBufToE2PRom(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_MSG_GUI_BUF_READ:
					CAN_GUI_ReadBuf(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;

				// 以下为GUI读故障记录的命令
				case CAN_MSG_RUN_NORMAL_REC :
					CAN_GUI_ReadNormalRec();
					break;
				case CAN_MSG_FAULT_REC_1 :
					CAN_GUI_ReadFaultCnt1();
					break;
				case CAN_MSG_FAULT_REC_2 :
					CAN_GUI_ReadFaultCnt2();
					break;
				case CAN_MSG_FAULT_REC_3 :
					CAN_GUI_ReadFaultCnt3();
					break;
				case CAN_MSG_FAULT_REC_4 :
					CAN_GUI_ReadFaultCnt4();
					break;
				case CAN_MSG_FAULT_REC_5 :
					CAN_GUI_ReadFaultCnt5();
					break;
				case CAN_GUI_CONFIG_COV_TH:
					CAN_GUI_ConfigCovThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_CUV_TH:
					CAN_GUI_ConfigCuvThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_COT_TH:
					CAN_GUI_ConfigCotThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_CUT_TH:
					CAN_GUI_ConfigCutThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_DOT_TH:
					CAN_GUI_ConfigDotThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_DUT_TH:
					CAN_GUI_ConfigDutThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_COC_TH:
					CAN_GUI_ConfigCocThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data); 
					break;
				case CAN_GUI_CONFIG_DOC_TH:
					CAN_GUI_ConfigDocThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_DLV_TH:
					CAN_GUI_ConfigDlvThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case CAN_GUI_CONFIG_DLT_TH:
					CAN_GUI_ConfigDltThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case  CAN_GUI_CONFIG_POV_TH:
					CAN_GUI_ConfigPovThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;  
				case  CAN_GUI_CONFIG_PUV_TH:
					CAN_GUI_ConfigPuvThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				case  CAN_GUI_CONFIG_ISO_TH:
					CAN_GUI_ConfigIsoThr(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;          
				// GUI发送的FLASH擦除命令，bootloader启动命令
				case CAN_MSG_IMAGE_ERASE:
					CAN_ClearImage(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].Data);
					break;
				default:
					break;
				}

			}
		}
		else if(CAN_ID_STD == g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].IDE)
		{
			if(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].COB_ID == GET_HEART_ID) // 响应整车的心跳
			{
				CAN_AckMcsToTxBuf();
			}
			else if(g_CanMsgBuf.RxBuf[g_CanMsgBuf.RxBuf_Rptr].COB_ID == CCSBROADCASTID) // 
			{
				// 充电器广播数据的解析
				CAN_PaserChargerMsg();
			}
		}

		// read指针指向下一条待处理的消息
		if (++g_CanMsgBuf.RxBuf_Rptr >= CAN_BUF_DEEP)
		{
			g_CanMsgBuf.RxBuf_Rptr = 0;
		}

   }
}


