#include <stdio.h>
#include <rtthread.h>

#include "bms_monitor.h"

#include "bms_hal_monitor.h"

#include "bms_energy.h"
#include "bms_global.h"



#define DBG_TAG "monitor"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"




// thread config
#define MONITOR_TASK_STACK_SIZE	512
#define MONITOR_TASK_PRIORITY	19
#define MONITOR_TASK_TIMESLICE	25

#define MONITOR_TASK_PERIOD		250



// sample time config  MS
#define UPDATE_CELL_VOLTAGE_CYCLE	250
#define UPDAYE_BAT_VOLTAGE_CYCLE	250
#define UPDATE_CELL_TEMP_CYCLE		2000
#define UPDATE_BAT_CURRENT_CYCLE	1000






BMS_MonitorDataTypedef BMS_MonitorData;



static bool FlagSampleIntCur = false;

static bool FlagCellVoltage = true;
static bool FlagBatVoltage = true;
static bool FlagCellTemp = true;
static bool FlagBatCurrent = true;


static uint16_t CountCellVoltage = 0;
static uint16_t CountBatVoltage = 0;
static uint16_t CountCellTemp = 0;
//static uint16_t CountBatCurrent = 0;



static void BMS_MonitorTaskEntry(void *paramter);
static void BMS_MonitorBattery(void);
static void BMS_MonitorSysMode(void);


void BMS_MonitorInit(void)
{
	rt_thread_t thread;
	
	thread = rt_thread_create("monitor", 
							   BMS_MonitorTaskEntry, 
							   NULL,
							   MONITOR_TASK_STACK_SIZE,
							   MONITOR_TASK_PRIORITY,
							   MONITOR_TASK_TIMESLICE);

   if (thread == NULL)
   {
	   LOG_E("Create Task Fail");
   }

	rt_thread_startup(thread);
}

static void BMS_MonitorTaskEntry(void *paramter)
{
	while (1)
	{
		BMS_MonitorBattery();
		BMS_MonitorSysMode();
		rt_thread_mdelay(MONITOR_TASK_PERIOD);
	}
}


// 监控电池各项数据
static void BMS_MonitorBattery(void)
{
	// 单体电芯电压
	CountCellVoltage += MONITOR_TASK_PERIOD;
	if (FlagCellVoltage == true && CountCellVoltage >= UPDATE_CELL_VOLTAGE_CYCLE)
	{
		Bms_HalMonitorCellVoltage();
		CountCellVoltage = 0;
	}
	else if (FlagCellVoltage == false)
	{
		CountCellVoltage = 0;
	}
	
	// 电池组电�?
	CountBatVoltage += MONITOR_TASK_PERIOD;
	if (FlagBatVoltage == true && CountBatVoltage >= UPDAYE_BAT_VOLTAGE_CYCLE)
	{
		Bms_HalMonitorBatteryVoltage();
		CountBatVoltage = 0;		
	}
	else if (FlagBatVoltage == false)
	{
		CountBatVoltage = 0;
	}

	// 电池温度
	CountCellTemp += MONITOR_TASK_PERIOD;
	if (FlagCellTemp == true && CountCellTemp++ >= UPDATE_CELL_TEMP_CYCLE)
	{
		Bms_HalMonitorCellTemperature();
		CountCellTemp = 0;
	}
	else if (FlagCellTemp == false)
	{
		CountCellTemp = 0;
	}


	/* 电流采样由软件触�?
	CountBatCurrent += MONITOR_TASK_PERIOD;
	if (FlagBatCurrent == true && CountBatCurrent >= UPDATE_BAT_CURRENT_CYCLE)
	{
		CountBatCurrent = 0;
		Bms_HalMonitorBatteryCurrent();
	}
	else if (FlagBatCurrent == false)
	{
		CountBatCurrent = 0;
	}
	*/


	/* 电流采样由硬件中断触�?太麻烦了,每次烧写都得重新给BQ重新下上�?*/
	if (FlagSampleIntCur == true && FlagBatCurrent == true)
	{
		Bms_HalMonitorBatteryCurrent();
		FlagSampleIntCur = false;		
	}
}



// 系统模式监控
// BatteryCurrent > 20mA || BatteryCurrent < -20mA  处于非睡眠模�?// BatteryCurrent < 20mA || BatteryCurrent > -20mA  处于待机模式或者睡眠模�?// BatteryCurrent <= -20mA 处于放电模式
// BatteryCurrent >=  20mA 处于充电模式
static void BMS_MonitorSysMode(void)
{
	static BMS_SysModeTypedef SysModeBackup = BMS_MODE_NULL;
	static uint32_t StandbyCount = 0;
	
	if (BMS_GlobalParam.SysMode == BMS_MODE_SLEEP)
	{
		if ((BMS_MonitorData.BatteryCurrent >= 0.02) || (BMS_MonitorData.BatteryCurrent <= -0.02))
		{
			// 可以加唤醒处理逻辑
			
			BMS_GlobalParam.SysMode = BMS_MODE_STANDBY;
			LOG_I("Wake Up");
		}
		return;
	}

	
	if (BMS_MonitorData.BatteryCurrent < 0.02 && BMS_MonitorData.BatteryCurrent > -0.02)
	{
		BMS_GlobalParam.SysMode = BMS_MODE_STANDBY;	
		
		if (StandbyCount >= BMS_ENTRY_SLEEP_TIME * 60000)
		{
			if (BMS_EnergyData.BalanceReleaseFlag != true)
			{
				// 可以加睡眠低功耗处理逻辑
				
				StandbyCount = 0;
				BMS_GlobalParam.SysMode = BMS_MODE_SLEEP;;
				
				LOG_I("Entry Sleep Mode");
			}
		}
		else
		{
			StandbyCount += MONITOR_TASK_PERIOD;
		}


		// 调试�?
		if (SysModeBackup != BMS_MODE_STANDBY)
		{
			SysModeBackup = BMS_MODE_STANDBY;
			LOG_I("Entry Standby Mode");
		}
	}
	else if (BMS_MonitorData.BatteryCurrent >= 0.02)
	{
		StandbyCount = 0;
		BMS_GlobalParam.SysMode = BMS_MODE_CHARGE;

		// 调试�?
		if (SysModeBackup != BMS_MODE_CHARGE)
		{
			SysModeBackup = BMS_MODE_CHARGE;
			LOG_I("Entry Charge Mode");
		}
	}
	else if (BMS_MonitorData.BatteryCurrent <= -0.02)
	{
		StandbyCount = 0;
		BMS_GlobalParam.SysMode = BMS_MODE_DISCHARGE;

		// 调试�?	
		if (SysModeBackup != BMS_MODE_DISCHARGE)
		{
			SysModeBackup = BMS_MODE_DISCHARGE;
			LOG_I("Entry Discharge Mode");
		}
	}
}










void BMS_MonitorStateCellVoltage(BMS_StateTypedef NewState)
{
	if (NewState == BMS_STATE_ENABLE)
	{
		FlagCellVoltage = true;
	}
	else if (NewState == BMS_STATE_DISABLE)
	{
		FlagCellVoltage = false;
	}
}

void BMS_MonitorStateBatVoltage(BMS_StateTypedef NewState)
{
	if (NewState == BMS_STATE_ENABLE)
	{
		FlagBatVoltage = true;
	}
	else if (NewState == BMS_STATE_DISABLE)
	{
		FlagBatVoltage = false;
	}
}

void BMS_MonitorStateCellTemp(BMS_StateTypedef NewState)
{
	if (NewState == BMS_STATE_ENABLE)
	{
		FlagCellTemp = true;
	}
	else if (NewState == BMS_STATE_DISABLE)
	{
		FlagCellTemp = false;
	}	
}

void BMS_MonitorStateBatCurrent(BMS_StateTypedef NewState)
{
	if (NewState == BMS_STATE_ENABLE)
	{
		FlagBatCurrent = true;
	}
	else if (NewState == BMS_STATE_DISABLE)
	{
		FlagBatCurrent = false;
	}	
}





void BMS_MonitorHwCurrent(void)
{
	FlagSampleIntCur = true;
}

