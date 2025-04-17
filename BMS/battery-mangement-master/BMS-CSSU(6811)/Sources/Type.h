//--------------------------------------------------------------------------*
// �ļ���: Type.h (�������ͱ����ļ�)                                        *
// ˵  ��: ����������͵ı���,Ŀ��:                                         *
//         (1)���̱���������д����;                                         *
//         (2)���������ֲ,���Ը�����Ҫ�������.                            *                       
//--------------------------------------------------------------------------*

#ifndef TYPE_H                                   // �������룬��ֹ�ظ�����  
#define TYPE_H
 
typedef unsigned char         uint8;             //  8 λ�޷�����
typedef unsigned short int    uint16;            // 16 λ�޷�����
typedef unsigned long int     uint32;            // 32 λ�޷�����
typedef char                  int8;              //  8 λ�з�����
typedef short int             int16;             // 16 λ�з����� 
typedef int                   int32;             // 32 λ�з�����
	 
//���Ż���������,�ؼ���volatile
typedef volatile uint8        vuint8;            //  8 λ�޷����� 
typedef volatile uint16       vuint16;           // 16 λ�޷����� 
typedef volatile uint32       vuint32;      



// 32 λ�޷����� 
typedef volatile int8         vint8;             //  8 λ�з����� 
typedef volatile int16        vint16;            // 16 λ�з����� 
typedef volatile int32        vint32;            // 32 λ�з����� 
	
//CAN���Ľṹ��
typedef struct CanFrame
{
    uint32 m_ID;      // msg���ͷ�ID
    uint8 m_IDE;      // ��չ֡Ϊ1����׼֡Ϊ0
    uint8 m_RTR;      // Զ��֡Ϊ1������֡Ϊ0
    uint8 m_data[8];  // ֡����
    uint8 m_dataLen;  // ֡���ݳ���
    uint8 m_priority; // �������ȼ� 
}CANFRAME,* pCANFRAME;   

typedef struct Task_Time_Counter 
{
    uint16 Time_Flag600ms;
    uint16 Time_FlagGPS;
    uint16 Time_Handle;
    uint16 Time_Roll;
    //uint16 Time_IIC;
    uint16 Counter_Wire_Open;
    uint16 Counter_Volt_Collect;
    uint16 Counter_Volt_Process;
    uint16 Counter_Pack_Collect;
    uint16 Counter_Pack_Process;
    uint16 Counter_Chip_Collect;
    uint16 Counter_Chip_Process;
    uint16 BMS_to_UpMonitor;
    uint16 Counter_Control_Task;
    uint16 Counter_Balance_open;
    uint16 Counter_Balance_close; 
    uint16 Counter_Roll_Boot;
  
}Task_Time;
extern Task_Time Task_Flag_Counter;




#endif