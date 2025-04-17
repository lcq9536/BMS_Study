/*=======================================================================
 *Subsystem:   ���
 *File:        CAN.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _CAN_H_
#define _CAN_H_
  
  #include  "TypeDefinition.h"
  
  //CAN��ʼ��
  enum Init_CAN
  {
    //��ʼ��λ������Ϊ0
    Init_Normal_CAN = 0,
    //��ʼ��λ��ʧ�ܵĶ�Ӧ
    Init_Fault_CAN_BaudRate,
    Init_Fault_CAN_Unready1,
    Init_Fault_CAN_Unready2,
    Init_Fault_CAN_Synchr
  };
  
  //CAN��������
  enum SendMsg_CAN
  {
    SendMsg_Normal = 0,
    SendMsg_Fault_Lenth,
    SendMsg_Fault_Synch,
    SendMsg_Fault_NoEmptyNode
  };
  
  //CAN��������
  enum GetMsg_CAN
  {
    GetMsg_Normal = 0,
    GetMsg_Fault_RFLG_RXF
  };
  
  //CAN���Ľṹ��
  typedef struct 
  {
    uint32 m_ID;      // msg���ͷ�ID
    uint8 m_IDE;      // ��չ֡Ϊ1����׼֡Ϊ0
    uint8 m_RTR;      // Զ��֡Ϊ1������֡Ϊ0
    uint8 m_data[8];  // ֡����
    uint8 m_dataLen;  // ֡���ݳ���
    uint8 m_priority; // �������ȼ� 
  }CANFRAME,*pCANFRAME; 


  //CAN0
  uint8 CAN0_Init(uint16 Baud_Rate);
  uint8 CAN0_SendMsg(pCANFRAME sendFrame);
  uint8 CAN0_GetMsg(pCANFRAME receiveFrame);

  //CAN1  ���
  uint8 CAN1_Init(uint16 Baud_Rate);
  uint8 CAN1_SendMsg(pCANFRAME sendFrame);
  uint8 CAN1_GetMsg(pCANFRAME receiveFrame);
  
  //CAN2  ���� 
  uint8 CAN2_Init(uint16 Baud_Rate);
  uint8 CAN2_SendMsg(pCANFRAME sendFrame);
  uint8 CAN2_GetMsg(pCANFRAME receiveFrame);
  
  void  CAN2_GetMsg_Process(pCANFRAME receiveFrame);
   
  //CAN3
  uint8 CAN3_Init(uint16 Baud_Rate);
  uint8 CAN3_SendMsg(pCANFRAME sendFrame);
  uint8 CAN3_GetMsg(pCANFRAME receiveFrame);

#endif  




