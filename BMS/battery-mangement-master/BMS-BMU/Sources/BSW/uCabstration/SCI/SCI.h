/*=======================================================================
 *Subsystem:   ���
 *File:        SCI.h
 *Author:      WenMing
 *Description: ͨ�ţ�SCI2.
               �ӿڣ�PJ0��RXD����PJ1��TXD��
               �����ʣ�
 ========================================================================
 * History:        // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ���
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _SCI_H_
#define _SCI_H_
  
  #include  "TypeDefinition.h"
  #include  "MC9S12XEP100.h"
    
  typedef enum
  {
    Init_Normal_SCI = 0
  };
  
  typedef enum
  {
    SendMsg_SCI = 0,
    SendMsg_Fault_SCI_SCI1SR1_TDRE
  };
  
  uint8 SCI1_Init(void);
  uint8 SCI1_Send_Byte(uint8 data);
  uint8 SCI1_Send_NByte(uint8 n,uint8 string[]) ;  
     

#endif