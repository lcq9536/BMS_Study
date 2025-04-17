/*=======================================================================
 *Subsystem:   ���
 *File:        Init_Flash.C
 *Author:      Wenming
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#include  "Includes.h"
/*=======================================================================
 *������:      Init_Flash(void)
 *����:        
 *����:        ��       
 *���أ�       FlashErr.ErrCode
 
 *˵����       
========================================================================*/

uint8 Init_Flash(void)
{ 
  Init_Eeprom(&FlashErr);
  FlashErr = LaunchFlashCommand(2 ,PARTITION_D_FLASH, 0, DFPART, EEE_RAM, 0, 0, 0, 0, 0); 
  ErrorCheck(FlashErr,(accerr|fpviol|mgstat1|mgstat0), (erserif|pgmerif|epviolif|ersvif1|ersvif0|dfdif|sfdif));
  if(FlashErr.ErrCode == NoErr) 
  {
     FlashErr = LaunchFlashCommand(0 ,EEPROM_QUERY, 0, 0, 0, 0, 0, 0, 0, 0); 
     ErrorCheck(FlashErr,(accerr|fpviol|mgstat1|mgstat0), (erserif|pgmerif|epviolif|ersvif1|ersvif0|dfdif|sfdif));
     ChecPartErr(&FlashErr);  
  }  
  if(FlashErr.ErrCode == NoErr) //�޴���ʹ��EEPROM
  {
    FlashErr = LaunchFlashCommand(0 ,ENABLE_EEPROM_EMULATION, 0, 0, 0, 0, 0, 0, 0, 0);//BuffRamģ��EEprom(EEE)
    ErrorCheck(FlashErr, (accerr|fpviol|mgstat1|mgstat0), (erserif|pgmerif|epviolif|ersvif1|ersvif0|dfdif|sfdif));    
  }
  return(FlashErr.ErrCode);
}