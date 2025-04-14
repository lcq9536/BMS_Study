/*=======================================================================
 *Subsystem:   ���
 *File:        BattInfoConfig.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _BATT_INFO_CONFIG_H_
#define _BATT_INFO_CONFIG_H_

 /*=======================================================================
                                ��������
========================================================================*/
  //����ģ��ʹ������                                   
  #define           ENABLE_CAHRGE_HEAT                              0       //������ʹ��(1:ʹ��)
  #define           ENABLE_RELAYADHESION_JUDGE                      1       //�̵���ճ���жϹ���ʹ��(1:ʹ��)
  #define           ENABLE_HVIL                                     1       //��ѹ��������ʹ��(1:ʹ��)
  #define           ENABLE_CHARGEGUN_TEMP                           0       //���ǹ�¶Ȳ���ʹ��(1:ʹ��)
  //ģʽѡ��
  #define           MODE_STOP_OR_SLEEP                              1       //1:��Ƭ������α����(�͹���״̬),0:����ֹͣ��ģʽ��Ƭ��������ģʽ
  #define           MODE_THERMISTOR                                 0x01    //0x00:10K���������裻0x01:100K����������
  #define           MODE_VCU_WORK                                   1       //1:���ʱVCU�ϵ�;0:���ʱVCU���ϵ�
  //��ʼ������                                  
  #define           RESET_EEPROM                                    0       //EEprom�ڴ�����(1:EEprom��ʼ��)
  #define           RESET_SOC                                       0       //SOC��ʼ��(1:��ʼ��)
  #define           RESET_CLOCK                                     0       //1:ʱ�ӳ�ʼ��,0:����ʼ��
  
  /*==============================================================
                              ��������
  ==============================================================*/
   //��о����
  #define CELL_TYPE                                                 0x03    /*0x01��Ǧ�᣻  0x02�����⣻        0x03��������ﮣ�0x04������ﮣ�0x05������ﮣ�
                                                                              0x06����Ԫﮣ�0x07���ۺ�������ӣ�0x08������ﮣ�  0xFF������*/
  //��ص����������
  #if(CELL_TYPE == 0x06) //��Ԫ﮵��
    #define         CELL_CAPACITY                                   46.0    //��������(Ah)
    #define         CELL_VOLT_NOMINAL                               3.7     //��Ƶ����ѹ(V)
    #define         CELL_VOLT_MAX                                   4.3     //��ߵ����ѹ(V)
    #define         CELL_VOLT_MIN                                   2.8     //��͵����ѹ(V)
  
  #elif(CELL_TYPE == 0x03)//������﮵��
    #define         CELL_CAPACITY                                   271.0    //��������(Ah)
    #define         CELL_VOLT_NOMINAL                               3.22    //��Ƶ����ѹ(V)
    #define         CELL_VOLT_MAX                                   3.65    //��ߵ����ѹ(V)
    #define         CELL_VOLT_MIN                                   2.75     //��͵����ѹ(V)     
  #endif
  
  //��ص���ʹ�ò���  
  #define           CELL_TEMP_MAX_DISCHARGE                         60      //���ŵ��¶�(��)
  #define           CELL_TEMP_MIN_DISCHARGE                         (-30)   //��С�ŵ��¶�(��)
  #define           CELL_TEMP_MAX_CHARGE                            60      //������¶�(��)
  #define           CELL_TEMP_MIN_CHARGE                            (-28)       //��С����¶�(��)
  #define           CELL_LIFE_CYCLE                                 2000    //ѭ���������Σ���������25��2�棬1C/1C��100%  DOD
  #define           CELL_RESIS_INTERNAL                             16      //�������(m��)
  
  //���屸�ò���
  //#define  CellEnergyRatio                145     //�����������Wh/KG��
  //#define  CellMass                       45      //�����о����(g)
  //#define  CellLength                     65.2    //�����о���ȳߴ�+-1(mm)
  //#define  Cellradius                     18.3    //�����о��ȳߴ�+-1(mm) 
  //#define  CellSelfDiscRate               0.03    //�Էŵ��� (25�棬28��)
  //#define  CellMaxStorageTemp             35      //�����ߴ洢�¶�(��)
  //#define  CellMinStorageTemp             15      //�����ʹ洢�¶�(��)
  
  //���ϵͳ���� 
  #define           SYS_PARALLEL                                    2.0                 //ϵͳ�ܲ���
  #define           SYS_SERIES                                      50                  //ϵͳ�ܴ���       
  #define           SYS_SERIES_YiDongLi                             25                  //�׶�����Ŀϵͳ����
  #define           SYS_NUMBER_BOX                                  1                   //ϵͳ������
  #define           SYS_NUMBER_MODULE                               (2*SYS_NUMBER_BOX)  //ϵͳ��ģ������   
  #define           SYS_NUMBER_MODULE_TEMP                          5                   //ϵͳÿ��ģ����¶�����

  #define           SYS_NUMBER_BOX_MODULE                           (SYS_NUMBER_MODULE/SYS_NUMBER_BOX) //ÿ�������ģ��
  #define           SYS_NUMBER_TEMP                                 (SYS_NUMBER_MODULE_TEMP*SYS_NUMBER_MODULE)//ϵͳ�¶��ܸ���
  
  #define           SYS_SERIES_BOX                                  (SYS_SERIES/SYS_NUMBER_BOX)    //ÿ������Ĵ�����
  #define           SYS_NUMBER_BOX_TEMP                             (SYS_NUMBER_BOX_MODULE*SYS_NUMBER_MODULE_TEMP)//ÿ��������¶�
  #define           SYS_PARALLEL_MODULE                             SYS_PARALLEL              //ÿ��ģ�鲢����
  #define           SYS_SERIES_MODULE                               (SYS_SERIES/SYS_NUMBER_MODULE) //ÿ��ģ�鴮����
  
  #define           SYS_VOLT_NOMINAL                                (CELL_VOLT_NOMINAL*SYS_SERIES_YiDongLi)//ϵͳ���ѹ
  #define           SYS_VOLT_MAX                                    (CELL_VOLT_MAX*SYS_SERIES_YiDongLi)  //ϵͳ��ߵ�ѹ
  #define           SYS_VOLT_MIN                                    (CELL_VOLT_MIN*SYS_SERIES_YiDongLi)  //ϵͳ��͵�ѹ
  
  #define           SYS_CAPACITY                                    (CELL_CAPACITY*SYS_PARALLEL)//ϵͳ����(Ah)
  #define           SYS_ELECTRIC_QUANTITY                           ((CELL_CAPACITY)*(SYS_VOLT_NOMINAL)/1000.0) //ϵͳ����(KWh)
  #define           SYS_LIFE_END_CAPACITY                           ((SYS_CAPACITY)*0.8) //�����ֹ����(%80SOH)
  
 
  


#endif