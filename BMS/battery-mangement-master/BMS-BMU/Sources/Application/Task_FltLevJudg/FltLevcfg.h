/*=======================================================================
 *Subsystem:   ���
 *File:        FltLevcfg.h
 *Author:      WenYuhao
 *Description: 
 ========================================================================
 * History:    
 * 1. Date:
      Author:
      Modification:
========================================================================*/

#ifndef _FIL_LEV_CFG_H_
#define _FIL_LEV_CFG_H_
  
  #include  "BattInfoConfig.h"

 /*======================================================================
              ���ϱ������ͷ���ֵ�궨��(2�������������ϵ������)
  ======================================================================*/
  //���ϱ�����ֵ�ȼ���������
  #define           FAULT_MODE                      1        //1:2������Ϊ��ߵȼ�;0:1������Ϊ��ߵȼ�
  
  //�����¶ȷֽ���
  #define           NTEMP_BOUNDARY                  (0+40)        //1:2������Ϊ��ߵȼ�;0:1������Ϊ��ߵȼ�
  
  //-------------------�ŵ���ϼ��ָ���ֵ(Level_DisCharge)----------------
 
  //�ŵ���ѹ��(�ֱ���:0.0001V)(ƫ����:0)
    //����״̬
  #define         F2_DISCHG_VOLTSL_NT               27000.0*(SYS_SERIES_YiDongLi)

  #define         F1_DISCHG_VOLTSL_NT               28500.0*(SYS_SERIES_YiDongLi)
  #define         R1_DISCHG_VOLTSL_NT               30500.0*(SYS_SERIES_YiDongLi)
    //����״̬
  #define         F2_DISCHG_VOLTSL_LT               25000.0*(SYS_SERIES_YiDongLi)

  #define         F1_DISCHG_VOLTSL_LT               27000.0*(SYS_SERIES_YiDongLi)
  #define         R1_DISCHG_VOLTSL_LT               28000.0*(SYS_SERIES_YiDongLi)
  
  #if (CELL_TYPE==0x03)//������﮵��
    //�ŵ絥���ѹ(�ֱ���:0.0001V)(ƫ����:0)
      //����״̬
    #define         F2_DISCHG_VOLTCL_NT             28000

    #define         F1_DISCHG_VOLTCL_NT             29000
    #define         R1_DISCHG_VOLTCL_NT             31000
      //����״̬
    #define         F2_DISCHG_VOLTCL_LT             25000

    #define         F1_DISCHG_VOLTCL_LT             27000
    #define         R1_DISCHG_VOLTCL_LT             28000
    
    //�ŵ絥��ѹ��(�ֱ���:0.0001V)(ƫ����:0)
    #define         F1_DISCHG_VOLTCD                3000  
    #define         R1_DISCHG_VOLTCD                2500  

  #endif    
  
  //�ŵ����(�ֱ���:1��)(ƫ����:-40)
  #define           F2_DISCHG_TEMPH                 (60 +40)

  #define           F1_DISCHG_TEMPH                 (55 +40)
  #define           R1_DISCHG_TEMPH                 (53 +40)
  
  //�ŵ����(�ֱ���:1��)(ƫ����:-40)
  #define           F2_DISCHG_TEMPL                 ((-30) +40) 

  #define           F1_DISCHG_TEMPL                 ((-15) +40)  
  #define           R1_DISCHG_TEMPL                 ((-12) +40)
  
  //�ŵ��²�
  #define           F1_DISCHG_TEMPD                 15
  #define           R1_DISCHG_TEMPD                 13
  
  //�ŵ����    
  #define           F2_DISCHG_CURRH                 650                   

  #define           F1_DISCHG_CURRH                 620                
  #define           R1_DISCHG_CURRH                 600 
  
  //��Ե���� 
  #define           F2_DISCHG_INSUL                 (100 * CELL_VOLT_MAX * SYS_SERIES_YiDongLi * 0.001)    //��λ:k��
  
  //------------------��������ֵ(FaultLevel_Charge)---------------------------
  //�����ѹ��(�ֱ���:0.0001V)(ƫ����:0)
  #define         F2_CHARGE_VOLTSH                  37000.0*(SYS_SERIES_YiDongLi)

  #define         F1_CHARGE_VOLTSH                  36500.0*(SYS_SERIES_YiDongLi)
  #define         R1_CHARGE_VOLTSH                  36000.0*(SYS_SERIES_YiDongLi)
  
  #if (CELL_TYPE==0x03)//������﮵��
    //��絥���ѹ(�ֱ���:0.0001V)(ƫ����:0)
    #define         F2_CHARGE_VOLTCH                37000

    #define         F1_CHARGE_VOLTCH                36500
    #define         R1_CHARGE_VOLTCH                36000
   
    //���絥��ѹ��(�ֱ���:0.0001V)(ƫ����:0)
    #define         F1_CHARGE_VOLTCD                3000  
    #define         R1_CHARGE_VOLTCD                2500  
  #endif    
  
  //������(�ֱ���:1��)(ƫ����:-40)
  #define           F2_CHARGE_TEMPH                 (55 +40)

  #define           F1_CHARGE_TEMPH                 (50 +40)
  #define           R1_CHARGE_TEMPH                 (48 +40)
  //������(�ֱ���:1��)(ƫ����:-40)
  #define           F2_CHARGE_TEMPL                 ((-5) +40) 

  #define           F1_CHARGE_TEMPL                 ((0) +40)  
  #define           R1_CHARGE_TEMPL                 ((5) +40)
  //����²�
  #define           F1_CHARGE_TEMPD                 15
  #define           R1_CHARGE_TEMPD                 13
  //������    
  #define           F2_CHARGE_CURRH                 275                   

  #define           F1_CHARGE_CURRH                 180                
  #define           R1_CHARGE_CURRH                 175 
  //��Ե����
  #define           F2_CHARGE_INSUL                 100                   
                   
  
  //-------------------����������ֵ(FaultLevel_Other)---------------------
  #define           OPENWIRE                        1
  
  
/*======================================================================
                      ��ŵ�����ʱ�估�ӳ�ʱ��궨��
========================================================================*/
  //����ʱ��
  #define PERIOD_DISCHARGE              500                 // �ŵ�����������100ms
  #define PERIOD_CHARGE                 500                 // �������������100ms
  //�ӳ�ʱ��
  #define DELAYTIME_UNLOCK              15                   //�����ӳ�
  #define DELAYTIME_DANGERLEVEL1        10      
  #define DELAYTIME_DANGERLEVEL2        5
  #define DELAYTIME_DANGERLEVEL3        3
  #define DELAYTIME_GETRIGHT            1
  #define DELAYTIME_OFFLINE             5         //�����ӳ�
  #define DELAYTIME_HVIL                5          //��ѹ�����ӳ�
  
  #define Addr_VoltLock                  0x0C02    //��ѹ������ǵ�ַ



#endif