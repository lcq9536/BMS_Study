/*=======================================================================
 *Subsystem:   ���
 *File:        LTC6811_CMDConfig.h
 *Author:      WenM
 *Description: �������ļ���Ҫ�����6811�ڲ��Ĵ������������
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:
      Author:
      Modification:
========================================================================*/
#ifndef _LTC6811_CMD_CONFIG_H_
#define _LTC6811_CMD_CONFIG_H_
  
  //���üĴ�����ĺ궨��
  #define  DGPIO               0xff        // GPIO���ſ���
  #define  DREFON                 1        // ��׼�ϵ�
  #define  DSWTRD                 0        // SWTEN����״̬
  #define  DADCOPT                0        // ADCģʽѡ��λ
  #define  UNDER_V             2.45        // ����Ƿѹ����ֵ
  #define  OVER_V              3.75        // ���ù�ѹ����ֵ
  #define  DDCC                   0        // �رշŵ翪��
  #define  DDCTW                  0        // �ŵ糬ʱֵ��ͣ�ã�
  #define  DDCTO                  2        // �ŵ糬ʱֵ��1���ӣ�
  #define  DCP_ENABLED            1        // �ŵ�õ�����
  #define  CELL_CH_ALL            0        // Channels to convert :all cell
  #define  CELL_CHST_ALL          0        // ״̬��ѡ��
  #define  chg                    0
  #define  pup_up                 1        // ��������
  #define  pup_down               0        // ��������
  #define  Temp_IC_over         110        // 6811����¶��趨
  //�ɼ�ģʽ
  #define MD_FAST                 1        // ADCOPT = 0  27kHz; ADCOPT = 1  14kHz
  #define MD_NORMAL               2        // ADCOPT = 0   7kHz; ADCOPT = 1   3kHz
  #define MD_FILTERED             3        // ADCOPT = 0   26Hz; ADCOPT = 1   2kHz

  //#define DCP_DISABLED 1

  //��ȡ���߷��������ֽ�����ʱ�����Ӧ
  //#define Time_4byte             50        // ΢���ģ������뿴LTC680оƬ��53ҳ��
  //#define Time_28byte           350       

#endif