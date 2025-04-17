/*=======================================================================
 *Subsystem:   ���
 *File:        System_Init.C
 *Author:      ZWB
 *Description: ͨ�ţ�
               �ӿڣ�
               ������:
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    
      Author:  
      Modification:            
========================================================================*/
#include  "Includes.h"                 /* ��������h�ļ������Ե������к�������� */ 



/*=======================================================================
 *������:      System_init(void) 
 *����:        ϵͳ��ʼ������
 *����:        ��       
 *���أ�       ��
 *˵����       ���еײ㺯���ĳ�ʼ�����Լ����ж�
========================================================================*/
void System_init(void) 
{
  
    DisableInterrupts;
    
    IVBR=0x7F;                         // �޸��жϵ�ַ��
    ADC_init();
    PLL_Init();
    Init_Flash();

    SPI1_Init();                       // ��ѹ�¶��ռ�spiͨѶ��ʼ��;
    
    INIT_CAN2();
    
    PITInit(0,0,249,1279);             // PIT0��ʱ10ms�ж�һ��; 
    EnablePIT(0);                      // ʹ��PIT0���ж�;
   // init_PIT0(); 
    
    //VoltageDetectInit();
    LTC6804_Init();                    // LTC6804��ʼ��;  

    Create_task();                     // ��ʼ��������������; 
    circuit_detection();
    
    Clear_Variable();
    
    DDRT=0XFF;
    PTT_PTT0 = 0;
    memset(&balance_receive,0,sizeof(balance_receive));

    EnableInterrupts;                  
    
}

/*=======================================================================
 *������:      delay_time(uint16 n)
 *����:        �ӳٺ���
 *����:        ��       
 *���أ�       ��
 *˵����       �����ӳٺ���
========================================================================*/
/*void delay_time(uint16 n)
{
    uint16 i;
    for(i = 0; i < n; i++);
}*/

/*=======================================================================
 *������:      Clear_Variable(void)
 *����:        ���㺯��
 *����:        ��       
 *���أ�       ��
 *˵����       ��һЩ�ṹ��ͱ����������㣻
========================================================================*/
void Clear_Variable(void)
{ 
    //memset(&Volt_Data_T,0,sizeof(Volt_Data_T));   
    //memset(&Task_Flag_Counter,0,sizeof(Task_Flag_Counter));   
    //memset(&Monitor_Stand,0,sizeof(Monitor_Stand));  
    //memset(&single_voltage,0,sizeof(single_voltage));
        
}
