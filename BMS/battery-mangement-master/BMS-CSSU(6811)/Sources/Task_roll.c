/*=======================================================================
 *Subsystem:   ���
 *File:        Task_Roll.C
 *Author:      ZWB
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    2017 - 11 - 9
      Author:  ZWB
      Modification: 
========================================================================*/
#include  "Includes.h"                

Task_Time Task_Flag_Counter;                             
                   
   
          
/*=======================================================================
 *������:      Task_Roll(void) 
 *����:        ����ʱ�亯��
 *����:        ��       
 *���أ�       ��
 *˵����       ����ʱ�䵽��־λ��1
========================================================================*/
void Task_Roll(void)                                
{    
     static uint8 cnt;
     
     
     switch(Task_Flag_Counter.Time_Flag600ms)
     {
         
            
         case 1:
               
               tasks[6].flags=1;   // �رվ��� 
               
             break;   
         
         
         case 3:
      
               tasks[0].flags=1;     // ��ѹ�ռ� 
             
             break;
             
         case 5:

               tasks[1].flags=1;   // ��ѹ���� 
             
             break;
             
             case 6:
               
               tasks[9].flags=1;   // �򿪾��� 
             
             break;
             
         case 7:
               
               
               tasks[2].flags=1;   // Pack�¶��ռ� 
             
             break;  
             
         case 9:
               
               tasks[3].flags=1;   // Pack�¶� 
              
             break; 
         
         case 11:
               
               tasks[4].flags=1;   // оƬ�¶��ռ�
             
             break;
         case 13:
               
               tasks[9].flags=1;   // �򿪾��� 
             
             break;
         case 15:
               
               tasks[5].flags=1;   // оƬ�¶ȴ��� 
             
             break;    
             
         case 17:
              if(cnt%2 == 0)
              {
               cnt = 0;
               tasks[7].flags=1;   // CAN 
              }
              cnt++;
              
             break; 
         case 19:
               
               tasks[8].flags=1;   // ��Ե��� 
             
             break;
             
         case 23:
               
               tasks[10].flags=1;   // bool 
             
             break;
         
                     
         case 50:
               
               Task_Flag_Counter.Time_Flag600ms = 0; 
             
             break;        
           
         default:
              break; 
      
     }

                           
     Task_Flag_Counter.Time_Roll++; 
}

/*========================================================================*/