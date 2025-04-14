#include "Includes.h"

/*************************************************************/
/*                      ��ʼ�����໷                         */
/*************************************************************/
void PLL_Init(void) 
{
    uint8 i = 0;
    
    //Task_Die |=0x00000001;

    CLKSEL &= 0x7f;       //����OSCCLK��Ϊϵͳʱ��
    PLLCTL &= 0x8F;       //��ֹ���໷
    CRGINT &= 0xDF;
    //PLLCLK=2��OSCCLK��(SYNR+1)/(REFDV+1), fbus=PLLCLK/2
    #if(BUS_CLOCK == 120000000) 
        SYNR = 0xcd;
    #elif(BUS_CLOCK == 104000000) 
      SYNR = 0xcc;
    #elif(BUS_CLOCK == 96000000) 
      SYNR = 0xcb;
    #elif(BUS_CLOCK == 88000000) 
      SYNR = 0xca;
    #elif(BUS_CLOCK == 80000000) 
      SYNR = 0xc9;
    #elif(BUS_CLOCK == 72000000) 
      SYNR = 0xc8;
    #elif(BUS_CLOCK == 64000000) 
      SYNR = 0xc7;
    #elif(BUS_CLOCK == 56000000) 
      SYNR = 0xc6;
    #elif(BUS_CLOCK == 48000000) 
      SYNR = 0xc5;
    #elif(BUS_CLOCK == 40000000) 
      SYNR = 0x44;
    #elif(BUS_CLOCK == 32000000)
      SYNR = 0x43;     
    #elif(BUS_CLOCK == 24000000)
      SYNR = 0x42;
    #elif(BUS_CLOCK == 16000000)
      SYNR = 0x01;
   #endif 

    REFDV = 0x81;
    PLLCTL |=0x70;         //ʹ�����໷
    asm NOP;
    asm NOP;
    while(!(CRGFLG&0x08)); //PLLCLK����
    CLKSEL |= 0x80;        //����PLLCLKΪϵͳʱ��
    
    //Task_Die &=0xFFFFFFFE;
}