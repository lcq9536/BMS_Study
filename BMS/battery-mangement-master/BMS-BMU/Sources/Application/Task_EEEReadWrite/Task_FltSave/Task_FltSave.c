/* ========================================================================
 *Author:      ZWB
 *Description: ͨ�ţ�
               �ӿڣ�
               �����ʣ�
 ========================================================================
 * History:    �޸���ʷ��¼�б�
 * 1. Date:    
      Author:  
      Modification: 
      
========================================================================*/

#include  "includes.h"

FltCodeSave_T  g_FltCodeSave;   
/*=========================================================================================================================
 *������:      static uint8 VoltSH( uint8 pack_volt_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       1 ��ѹ��ѹ����ֵ�洢
=========================================================================================================================*/
static 
uint8 VoltSH( uint8 pack_volt_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8  Epag;
    Epag  = EPAGE; 
    if( pack_volt_over != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0826);
        
        temp = temp & 0x03;
        
        if( temp != pack_volt_over || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = pack_volt_over; 
            write_val = 0x0300 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0820 = year;
            *(uint8*)0x0821 = month;
            *(uint8*)0x0822 = day;
            *(uint8*)0x0823 = hour;
            *(uint8*)0x0824 = minute;
            *(uint16*)0x0825 = write_val; 
            
            read_val = *(uint16*)(0x0825);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    
    EPAGE = Epag; 
    return error;    
}

/*==========================================================================================================================
 *������:      static uint8 VoltSL( uint8 pack_volt_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       2��ѹǷѹ����ֵ�洢
===========================================================================================================================*/
static 
uint8 VoltSL( uint8 pack_volt_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8  Epag;
    Epag  = EPAGE;   
    if( pack_volt_under != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x082d);
        
        temp = temp & 0x03;
        
        if( temp != pack_volt_under || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = pack_volt_under; 
            write_val = 0x0304 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0827 = year;
            *(uint8*)0x0828 = month;
            *(uint8*)0x0829 = day;
            *(uint8*)0x082a = hour;
            *(uint8*)0x082b = minute;
            *(uint16*)0x082c = write_val; 
            
            read_val = *(uint16*)(0x082c);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag; 
    return error;    
}

/*=========================================================================================================================
 *������:      static uint8 VoltCH( uint8 cell_volt_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute )   
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       3�����ѹ����ֵ�洢
=========================================================================================================================*/
static 
uint8 VoltCH( uint8 cell_volt_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8  Epag;
    Epag  = EPAGE;   
    if( cell_volt_over != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0834);
        
        temp = temp & 0x03;
        
        if( temp != cell_volt_over || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = cell_volt_over; 
            write_val = 0x0100 | write_val;
            first_fault = 1;
            
            *(uint8*)0x082e = year;
            *(uint8*)0x082f = month;
            *(uint8*)0x0830 = day;
            *(uint8*)0x0831 = hour;
            *(uint8*)0x0832 = minute;
            *(uint16*)0x0833 = write_val; 
            
            read_val = *(uint16*)(0x0833);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
             
        } 
    }
    EPAGE = Epag;
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 VoltCL( uint8 cell_volt_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       4����Ƿѹ����ֵ�洢
============================================================================================================================*/
static 
uint8 VoltCL( uint8 cell_volt_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag;
    Epag  = EPAGE;   
    if( cell_volt_under != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x083b);
        
        temp = temp & 0x03;
        
        if( temp != cell_volt_under || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = cell_volt_under; 
            write_val = 0x0104 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0835 = year;
            *(uint8*)0x0836 = month;
            *(uint8*)0x0837 = day;
            *(uint8*)0x0838 = hour;
            *(uint8*)0x0839 = minute;
            *(uint16*)0x083a = write_val; 
            
            read_val = *(uint16*)(0x083a);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag;
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 VoltCD( uint8 cell_volt_delta,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       5����ѹ�����ֵ����
============================================================================================================================*/
static 
uint8 VoltCD( uint8 cell_volt_delta,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag;
    Epag  = EPAGE;   
    if( cell_volt_delta != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0842);
        
        temp = temp & 0x03;
        
        if( temp != cell_volt_delta || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = cell_volt_delta; 
            write_val = 0x0108 | write_val;
            first_fault = 1;
            
            *(uint8*)0x083c = year;
            *(uint8*)0x083d = month;
            *(uint8*)0x083e = day;
            *(uint8*)0x083f = hour;
            *(uint8*)0x0840 = minute;
            *(uint16*)0x0841 = write_val; 
            
            read_val = *(uint16*)(0x0841);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag;
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 CurrH_Charge( uint8 charge_cur_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       6����������ֵ����
============================================================================================================================*/
static 
uint8 CurrH_Charge( uint8 charge_cur_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag;
    Epag  = EPAGE;  
    if( charge_cur_over != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0849);
        
        temp = temp & 0x03;
        
        if( temp != charge_cur_over || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = charge_cur_over; 
            write_val = 0x2000 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0843 = year;
            *(uint8*)0x0844 = month;
            *(uint8*)0x0845 = day;
            *(uint8*)0x0846 = hour;
            *(uint8*)0x0847 = minute;
            *(uint16*)0x0848 = write_val; 
            
            read_val = *(uint16*)(0x0848);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
             
        } 
    }
    EPAGE = Epag; 
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 CurrH_DisChg( uint8 dischar_cur_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       7�ŵ��������
============================================================================================================================*/
static 
uint8 CurrH_DisChg( uint8 dischar_cur_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag; 
    Epag  = EPAGE;  
    if( dischar_cur_over != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0850);
        
        temp = temp & 0x03;
        
        if( temp != dischar_cur_over || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = dischar_cur_over; 
            write_val = 0x2010 | write_val;
            first_fault = 1;
            
            *(uint8*)0x084a = year;
            *(uint8*)0x084b = month;
            *(uint8*)0x084c = day;
            *(uint8*)0x084d = hour;
            *(uint8*)0x084e = minute;
            *(uint16*)0x084f = write_val; 
            
            read_val = *(uint16*)(0x084f);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag;
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 TempH_DisChg( uint8 dischar_temper_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       8�ŵ���¹���
============================================================================================================================*/
static 
uint8 TempH_DisChg( uint8 dischar_temper_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag; 
    Epag  = EPAGE;  
    if( dischar_temper_over != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0857);
        
        temp = temp & 0x03;
        
        if( temp != dischar_temper_over || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = dischar_temper_over; 
            write_val = 0x1020 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0851 = year;
            *(uint8*)0x0852 = month;
            *(uint8*)0x0853 = day;
            *(uint8*)0x0854 = hour;
            *(uint8*)0x0855 = minute;
            *(uint16*)0x0856 = write_val; 
            
            read_val = *(uint16*)(0x0856);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag; 
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 TempH_Charge( uint8 charge_temper_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       9�����¹���
============================================================================================================================*/
static 
uint8 TempH_Charge( uint8 charge_temper_over,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag; 
    Epag  = EPAGE;  
    if( charge_temper_over != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x085e);
        
        temp = temp & 0x03;
        
        if( temp != charge_temper_over || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = charge_temper_over; 
            write_val = 0x1010 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0858 = year;
            *(uint8*)0x0859 = month;
            *(uint8*)0x085a = day;
            *(uint8*)0x085b = hour;
            *(uint8*)0x085c = minute;
            *(uint16*)0x085d = write_val; 
            
            read_val = *(uint16*)(0x085d);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag; 
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 TempL_DisChg( uint8 dischar_temper_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       10�ŵ���¹���
============================================================================================================================*/
static 
uint8 TempL_DisChg( uint8 dischar_temper_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag; 
    Epag  = EPAGE;   
    if( dischar_temper_under != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0865);
        
        temp = temp & 0x03;
        
        if( temp != dischar_temper_under || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = dischar_temper_under; 
            write_val = 0x1024 | write_val;
            first_fault = 1;
            
            *(uint8*)0x085f = year;
            *(uint8*)0x0860 = month;
            *(uint8*)0x0861 = day;
            *(uint8*)0x0862 = hour;
            *(uint8*)0x0863 = minute;
            *(uint16*)0x0864 = write_val; 
            
            read_val = *(uint16*)(0x0864);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        }  
    }
    EPAGE = Epag; 
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 TempL_Charge( uint8 charge_temper_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       11�����¹���
============================================================================================================================*/
static 
uint8 TempL_Charge( uint8 charge_temper_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag;
    Epag  = EPAGE;  
    if( charge_temper_under != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x086c);
        
        temp = temp & 0x03;
        
        if( temp != charge_temper_under || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = charge_temper_under; 
            write_val = 0x1014 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0866 = year;
            *(uint8*)0x0867 = month;
            *(uint8*)0x0868 = day;
            *(uint8*)0x0869 = hour;
            *(uint8*)0x086a = minute;
            *(uint16*)0x086b = write_val; 
            
            read_val = *(uint16*)(0x086b);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag;
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 TempD_DisChg( uint8 dischar_temper_delta,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       12�ŵ��²����
============================================================================================================================*/
static 
uint8 TempD_DisChg( uint8 dischar_temper_delta,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag; 
    Epag  = EPAGE;  
    if( dischar_temper_delta != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0873);
        
        temp = temp & 0x03;
        
        if( temp != dischar_temper_delta || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = dischar_temper_delta; 
            write_val = 0x1028 | write_val;
            first_fault = 1;
            
            *(uint8*)0x086d = year;
            *(uint8*)0x086e = month;
            *(uint8*)0x086f = day;
            *(uint8*)0x0870 = hour;
            *(uint8*)0x0871 = minute;
            *(uint16*)0x0872 = write_val; 
            
            read_val = *(uint16*)(0x0872);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag; 
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 TempD_Charge( uint8 charge_temper_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       13����²����
============================================================================================================================*/
static 
uint8 TempD_Charge( uint8 charge_temper_under,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8  Epag;  
    Epag  = EPAGE; 
    if( charge_temper_under != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0880);
        
        temp = temp & 0x03;
        
        if( temp != charge_temper_under || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = charge_temper_under; 
            write_val = 0x1018 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0874 = year;
            *(uint8*)0x0875 = month;
            *(uint8*)0x0876 = day;
            *(uint8*)0x0877 = hour;
            *(uint8*)0x0878 = minute;
            *(uint16*)0x0879 = write_val; 
            
            read_val = *(uint16*)(0x0879);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        }  
    }
    EPAGE = Epag; 
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 InsulationPos_Fault( uint8 insulation_pos_fault,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       14������Ե����
============================================================================================================================*/
static
uint8 InsulationPos_Fault( uint8 insulation_pos_fault,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8 Epag; 
    Epag  = EPAGE;  
    if( insulation_pos_fault != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0881);
        
        temp = temp & 0x03;
        
        if( temp != insulation_pos_fault || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = insulation_pos_fault; 
            write_val = 0x3000 | write_val;
            first_fault = 1;
            
            *(uint8*)0x087b = year;
            *(uint8*)0x087c = month;
            *(uint8*)0x087d = day;
            *(uint8*)0x087e = hour;
            *(uint8*)0x087f = minute;
            *(uint16*)0x0880 = write_val; 
            
            read_val = *(uint16*)(0x0880);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag; 
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 InsulationNeg_Fault( uint8 insulation_neg_fault,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       15������Ե����
============================================================================================================================*/
static 
uint8 InsulationNeg_Fault( uint8 insulation_neg_fault,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
    uint8 error = 0,temp;
    static uint8 first_fault;
    uint16 write_val,read_val;
    uint8  Epag;
    Epag  = EPAGE;   
    if( insulation_neg_fault != 0 )                 
    {
        EPAGE = 0xFC;
        
        temp = *(uint8*)(0x0888);
        
        temp = temp & 0x03;
        
        if( temp != insulation_neg_fault || first_fault == 0 ) 
        {
            error = 1;
             
            write_val = insulation_neg_fault; 
            write_val = 0x3004 | write_val;
            first_fault = 1;
            
            *(uint8*)0x0882 = year;
            *(uint8*)0x0883 = month;
            *(uint8*)0x0884 = day;
            *(uint8*)0x0885 = hour;
            *(uint8*)0x0886 = minute;
            *(uint16*)0x0887 = write_val; 
            
            read_val = *(uint16*)(0x0887);
            
            if( write_val == read_val ) 
            {
                error = 0; 
            }
        } 
    }
    EPAGE = Epag;
    return error;    
}

/*===========================================================================================================================
 *������:      static uint8 OpenWire_Fault( uint8 open_wire_fault,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       16���߿�·����
============================================================================================================================*/
static 
uint8 OpenWire_Fault( uint8 open_wire_fault,uint8 year,uint8 month,uint8 day,uint8 hour,uint8 minute ) 
{
  uint8 error = 0,temp;
  static uint8 first_fault;
  uint16 write_val,read_val;
  uint8 Epag; 
  Epag  = EPAGE;  
  if( open_wire_fault != 0 )                 
  {
    EPAGE = 0xFD;
    
    temp = *(uint8*)(0x0826);
    
    temp = temp & 0x03;
    
    if( temp != open_wire_fault || first_fault == 0 ) 
    {
      error = 1;
       
      write_val = 0xD100;
      first_fault = 1;
      
      *(uint8*)0x0820 = year;
      *(uint8*)0x0821 = month;
      *(uint8*)0x0822 = day;
      *(uint8*)0x0823 = hour;
      *(uint8*)0x0824 = minute;
      *(uint16*)0x0825= write_val; 
      
      read_val = *(uint16*)(0x0825);
      
      if( write_val == read_val ) 
      {
          error = 0; 
      }
    }
  }
  EPAGE = Epag;
  return error;    
}

/*=======================================================================
 *������:      FltSave_DelayTimeus(uint16 us)
 *����:        ���ķ�����ʱ����
 *����:        us:��ʱ������      
 *���أ�       
 *˵����       ���ķ�����ʱ����
========================================================================*/
static
void FltSave_DelayTimeus(uint16 us) 
{
	  uint16 delayval;
	  delayval = us * 9;
	  while(delayval--); 
}
/*=======================================================================
 *������:      static uint8 ReadFltCodeFromEeprom(uint8 read_falg,uint8 clear_falg)
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       ��ȡ���ϻ�������ϱ�־λ,read_flag=0xAA,���ȡ���й��ϣ�
               clear_flag = 0xAA,�����EEPROM�д洢�Ĺ���
========================================================================*/
static
uint8 ReadFltCodeFromEEE(uint8 *read_flag,uint8 *clear_flag)
{
    uint8 error = 0;
    uint8 j;
    uint8 Epag;
    uint8 *EEPROM_ptr=NULL;
    uint16 i;
    CANFRAME  FltCodeBMSToUpmonitor;
    
    Epag  = EPAGE;
    if(*read_flag == 0xAA) 
    {
       error = 1;
       *read_flag = 0;
       EPAGE = 0xFC;
       EEPROM_ptr = (uint8*)0x0820;
       for( i = 0 ; i < 15 ; i++) 
       {
           FltCodeBMSToUpmonitor.m_ID = Eeprom_Communication_ID; //0x1610C0F4      
        	 FltCodeBMSToUpmonitor.m_IDE = 1;
         	 FltCodeBMSToUpmonitor.m_RTR = 0;
         	 FltCodeBMSToUpmonitor.m_dataLen = 8;
        	 FltCodeBMSToUpmonitor.m_priority = 6;
        	 FltCodeBMSToUpmonitor.m_data[0] = 0xFC;              // ��������
           FltCodeBMSToUpmonitor.m_data[1] = *EEPROM_ptr++;
        	 FltCodeBMSToUpmonitor.m_data[2] = *EEPROM_ptr++;   
        	 FltCodeBMSToUpmonitor.m_data[3] = *EEPROM_ptr++;
        	 FltCodeBMSToUpmonitor.m_data[4] = *EEPROM_ptr++;
        	 FltCodeBMSToUpmonitor.m_data[5] = *EEPROM_ptr++;
        	 FltCodeBMSToUpmonitor.m_data[6] = *EEPROM_ptr++;
        	 FltCodeBMSToUpmonitor.m_data[7] = *EEPROM_ptr++;       
           while(CAN_ToUpMonitor(&FltCodeBMSToUpmonitor)); 
           FltSave_DelayTimeus(10);
       } 
       
       EPAGE = 0xFD;
       EEPROM_ptr = (uint8* )0x0820;
       FltCodeBMSToUpmonitor.m_ID = Eeprom_Communication_ID;//0x1610C0F4          
    	 FltCodeBMSToUpmonitor.m_IDE = 1;
     	 FltCodeBMSToUpmonitor.m_RTR = 0;
     	 FltCodeBMSToUpmonitor.m_dataLen = 8;
    	 FltCodeBMSToUpmonitor.m_priority = 6;
    	 FltCodeBMSToUpmonitor.m_data[0] = 0xFD;                  // ���ϴ���
       FltCodeBMSToUpmonitor.m_data[1] = *EEPROM_ptr++;
    	 FltCodeBMSToUpmonitor.m_data[2] = *EEPROM_ptr++;   
    	 FltCodeBMSToUpmonitor.m_data[3] = *EEPROM_ptr++;
    	 FltCodeBMSToUpmonitor.m_data[4] = *EEPROM_ptr++;
    	 FltCodeBMSToUpmonitor.m_data[5] = *EEPROM_ptr++;
    	 FltCodeBMSToUpmonitor.m_data[6] = *EEPROM_ptr++;
    	 FltCodeBMSToUpmonitor.m_data[7] = *EEPROM_ptr++;       
       while(CAN_ToUpMonitor(&FltCodeBMSToUpmonitor));
       FltSave_DelayTimeus(10); 
       error = 0;
    } 
    else if( *clear_flag == 0xAA ) 
    {
        DisableInterrupts;
        error = 1;
        *clear_flag = 0;
        
        for( j = 0 ; j < 2 ; j++) 
        {
           EPAGE = 0xFC + j ; 
           
           EEPROM_ptr = (uint8*)0x0800;
           
           for( i = 0 ; i < 1024 ; i++ ) 
           {
               
               *(EEPROM_ptr++) = 0x00 ;
           }     
        }
        error = 0;
        EnableInterrupts;
    }
    EEPROM_ptr = NULL;
    EPAGE = Epag;
    g_Roll_Tick.Roll_FltCodeP++;
    return error;
}

/*=======================================================================
 *������:      void Task_FltCodeProcess(void)
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       
========================================================================*/
void Task_FltCodeProcess(void)
{
  ReadFltCodeFromEEE(&g_FltCodeSave.ReadFlag, &g_FltCodeSave.ClearFlag);
}
/*=======================================================================
 *������:      void Task_FltCodeStore(void)
 *����:       
 *����:        ��       
 *���أ�       ��
 *˵����       �洢���ϵ��ڲ���ҳ��EEprom��0xFC,0xFD��0x08200~0x0BFF.
               0x0800~0x081FԤ��������EEPROM�ռ����ޣ�����ֻ�洢���
               һ�εĹ��ϡ�
========================================================================*/
void Task_FltCodeStore(void)
{
   
  uint8 readfault[16];
  uint8 year,day,month,hour,min;
  year  = HEX2BCD(Read_IIC_Time.IIC_Read_Year);
  month = HEX2BCD(Read_IIC_Time.IIC_Read_Month);
  day   = HEX2BCD(Read_IIC_Time.IIC_Read_Day);
  hour  = HEX2BCD(Read_IIC_Time.IIC_Read_Hour);
  min   = HEX2BCD(Read_IIC_Time.IIC_Read_Minute);

  readfault[0] = VoltSH(g_Flt_Charge.Level_Volt_Sys_High, year, month, day, hour, min); 
  
  readfault[1] = VoltSL(g_Flt_DisChg.Level_Volt_Sys_Low, year, month, day, hour, min); 
  
  readfault[2] = VoltCH(g_Flt_Charge.Level_Volt_Cell_High, year, month, day, hour, min); 
  
  readfault[3] = VoltCL(g_Flt_DisChg.Level_Volt_Cell_Low, year, month, day, hour, min); 
  
  readfault[4] = VoltCD((g_Flt_DisChg.Level_Volt_Cell_Diff_High|g_Flt_Charge.Level_Volt_Cell_Diff_High), year, month, day, hour, min); 
  
  readfault[5] = CurrH_Charge(g_Flt_Charge.Level_Current_Charge_High, year, month, day, hour, min); 
  
  readfault[6] = CurrH_DisChg(g_Flt_DisChg.Level_Current_DisCharge_High, year, month, day, hour, min); 
  
  readfault[7] = TempH_DisChg(g_Flt_DisChg.Level_Temp_High, year, month, day, hour, min); 
  
  readfault[8] = TempH_Charge(g_Flt_Charge.Level_Temp_High, year, month, day, hour, min); 
  
  readfault[9] = TempL_DisChg(g_Flt_DisChg.Level_Temp_Low, year, month, day, hour, min); 
  
  readfault[10] = TempL_Charge(g_Flt_Charge.Level_Temp_Low, year, month, day, hour, min); 
  
  readfault[11] = TempD_DisChg(g_Flt_DisChg.Level_Temp_Diff_High, year, month, day, hour, min); 
  
  readfault[12] = TempD_Charge(g_Flt_Charge.Level_Temp_Diff_High, year, month, day, hour, min); 
  
  readfault[13] = InsulationNeg_Fault(g_Flt_DisChg.Level_Insul|g_Flt_DisChg.Level_Insul, year, month, day, hour, min); 
  
  readfault[14] = InsulationPos_Fault(g_Flt_DisChg.Level_Insul|g_Flt_DisChg.Level_Insul, year, month, day, hour, min); 
  
  readfault[15] = OpenWire_Fault(g_OpenWireInfo.OpenWire_Status, year, month, day, hour, min); 
  
  g_Roll_Tick.Roll_FltCodeS++;
}


/*=======================================================================
========================================================================*/
