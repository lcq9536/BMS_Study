/*******************************************************************************
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2005 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
********************************************************************************
Services performed by FREESCALE in this matter are performed AS IS and without 
any warranty. CUSTOMER retains the final decision relative to the total design 
and functionality of the end product. FREESCALE neither guarantees nor will be 
held liable by CUSTOMER for the success of this project.
FREESCALE DISCLAIMS ALL WARRANTIES, EXPRESSED, IMPLIED OR STATUTORY INCLUDING, 
BUT NOT LIMITED TO, IMPLIED WARRANTY OF MERCHANTABILITY OR FITNESS FOR 
A PARTICULAR PURPOSE ON ANY HARDWARE, SOFTWARE OR ADVISE SUPPLIED TO THE PROJECT
BY FREESCALE, AND OR NAY PRODUCT RESULTING FROM FREESCALE SERVICES. IN NO EVENT
SHALL FREESCALE BE LIABLE FOR INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF 
THIS AGREEMENT.

CUSTOMER agrees to hold FREESCALE harmless against any and all claims demands or 
actions by anyone on account of any damage, or injury, whether commercial, 
contractual, or tortuous, rising directly or indirectly as a result of an advise
or assistance supplied CUSTOMER in connection with product, services or goods 
supplied under this Agreement.
********************************************************************************
* File      Srec.c
* Owner     b01802
* Version   1.0   
* Date      Dec-02-2010
* Classification   General Business Information
* Brief     Srecord utilities
********************************************************************************
Revision History:
Version   Date          Author    Description of Changes
1.0       Dec-02-2010   b01802    Initial version
*******************************************************************************/
#include <ctype.h>
#include <string.h>
#include "Types.h"
#include "derivative.h" /* derivative-specific definitions */
#include "Errors.h"
#include "Srec.h"
#include "CAN.h"

UINT8 NumDataBytes;
    //holds the number of 'data' bytes in the received S-Record

extern unsigned int *boot;
/******************************************************************************/
static UINT8 GetHexByte(UINT8 *MemByte)    //��ʮ����ת��Ϊ��λ16���Ƶ���
{
  INT8 c;           //used to hold the received ASCII hex byte   ����16�����ֽڵ�ASCII��
  
  *MemByte = 0;
  
  c = getchar();    //get an ASCII hex byte from stdin (upper nybble of byte)
  if (!isxdigit(c))      //is it a valid hex digit�Ƿ�Ϊ��Ч����
  {
    can_send.data[0]=5;
    MSCAN2SendMsg(can_send);
    *boot=0x66;
    return(BadHexData);  //no. return an error
  }
  
  //convert the ASCII character to a binary hex nybble
  *MemByte = (isdigit(c) ? c - '0' : c - 'A' + 10) << 4;
  
  c = getchar();    //get an ASCII hex byte from stdin (lower nybble of byte)
  if (!isxdigit(c))      //is it a valid hex digit
  {
    can_send.data[0]=5;
    MSCAN2SendMsg(can_send);
    *boot=0x66;
    return(BadHexData);  //no. return an error
  }
  
  //convert the ASCII character to a binary hex nybble
  *MemByte += (isdigit(c) ? c - '0' : c - 'A' + 10);
  return(noErr);         //return 'no error'
}

/******************************************************************************/
UINT8 RcvSRecord(SRecDataRec *SRecData)      //����S��¼�ĵ�
{ 
  UINT8 Error;
    //used to hold the return error code
  UINT8 CheckSum;
    //used to hold calculated checksum value
  UINT8 SRecByte;
    //used to hold memory/address bytes received through GetHexByte()ʵ�ʽ��յ����ֽ���
  UINT32 LoadAddress = 0;
    // holds the converted load address of S-Record
  UINT8 x;
    //used as a loop counter when receiving 'data' field of the S-Record
  UINT8 AddrBytes;
    //number of bytes in the Srecord address
  UINT8 RecType;
    //holds the character received after the Start-of-Record character (S)
  
  
  for(;;)
  {
    
    while (getchar() != 'S'); //get a character. Start of record character?
    
    
    RecType = getchar();      //yes. get the next character
    
   /* if (RecType == '0')       //S0 record?
    {
      SRecData->RecType = HeaderRec;  //yes. mark as a header record
      AddrBytes = 2;          //with 2 address bytes
    } */
   if (RecType == '9')  //S9 record?
    {
      SRecData->RecType = EndRec;     //yes. mark as a end record
      AddrBytes = 2;          //with 2 address bytes    2���ֽڵ�ַ
    }
    else if (RecType == '8')  //S8 record?
    {
      SRecData->RecType = EndRec;     //yes. mark as a end record
      AddrBytes = 3;          //with 3 address bytes
    }
    else if (RecType == '7')  //S7 record?
    {
      SRecData->RecType = EndRec;     //yes. mark as a end record
      AddrBytes = 4;          //with 4 address bytes
    }
    else if (RecType == '2')  //S2 record?
    {
      SRecData->RecType = DataRec;    //yes. mark as a data record
      AddrBytes = 3;          //with 3 address bytes
    } 
    else if (RecType == '1') 
    {
      SRecData->RecType = DataRec;    //yes. mark as a data record
      AddrBytes = 2;
    }
    else
      continue;        //we only receive S0, S1,S2, S7, S8 & S9 S-Records
    
    Error = GetHexByte(&SRecData->NumBytes);
    if (Error != noErr)       //get the record length field. valid hex byte?���յ�һ����Ч16���Ƶ���   
      return(Error);          //no. return an error
    
    //initialize the checksum with the record length
    CheckSum = SRecData->NumBytes;        //��ʼ��У���
    
    //subtract number of bytes in the address+1 to get the length of data field
    NumDataBytes = (SRecData->NumBytes -= (AddrBytes +1));

    
    if (NumDataBytes > MaxSRecLen)  //is the S-Record longer than allowed?
    {
      can_send.data[0]=6;
      MSCAN2SendMsg(can_send);
      *boot=0x66;
      return(SRecTooLong);         //yes. report an error
    }
    
    for (x = 0; x < AddrBytes; x++)         //�ֽڵ�ַ
    {
      Error = GetHexByte(&SRecByte);
      if (Error != noErr)     //get a byte of the load address. valid hex byte?
        return(Error);        //no return an error
      CheckSum += SRecByte;   //add it into the checksum
      LoadAddress = (LoadAddress << 8) + SRecByte;
    } 
    
    SRecData->LoadAddr = LoadAddress;
    
    for (x = 0; x < NumDataBytes; x++)  //receive the data field���յ�S��¼�е������ֽ���
    {
      Error = GetHexByte(&SRecByte);
      if (Error != noErr)     //get a data field byte. valid hex byte?
        return(Error);        //no return an error
      CheckSum += SRecByte;   //yes. add it in to the checksum
      
      SRecData->Data[x] = SRecByte;   //ת��֮����ֽ�
    }
    
    Error = GetHexByte(&SRecByte);
    
    //get the record checksum (last byte of the record). valid hex byte?���һ��У��λ�ֽ�
    if (Error != noErr)
      return(Error);          //no. return an error
      
    CheckSum = ~CheckSum;     //take 1's compliment of calculated checksum
    if (CheckSum != SRecByte) //calculated checksum == received checksum ?
    {
      can_send.data[0]=7;
      MSCAN2SendMsg(can_send);
      *boot=0x66;
      return(CheckSumErr);   //no. return an error
    }
    else 
    {
      return(noErr);
      
    }
      
  }
}
/******************************************************************************/
// 1) convert the logical/local address to global address
// 2) align the start address to 4 words and staff data 0xFFFF accordingly
void SrcdAlign4W(SRecDataRec *srcd_t)
{
  
    word i,j;
    UINT32 stafno;
    
//    Srcd_Logical2Physical_HY64(&srcd);
//    Srcd_Logical2Physical_XEP100(&srcd_t);

    byte *p;
    volatile dword physical_address;
    
    p = (byte *)&srcd_t->LoadAddr;
    p++;
    if(*p != 0)                 //�߼���ַ
    {
      physical_address = (srcd_t->LoadAddr & 0xFF0000)>>16;           //ȡ���߼���ַ�߰�λҳ��
      physical_address = (physical_address - 0xC0) * 0x4000 + 0x700000;
      physical_address = physical_address + ((srcd_t->LoadAddr & 0xFFFF) - 0x8000);   //�õ�ȫ�ֵ�ַ
      
      
      srcd_t->LoadAddr = physical_address;                    //ȫ�ֵ�ַ
    } 
    else                        //�����ַ
    {
      srcd_t->LoadAddr = srcd_t->LoadAddr + 0x7F0000;
    }
    
    if(NumDataBytes != 32) 
    {  
      stafno = (srcd_t->LoadAddr & 0x0000001FUL);      // number of word need to be pre-staffed
      srcd_t->LoadAddr = (srcd_t->LoadAddr - (srcd_t->LoadAddr & 0x0000001FUL));    // align data base address at 4 word LoadAddr[2:0] must be 000
      if(stafno) 
      {  
         for(i=NumDataBytes; i>0; i--) 
           srcd_t->Data[i+stafno-1]=srcd_t->Data[i-1];  // move forward data in number of stafno
       
         NumDataBytes+=stafno;    // update dtlen 
 
         while(stafno--)    
           srcd_t->Data[stafno]=0xFF;           // pre-staff 0xFF
      }

      while(((srcd_t->LoadAddr & 0x0000001FUL) + NumDataBytes) < 32)   // need to append post-staffing word
        srcd_t->Data[NumDataBytes++]=0xFF;
    }
}  
