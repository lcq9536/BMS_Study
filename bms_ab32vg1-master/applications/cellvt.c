/*
  ******************************************************************************
  * @file    cellvt.c
  * @author  Angel_YY
  * @version V1.0.0
  * @date    2021
  * @brief   get cell voltage and temperature.
  ******************************************************************************
*/
#include <rtthread.h>
#include "cellvt.h"
#include "temperature.h"
#include "my_spi.h"
#include "bms.h"
#include "ltc6811.h"
#include "string.h"

/* ����LTC6811�ĸ��� */
cell_asic bms_ic[TOTAL_IC];

int LTC6811_init()
{
    int i;
    spi_init();
    //InitLTC68xx();
    /* �����ʼ��һЩ���ã�ֻ�ǰ�����д�뵽�˽ṹ���� */
    LTC681x_init_cfg(TOTAL_IC, bms_ic);         /* ��������Ϣд������ */
    LTC6811_reset_crc_count(TOTAL_IC,bms_ic);   /* ��ʼ��PEC */
    LTC6811_init_reg_limits(TOTAL_IC,bms_ic);   /* ����6811��һЩ���� */
    for(i=0;i<TOTAL_IC;i++)
    {
        bms_ic[i].config.tx_data[0] = 0xFD;  /* 0B 0000 0101   0xfc|ADCOPT_INIT,//#define ADCOPT_INIT  0 */
        bms_ic[i].config.tx_data[1] = (unsigned char)((((unsigned long)DEFAULT_VOLT_LOW_LIMIT_WARNING*5/8)-1))&0xff; //0xE1;  //Ƿѹ(x+1)*16=20000,��λ����0.1mV��1249 */
        bms_ic[i].config.tx_data[2] = ((unsigned char)(((unsigned long)DEFAULT_VOLT_HIGH_LIMIT_WARNING*5/8)<<4)&0xf0)|\
                ((unsigned char)((((unsigned long)DEFAULT_VOLT_LOW_LIMIT_WARNING*5/8)-1)>>8)&0x0f); //0x84; */
        bms_ic[i].config.tx_data[3] = (unsigned char)(((unsigned long)DEFAULT_VOLT_HIGH_LIMIT_WARNING*5/8)>>4)&0xff;  //0x90;  //��ѹx*16=37000��������0.1mV�� 2312 */
        bms_ic[i].config.tx_data[4] = 0x00;  /* �����ǿ����Ƿ�ŵ�ģ�bitֵΪ1�������ŵ� */
        bms_ic[i].config.tx_data[5] = 0x00;
    }
    wakeup_sleep(TOTAL_IC);          /* ����6811 */
    LTC6811_wrcfg(TOTAL_IC,bms_ic);  /* �������е�������Ϣд��6811 */
    return 0;
}
int GetCellV(Packet_t * pMyPacket){
    int i,m,j;
    long ltemp;
    int iTemp;
    int8_t error = 0;
    
    /* ��ʼADCת�� */
    wakeup_sleep(TOTAL_IC);
    LTC6811_adcv(MD_7KHZ_3KHZ,DCP_DISABLED,CELL_CH_ALL);/* ����3K(2)����ֹ�ŵ�(0)��ͨ����0(0)������2�ֽ����2�ֽ�PEC[03 60 f4 6c]�� */

    /* ��ս��ջ����� */
    for(i = 0;i < BATTNUMONPACK;i++)       /* �������鸳ֵ */
    {
        for(m = 0;m < CELLNUMONEBAT;m++)   /* �����ѹ��ֵ */
        {
            bms_ic[i].cells.c_codes[m] = 0;/* 3001+m; */
        }
        bms_ic[i].stat.stat_codes[0] = 0;
        bms_ic[i].stat.stat_codes[1] = 0;
        bms_ic[i].stat.stat_codes[2] = 0;
        bms_ic[i].stat.stat_codes[3] = 0;
    }

    /* 48�ֽ�ÿһ��IC��48��Ϊ���飬ÿһ��12�ֽڣ���ȡ������о��ѹ���������ֽڶ�ȡ�������������ѹ����6�ֽڡ���2�ֽ�PEC�� */
    error = LTC6811_rdcv(0, TOTAL_IC,bms_ic); /* Cell���(0Ϊ����)��IC��������ȡ���������ݴ�Žṹ�壨���鷢4�����8���ݺ�2PEC��*/
    
    for(i = 0;i < BATTNUMONPACK;i++)      /* �������鸳ֵ */
    {
        for(m = 0;m < CELLNUMONEBAT;m++)  /* �����ѹ��ֵ */
        {
            if(bms_ic[i].cells.c_codes[m] > 45000)
            {
                pMyPacket->MyBat[i].usCellV[m] = 0;
            }else
            {
                pMyPacket->MyBat[i].usCellV[m] = (bms_ic[i].cells.c_codes[m])/10; //����ɼ�����������0.1mVΪ��λ�����ݣ���Ҫת��Ϊ1mVΪ��λ����ֵ
            }
        }
    }
    return 0;
}
int GetCellT(Packet_t * pMyPacket){  
    int i,m,j;
    long ltemp;
    int iTemp;
    int8_t error = 0;
    
    /*��ʼGPIO��ADC���� */
    LTC6811_adax(MD_7KHZ_3KHZ , AUX_CH_ALL);/* ����2�ֽ�����[05 60]��2�ֽ�PEC [D3 A0] */
    //LTC6811_pollAdc();/* �ȴ�ת������ */
    /* ��ȡAUX�ĵ�ѹ���� 2*12�ֽ�[�������ֽ��������8�ֽ�[6�ֽ����ݣ�����ͨ����+2�ֽ�PEC]]*/;
    /* ��ս��ջ����� */
    for(i = 0;i < BATTNUMONPACK;i++)       /* �������鸳ֵ */
    {
        for(j = 0;j < TEMPNUMONEBAT;j++)   /* �����¶ȸ�ֵ */
        {
            bms_ic[i].aux.a_codes[j] = 0;  /* 51+j;*/
        }
    }
    error = LTC6811_rdaux(0,TOTAL_IC,bms_ic); /* ��һ������Ϊ0����ȡ����GPIOͨ���ĵ�ѹֵ*/
    
    for(i = 0;i < BATTNUMONPACK;i++)      /* �������鸳ֵ */
    {
        for(j = 0;j < TEMPNUMONEBAT;j++)  /*�����¶ȸ�ֵ */
        {
            ltemp = bms_ic[i].aux.a_codes[j];
            ltemp = ltemp *13651;
            ltemp = ltemp/100000;        /* 12λADֵ */
            //Temperature((unsigned short)ltemp);
            iTemp = Temperature((unsigned short)ltemp);
            pMyPacket->MyBat[i].ucCellT[j] = iTemp;//Temperature((unsigned short)ltemp);/* �¶ȷ�Χ-40 ~ 125 */
        }
    }

    return 0;
}
    
int GetCellTest(Packet_t * pMyPacket){
    int i,m,j;
    /*----------------------���һ�½ṹ�壬����SPIͨѶ��ȡ������Ϣ--------- */
    for(i = 0;i < BATTNUMONPACK;i++)      /* �������鸳ֵ */
    {
        for(m = 0;m < CELLNUMONEBAT;m++)  /* �����ѹ��ֵ */
        {
            pMyPacket->MyBat[i].usCellV[m] =  3001+m;  
        }
        for(j = 0;j < TEMPNUMONEBAT;j++)  /*�����¶ȸ�ֵ */
        {
            pMyPacket->MyBat[i].ucCellT[j] = 20+j;/* �¶ȷ�Χ-40 ~ 125 */
        }
    }
    
    return 0;
}

void ProcessData(Packet_t * pMyPacket,Packet_Alarm_Val_t * pstAlarm,uint32_t *puDataOverFlag) {

    int i,j,m;
    char ucMaxT  = 0,ucMinT = 127;
    float usMaxV = 0.0f,usMinV = 5.0f;
    unsigned short usMaxCellLocal = 0,usMinCellLocal = 0;
    float ulTolV = 0.0f,ulTolT = 0.0f;
    float  l_WeighCellV;
    //float  l_CellVTemp[3]; 
    /* ��������¶ȵ�ѹ����С�¶ȵ�ѹ���г�ʼ�� */
    usMaxV = usMinV = pMyPacket->MyBat[0].usCellV[0];
    ucMaxT = ucMinT = pMyPacket->MyBat[0].ucCellT[0];
    usMaxCellLocal  = 0x0101;
    usMinCellLocal  = 0x0101;

    for(i = 0;i < BATTNUMONPACK;i++)    /* ���6811����ѭ�� */
    {
        for(j = 0;j < CELLNUMONEBAT;j++)/* ����������ѭ�� */
        {
            if(pMyPacket->MyBat[i].usCellV[j] > usMaxV)
            {
                usMaxV = pMyPacket->MyBat[i].usCellV[j];    /* ��õ�������ѹ */
                usMaxCellLocal = ((i + 1) << 8) + j + 1;    /* �������ѹ����λ�� */
            }
            if(pMyPacket->MyBat[i].usCellV[j] < usMinV)
            {
                usMinV = pMyPacket->MyBat[i].usCellV[j];    /* �����С��ѹ */
                usMinCellLocal=((i + 1) << 8) + j + 1;      /* �����С��ѹ����λ�� */
            }
            ulTolV+=pMyPacket->MyBat[i].usCellV[j];
        }
        for(m = 0;m < TEMPNUMONEBAT;m++)
        { 
            if(pMyPacket->MyBat[i].ucCellT[m] > ucMaxT)
            {
                ucMaxT = pMyPacket->MyBat[i].ucCellT[m];
            }
            if(pMyPacket->MyBat[i].ucCellT[m] < ucMinT)
            {
                ucMinT = pMyPacket->MyBat[i].ucCellT[m];
            }
            ulTolT += pMyPacket->MyBat[i].ucCellT[m];
        }
    }
    /* �Ѽ���õ�����ߵ�ѹ����Ϣ���浽�ṹ�� */
    pMyPacket->usMaxCellV = usMaxV;
    pMyPacket->usMinCellV = usMinV;
    pMyPacket->ucMaxVLocal= usMaxCellLocal;
    pMyPacket->ucMinVLocal= usMinCellLocal;
    pMyPacket->ucMaxCellT = ucMaxT;
    pMyPacket->ucMinCellT = ucMinT;
    pMyPacket->usEvgCellV = ulTolV / (BATTNUMONPACK*CELLNUMONEBAT);  /* ����ƽ����ѹ */
    pMyPacket->ucEvgCellT = ulTolT / (BATTNUMONPACK*TEMPNUMONEBAT);  /* ����ƽ���¶� */

    // *puDataOverFlag
    if(pMyPacket->usMaxCellV > pstAlarm->fCellVoltH){/* �����ѹ���� */
        *puDataOverFlag |= (1 << 15);     /* ��־λ��1 */
    }else
    {
        *puDataOverFlag &= ~(1 << 15);    /* ��־λ���� */
    }
    if(pMyPacket->usMinCellV < pstAlarm->fCellVoltL) /* �����ѹ���� */
    {
        *puDataOverFlag |= (1 << 14);     /* ��־λ��1 */
    }
    else{
        *puDataOverFlag &= ~(1 << 14);    /* ��־λ���� */
    }
    if(pMyPacket->usMaxCellV-pMyPacket->usMinCellV > pstAlarm->fCellVoltDiff)  /* ������ߡ���͵�ѹ֮���ѹ���һ��ֵ���б����˴���100mV */
    {
        *puDataOverFlag |= (1 << 13);     /* ��־λ��1 */
    }
    else{
        *puDataOverFlag &= ~(1 << 13);    /* ��־λ���� */
    }

    if(pMyPacket->ucMaxCellT > pstAlarm->iCellTempH)  /* �����¶ȹ��� */
    {
        *puDataOverFlag |= (1 << 12);
    }else{
        *puDataOverFlag &= ~(1 << 12);         /* ��־λ���� */
    }

    if(pMyPacket->ucMinCellT < pstAlarm->iCellTempL)  /* �����¶ȹ��� */
    {
        *puDataOverFlag |= (1 << 11);
    }else{
        *puDataOverFlag &= ~(1 << 11);         /* ��־λ���� */
    }

    if(pMyPacket->ucMaxCellT - pMyPacket->ucMinCellT> pstAlarm->iCellTempDiff)  /* ������ߡ�����¶�֮����²��һ��ֵ���б����˴���10�� */
    {
        *puDataOverFlag |= (1 << 10);
    }else{
        *puDataOverFlag &= ~(1 << 10);         /* ��־λ���� */
    }
}

int CheckWarning()
{
    unsigned char tempbuf[100];
    int s=0;
    unsigned char tmplen;
    /* ���������Ƿ��д���������쳣ֵ���� */
    if(uDataOverFlag)//������쳣���� */
    {
        memset(tempbuf,0,100);  /* ע�⣺���ﻺ����ֻ��100�ֽڣ������8�ֻ������쳣ͬʱ����������ͻ���� */
        if((uDataOverFlag >> 23) & 0x01){
            s++;
            if(s<3)
            strcat((char *)tempbuf,"PackV Over,");
        }
        if((uDataOverFlag >> 22) & 0x01){
            s++;
            if(s<3)
            strcat((char *)tempbuf,"PackV Low,");
        }
        if((uDataOverFlag >> 21) & 0x01){
            s++;
            if(s<3)
            strcat((char *)tempbuf,"Charge Over,");
        }
        if((uDataOverFlag >> 20) & 0x01){
            s++;
            if(s<3)
            strcat((char *)tempbuf,"DisCh Over,");
        }
        if((uDataOverFlag >> 15) & 0x01){
            s++;
            if(s<3)
            strcat((char *)tempbuf,"CellV Over,");
        }
         if((uDataOverFlag >> 14) & 0x01){
             s++;
             if(s<3)
            strcat((char *)tempbuf,"CellV Low,");
        }
        if((uDataOverFlag >> 13) & 0x01){
            s++;
            if(s<3)
            strcat((char *)tempbuf,"CellV Diff Over,");
        }
         if((uDataOverFlag >> 12) & 0x01){
             s++;
             if(s<3)
            strcat((char *)tempbuf,"CellT Over,");
        }
        if((uDataOverFlag >> 11) & 0x01){
            s++;
            if(s<3)
            strcat((char *)tempbuf,"CellT Low,");
        }
         if((uDataOverFlag >> 10) & 0x01){
             s++;
             if(s<3)
            strcat((char *)tempbuf,"CellT Diff Over,");
        }

        tmplen = strlen((char *)tempbuf);
        if(tmplen > 3)
        {
            tempbuf[tmplen-1] = 0x20;
        }

        /* ���쳣��Ϣ��ӡ���� */
        rt_kprintf("\r\n!!! Warring:%s [%d]!!!\r\n\r\n",tempbuf,s);
    }
    return 0;
}

void PrintCellMsg(void)
{
    int i,m,j;
    for(i = 0;i < BATTNUMONPACK;i++)      /* �������� */
    {
        rt_kprintf("Cell-V",i+1);
        for(m = 0;m < CELLNUMONEBAT;m++)  /* �����ѹ */
        {
            rt_kprintf("[%02d:%04d]",m+1,MyPacket.MyBat[i].usCellV[m]); 
        }
        rt_kprintf("\r\nCell-T");
        for(j = 0;j < TEMPNUMONEBAT;j++)  /*�����¶� */
        {
            rt_kprintf("[%d:%d]",j+1,MyPacket.MyBat[i].ucCellT[j]);
        }
        rt_kprintf("\r\n");
    }
}
