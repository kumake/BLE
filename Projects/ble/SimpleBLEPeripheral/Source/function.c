//#include <ioCC2540.h>



#include "gap.h"
#include "OSAL.h"
#include "hal_types.h"

#include "hal_defs.h"
#include "function.h"
#include "simpleBLEPeripheral.h"
#include "simpleGATTprofile.h"
#include "hal_led.h"

Global_Data global_data;

Global_Data default_global_data ={
.factory_flag=0xEA,
.dev_name="YY_iBeacon",
.uuid={0xFD,0xA5,0x06,0x93,0xA4,0xE2,0x4F,0xB1,0xAF,0xCF,0xC6,0xEB,0x07,0x64,0x78,0x25},
.major={0x00,0x0A},
.minor={0x00,0x07},
.mp={0x12},
.tx_power=0,
};




uint8 POWER_ON=TRUE;

uint16 POWER_ON_TIME =0;//��������



/****************************************************************************
��    ��: DelayMS()
��    ��: ������ʱ��ϵͳʱ�Ӳ�����ʱĬ��ʹ���ڲ�16M
��ڲ���: msec ��ʱ������ע�⣬�����ֵΪ65536ms
���ڲ���: ��
****************************************************************************/
void DelayMS(uint16 msec)
{ 
    uint16 i,j;
    
    for (i=0; i<msec; i++)
        for (j=0; j<536; j++);
}


/****************************************************************************
��    ��: PowerMode()
��    ��: ����ϵͳ����ģʽ
��ڲ���: PM0:0��PM1:1��PM2:2��PM3:3 ������0�ָ�����ģʽ         
****************************************************************************/
void PowerMode(uint8 mode) 
{ 
    if(mode>0 && mode < 4) 
    {  
        SLEEPCMD |= mode;    //����ϵͳ˯��ģʽ 
        PCON |= BV(0);         //����PowerMode˯��ģʽ
    }else 
        PCON &= ~BV(0);         //��PowerMode�ָ�
}


/****************************************************************************
��    ��: InitKey()
��    ��: ���ð�����Ӧ��IO��
��ڲ���: ��
���ڲ���: ��
****************************************************************************/
void InitKey(void)
{
    P0IEN |= BV(1);    // P0.1 ����Ϊ�жϷ�ʽ 1���ж�ʹ��
    #if (KEY_TRIGGER_RISING)
    //������
    PICTL &= ~BV(0);    //�˿� �����ش���
    #else
    //�½���
    PICTL |= BV(0);    //�˿� �����ش���
    #endif
    IEN1  |= BV(5);    //���� P0 ���ж�
    P0IFG = 0x00;    //��ʼ���жϱ�־λ
    EA = 1; //�����ж�
}


int16 mystrlen (const char * str)
{
	int16 length = 0;
	while( *str++ )
	++length;
	return( length );
}

uint8 *Build_ScanRspData(Global_Data gd,uint8 *len)
{
	uint8 * retp;

	uint8 data1[]={
	   // complete name
	  0x14,   // length of this data
	  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
	};

	uint8 data2[]={
		  // connection interval range
		  0x05,   // length of this data
		  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
		  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
		  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
		  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
		  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),
		  // Tx power level
		  0x02,   // length of this data
		  GAP_ADTYPE_POWER_LEVEL,
		  gd.tx_power// 0dBm//���书��
	};
	
	uint16 dev_name_len=mystrlen((const char *)gd.dev_name);

	//�ܳ���
	*len=dev_name_len+sizeof(data1)+sizeof(data2);
	
	retp=osal_msg_allocate(*len);
	VOID osal_memcpy( retp, data1, sizeof(data1) );//����ͷ
	retp[0]=dev_name_len+1;//�����豸������
	VOID osal_memcpy( retp+sizeof(data1) , gd.dev_name, dev_name_len );//�����豸��
	
	VOID osal_memcpy( retp+sizeof(data1)+dev_name_len, data2, sizeof(data2) ); //β��

	return retp;

}



uint8 *Build_AdvData(Global_Data gd,uint8 *len)
{
	uint8 * retp;

	uint8 data1[]={ 
		  // Flags; this sets the device to use limited discoverable
		  // mode (advertises for 30 seconds at a time) instead of general
		  // discoverable mode (advertises indefinitely)
		  0x02,   // length of this data
		  GAP_ADTYPE_FLAGS,
		  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

		  // in this peripheral
		  0x1A,   // length of this data 26byte
		  GAP_ADTYPE_MANUFACTURER_SPECIFIC,  
		  /*Apple Pre-Amble*/
		  0x4C,
		  0x00,
		  0x02,
		  0x15
	 };
	
	//�ܳ���
	*len=sizeof(data1)+sizeof(gd.uuid)+sizeof(gd.major)+sizeof(gd.minor)+sizeof(gd.mp);
	
	retp=osal_msg_allocate(*len);

	VOID osal_memcpy( retp, data1, sizeof(data1) );//����ͷ

	
	VOID osal_memcpy( retp+sizeof(data1) , gd.uuid, 16 );//����uuid

	VOID osal_memcpy( retp+sizeof(data1) +16, gd.major, 2 );//����major

	VOID osal_memcpy( retp+sizeof(data1) + 16+2, gd.minor, 2 );//����minor
	
	//VOID osal_memcpy( retp+sizeof(data1) + 16+2+2, gd.mp, 1 );//����MP
	
	*(retp+sizeof(data1) + 16+2+2)=gd.mp;//����MP
	
	return retp;

}

void Prepare_Profile(void)
{
/*
	VOID osal_memcpy( simpleProfileChar1, global_data.dev_name, SIMPLEPROFILE_CHAR1_LEN );//�豸��
	VOID osal_memcpy( simpleProfileChar2, global_data.uuid, SIMPLEPROFILE_CHAR2_LEN );//UUID
	VOID osal_memcpy( simpleProfileChar3, global_data.major, SIMPLEPROFILE_CHAR3_LEN );//major
	VOID osal_memcpy( simpleProfileChar4, global_data.minor, SIMPLEPROFILE_CHAR4_LEN );//minor
	VOID osal_memcpy( simpleProfileChar5, global_data.mp, SIMPLEPROFILE_CHAR5_LEN );//mp
*/
	SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, global_data.dev_name );
	SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, global_data.uuid );
	SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, global_data.major );
	SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, global_data.minor );
	SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, &global_data.mp );
}



void System_Down(void) //ϵͳ�ػ�
{
	if (POWER_ON)
 	{
		POWER_ON=FALSE;
		//osal_pwrmgr_device( PWRMGR_BATTERY );
		HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_OFF );//by xiaoku
		//InitKey();
		IEN1  = BV(5);    //ֻ���� P0 ���ж�
		PowerMode(3);
		//while ( (CLKCONSTA & 0x80) != 0 );      // �ȴ�˯��ʱ���л����ⲿ32K����
	}
}


void System_Restart(void) //ϵͳ����
{

	osal_int_disable( INTS_ALL );
	HAL_SYSTEM_RESET();
}



