
#ifndef FUNCTION_H
#define FUNCTION_H

#ifdef __cplusplus
extern "C"
{
#endif

#define KEY_TRIGGER_RISING	1


#define KEY1 P0_1       //����P0.1Ϊ��������˿�
 
extern uint8 POWER_ON ;

extern uint16 POWER_ON_TIME;
#define REFUSE_CONNECT_TIME  (10*60)       //�ܾ����ӵ�ʱ��



#define DEV_NAME_MAX_LEN	19

typedef struct
{
  
  uint8 factory_flag;//������־
  uint8 dev_name[DEV_NAME_MAX_LEN];//�豸����
  uint8 uuid[16]; 
  uint8 major[2];
  uint8 minor[2];
  uint8 mp;//����ֵ
  uint8 tx_power;//���书��

  //�㲥���
  //����
} Global_Data;



extern Global_Data global_data;
extern  Global_Data default_global_data; 


extern void DelayMS(uint16 msec);
extern void PowerMode(uint8 mode) ;
extern  void InitKey(void);

uint8 *Build_ScanRspData(Global_Data gd,uint8 * len);
uint8 *Build_AdvData(Global_Data gd,uint8 * len);

void Prepare_Profile(void);

void System_Down(void); //ϵͳ�ػ�
void System_Restart(void); //ϵͳ����

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEBLEPERIPHERAL_H */
