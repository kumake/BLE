/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#include "function.h"


#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
/*
// How often to perform periodic event
//#define SBP_PERIODIC_EVT_PERIOD                   5000   //by xiaoku
#define SBP_PERIODIC_EVT_PERIOD                   1000   //by xiaoku

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
//#define DEFAULT_ADVERTISING_INTERVAL          160 //by xiaoku
#define DEFAULT_ADVERTISING_INTERVAL          1600 //by xiaoku

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif
*/
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

/*
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

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
  0       // 0dBm
};
*/

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
/*
static uint8 advertData[] =
{

  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),
  
//------------------------------------------------------------
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // in this peripheral
  0x1A,   // length of this data 26byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,  
  //Apple Pre-Amble
  0x4C,
  0x00,
  0x02,
  0x15,
  //Device UUID (16 Bytes)
    0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F,0xB1, 0xAF, 0xCF, 0xC6, 0xEB, 0x07, 0x64, 0x78, 0x25,
    
  //Major Value (2 Bytes)
  0x00, 0x0A,
  
  //Minor Value (2 Bytes)
  0x00,0x07,
  
  //Measured Power
  0xCD

};
*/
static uint8 *scanRspData;
static uint8 scanRspData_len;

static uint8 *advertData;
static uint8 advertData_len;




// GAP GATT Attributes
//static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "aaaple BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

//#if defined( CC2540_MINIDK )//by xiaoku
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
//#endif //by xiaoku

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{

  simpleBLEPeripheral_TaskID = task_id;


  //从SNV中读取数据
  if((osal_snv_read(0x80,sizeof(Global_Data),(void *)&global_data)!=SUCCESS)||(global_data.factory_flag!=0xEA))
  {
		//存入默认值
		osal_snv_write(0x80,sizeof(Global_Data),(void *)&default_global_data);
                //重启
                System_Restart();
  }


  scanRspData=Build_ScanRspData(global_data,&scanRspData_len);
  advertData=Build_AdvData(global_data,&advertData_len);

  //----------------------------------------------------------


  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
     uint8 initial_advertising_enable = TRUE;
     //uint8 initial_advertising_enable = FALSE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

 //   uint16 advType = GAP_ADTYPE_ADV_NONCONN_IND;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

//by xiaoku
//只广播 不可连接
 //   GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );

	//by xiaoku
    //GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    //GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, scanRspData_len, scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, advertData_len, advertData );


    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );

 }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    //uint8 charValue1 = 1;
    //uint16 charValue2 = 2;
    //uint16 charValue3 = 3;
    //uint8 charValue4 = 4;
   // uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
    //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
   // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint16 ), &charValue2 );
    //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint16 ), &charValue3 );
   // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    Prepare_Profile();//by xiaoku
  }

/*//by xiaoku
#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile
*/
  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

  // makes sure LEDs are off
  //HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );//by xiaoku
 HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_ON );//by xiaoku
  //HalLedBlink( HAL_LED_1,2, 50,500 );


//nv存储测试
/*
uint8 testbuf[]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8 testbuf2[50];
osal_snv_write(0x80,50,&testbuf);

if (osal_snv_write(0x81,50,&testbuf)==SUCCESS)
{
	if(osal_snv_read(0x80,50,&testbuf2)!=NV_OPER_FAILED)
	{
		//advertData[29]=gMP;
		testbuf[29]=testbuf2[49];
  	}

}
*/

  
//by xiaoku
/*
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
*/
  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )


  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
   osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    //osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, 1000 );
  // InitKey();
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    //if ( SBP_PERIODIC_EVT_PERIOD )//by xiaoku
    if (( SBP_PERIODIC_EVT_PERIOD )&&(POWER_ON_TIME++<REFUSE_CONNECT_TIME)) //by xiaoku
    {
      osal_start_timerEx(simpleBLEPeripheral_TaskID , SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }

  if ( events & SBP_POWER_OFF_EVT ) //by xiaoku
  {
  	System_Down() ;//关机
  	/*
	if (POWER_ON)
 	{
		POWER_ON=FALSE;
		//osal_pwrmgr_device( PWRMGR_BATTERY );
		HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_OFF );//by xiaoku
		//InitKey();
		IEN1  = BV(5);    //只允许 P0 口中断
		PowerMode(3);
		//while ( (CLKCONSTA & 0x80) != 0 );      // 等待睡眠时钟切换到外部32K晶振
	}
	
	else
	{
		POWER_ON=TRUE;
		HAL_SYSTEM_RESET();
	}
	*/
	 
  	return ( events ^ SBP_POWER_OFF_EVT );
  	
  }


 

#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  /*  //by xiaoku
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )
*/
   case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  

  default:
    // do nothing
    break;
  }
}



//#if defined( CC2540_MINIDK ) //by xiaoku
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */


static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
 // uint8 SK_Keys = 0; //by xiaoku

//  VOID shift;  // Intentionally unreferenced parameter //by xiaoku
/*
  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }

  }
*///by xiaoku
/*
  if ( keys & HAL_KEY_SW_6 )
 {
 	if (POWER_ON)
 	{
		POWER_ON=FALSE;
		HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_OFF );//by xiaoku
		PowerMode(3);
	}else
	{

	//重启
		HAL_SYSTEM_RESET();

	}
 //}
 */
  //HalLedBlink( HAL_LED_1,1, 50,1000 );
  //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
  //osal_pwrmgr_device( PWRMGR_BATTERY );
	// Set the GAP Role Parameters
	//uint8 initial_advertising_enable=FALSE;
    //GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable);  	
		//
	//	osal_pwrmgr_device( PWRMGR_BATTERY );
         //       HalLedBlink( HAL_LED_1,5, 50,1000 );


  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
 // SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys ); //by xiaoku
}
//#endif // #if defined( CC2540_MINIDK )//by xiaoku

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);



        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING://断开连接事件
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
	 System_Restart();//重启
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
//by xiaoku
  //uint8 valueToCopy;
  //uint8 stat;

	//POWER_ON=FALSE;
	//	HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_OFF );//by xiaoku
	//	P0IFG = 0;       //清中断标志  
    	//	P0IF = 0;        //清中断标志 
    	//	EA=1;
	//	PowerMode(3);

  // Call to retrieve the value of the third characteristic in the profile
  //stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);

 // if( stat == SUCCESS )
 // {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
   // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
  //}
  	//if (POWER_ON_TIME>=1)
 		//osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
	//else
		//osal_pwrmgr_device( PWRMGR_BATTERY );

	if (POWER_ON_TIME>=REFUSE_CONNECT_TIME)
	{
		uint16 advType = GAP_ADTYPE_ADV_NONCONN_IND;
    		GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
	}
	
//HalLedBlink( HAL_LED_1,2, 50,100 );
//PowerMode(2);
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{


  switch( paramID )
  {
  /*
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    default:
      // should not reach here!
      break;
*/
    uint8 newValue[19];
    case SIMPLEPROFILE_CHAR1:
	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newValue );
	VOID osal_memcpy( global_data.dev_name, newValue, SIMPLEPROFILE_CHAR1_LEN );
	osal_snv_write(0x80,sizeof(Global_Data),(void *)&global_data);//存储
      break;
   
    case SIMPLEPROFILE_CHAR2:
	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2, newValue );
	VOID osal_memcpy( global_data.uuid, newValue, SIMPLEPROFILE_CHAR2_LEN );
	osal_snv_write(0x80,sizeof(Global_Data),(void *)&global_data);//存储
      break;
	  
    case SIMPLEPROFILE_CHAR3:
	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, newValue );
	VOID osal_memcpy( global_data.major, newValue, SIMPLEPROFILE_CHAR3_LEN );
	osal_snv_write(0x80,sizeof(Global_Data),(void *)&global_data);//存储
      break;
	  
    case SIMPLEPROFILE_CHAR4:
	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR4, newValue );
	VOID osal_memcpy( global_data.minor, newValue, SIMPLEPROFILE_CHAR4_LEN );
	osal_snv_write(0x80,sizeof(Global_Data),(void *)&global_data);//存储
      break;
	  
    case SIMPLEPROFILE_CHAR5:
	SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, newValue );
	global_data.mp=newValue[0];
	osal_snv_write(0x80,sizeof(Global_Data),(void *)&global_data);//存储
      break;
	  
    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)







/**************************************************************************** 
名    称: P0_ISR(void) 中断处理函数  
描    述: #pragma vector = 中断向量，紧接着是中断处理程序 
****************************************************************************/ 


#pragma vector = P0INT_VECTOR     
__interrupt void KEY1_ISR(void)  
{
/*
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
    EA=0;
    PowerMode(0);

    DelayMS(10);     //延时去抖 ,不管是中断还是轮询，都需要去抖动，否则干扰很大
    if(KEY1==1){
      PowerMode(3);   //进入正常工作模式
    }

    P0IFG = 0;       //清中断标志  
    P0IF = 0;        //清中断标志 
    EA=1;
*/
  
  
 /*
PowerMode(0);
  //EA=0;
    //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
    
    
    while ( (CLKCONSTA & ~0x80) != 0 );     // 等待主时钟切换到32M晶振
    
    DelayMS(5);     //延时去抖 ,不管是中断还是轮询，都需要去抖动，否则干扰很大

    if(KEY1==1){
	if (POWER_ON)
 	{
		POWER_ON=FALSE;
		HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_OFF );//by xiaoku
		P0IFG = 0;       //清中断标志  
    		P0IF = 0;        //清中断标志 
    		EA=1;
		PowerMode(3);
	}else
	{

	//重启 
              P0IFG = 0;       //清中断标志  
              P0IF = 0;        //清中断标志 
                EA=1;
                //HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_ON );//by xiaoku
                POWER_ON=TRUE;
                while(KEY1==0);
                HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_ON );//by xiaoku
		HAL_SYSTEM_RESET();

	}
    }
*/
    DelayMS(15);     //延时去抖 ,不管是中断还是轮询，都需要去抖动，否则干扰很大

#if (KEY_TRIGGER_RISING)
//上升沿
    if(KEY1==1){
#else
//下降沿
    if(KEY1==0){
#endif
	if (POWER_ON)
 	{ 
 		
 		PowerMode(0);
		#if (KEY_TRIGGER_RISING)
		//上升沿
                while(KEY1==1);
		#else
		//下降沿
    		while(KEY1==0);
    		#endif
		//osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
		osal_set_event( simpleBLEPeripheral_TaskID, SBP_POWER_OFF_EVT );
			
	}else
	{
		osal_int_disable( INTS_ALL );
                #if (KEY_TRIGGER_RISING)
		//上升沿
                while(KEY1==1);
		#else
		//下降沿
    		while(KEY1==0);
    		#endif
                HAL_SYSTEM_RESET();
		//P0IFG = 0;       //清中断标志  
    		//P0IF = 0;        //清中断标志 
                /*
                PowerMode(0);
                while ( (CLKCONSTA & ~0x80) != 0 );     // 等待主时钟切换到32M晶振
                //HalLedSet( (HAL_LED_1 ), HAL_LED_MODE_ON );//by xiaoku
		//HAL_SYSTEM_RESET();
		//return ;
		osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
		osal_set_event( simpleBLEPeripheral_TaskID, SBP_POWER_OFF_EVT );
                */
	}
    }

    P0IFG = 0;       //清中断标志  
    P0IF = 0;        //清中断标志 
   
   
}







/*********************************************************************
*********************************************************************/
