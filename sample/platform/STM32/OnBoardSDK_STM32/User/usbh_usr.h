/**
  ******************************************************************************
  * @file    usbh_usr.h
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file is the header file for usb usr file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      <http://www.st.com/SLA0044>
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USH_USR_H__
#define __USH_USR_H__


/* Includes ------------------------------------------------------------------*/
#include "usbh_core.h"
#include "usb_conf.h"
#include <stdio.h>

/** @addtogroup USBH_USER
* @{
*/

/** @addtogroup USBH_HID_CDC_USER_CALLBACKS
* @{
*/
  
/** @defgroup USBH_USR 
  * @brief This file is the header file for user action
  * @{
  */ 

#define LINE_01                           (LCD_PIXEL_HEIGHT - 36)
#define LINE_02                           (LCD_PIXEL_HEIGHT - 24)
#define LINE_03                           (LCD_PIXEL_HEIGHT - 12) 

#define NB_BAUD_RATES                     10
#define NB_OF_FILES                       3

typedef enum {
  CDC_DEMO_IDLE   = 0,
  CDC_DEMO_WAIT,  
  CDC_DEMO_SEND,
  CDC_DEMO_RECEIVE,
  CDC_DEMO_CONFIGURATION,  
}CDC_Demo_State;

typedef enum {
  CDC_SEND_IDLE   = 0,
  CDC_SEND_WAIT,  
  CDC_SEND_FILE1,  
  CDC_SEND_FILE2,  
  CDC_SEND_FILE3,    
}CDC_Send_State;

typedef enum {
  CDC_RECEIVE_IDLE   = 0,
  CDC_RECEIVE_WAIT,    
  CDC_RECEIVE_RECEIVE,      
}CDC_Receive_State;


typedef enum {
  CDC_CONFIGURATION_IDLE   = 0,
  CDC_CONFIGURATION_WAIT,    
  CDC_CONFIGURATION_SET_BAUD_RATE,   
  CDC_CONFIGURATION_SET_DATA_BITS,    
  CDC_CONFIGURATION_SET_PARITY,    
  CDC_CONFIGURATION_SET_STOP_BIT,     
}CDC_Configuration_State;

typedef struct _DemoStateMachine
{
  
  __IO CDC_Demo_State              state;
  __IO CDC_Send_State              Send_state;
  __IO CDC_Receive_State           Receive_state;
  __IO CDC_Configuration_State     Configuration_state;    
  __IO uint8_t                     select;
  __IO uint8_t                     lock;
  
}CDC_DEMO_StateMachine;

typedef struct _DemoSettings
{
  __IO uint8_t                     BaudRateIdx;
  __IO uint8_t                     DataBitsIdx;
  __IO uint8_t                     ParityIdx;
  __IO uint8_t                     StopBitsIdx;
  
}CDC_DEMO_Settings;

typedef struct _DemoSettingStateMachine
{
  
  CDC_DEMO_Settings                settings;
  __IO uint8_t                     select;
  __IO uint8_t                     lock;
  
}CDC_DEMO_SETTING_StateMachine;

typedef enum 
{
  CDC_SELECT_MENU   = 0,
  CDC_SELECT_CONFIG,    
}CDC_DEMO_SelectMode;
/** @defgroup USBH_CORE_Exported_Variables
  * @{
  */ 


extern  USBH_Usr_cb_TypeDef USR_Callbacks;


/**
  * @}
  */ 


/** @defgroup USBH_CORE_Exported_FunctionsPrototype
  * @{
  */ 

void USBH_USR_ApplicationSelected(void);
void USBH_USR_Init(void);
void USBH_USR_DeInit(void);
void USBH_USR_DeviceAttached(void);
void USBH_USR_ResetDevice(void);
void USBH_USR_DeviceDisconnected (void);
void USBH_USR_OverCurrentDetected (void);
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed); 
void USBH_USR_Device_DescAvailable(void *);
void USBH_USR_DeviceAddressAssigned(void);
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc);
void USBH_USR_Manufacturer_String(void *);
void USBH_USR_Product_String(void *);
void USBH_USR_SerialNum_String(void *);
void USBH_USR_EnumerationDone(void);
USBH_USR_Status USBH_USR_UserInput(void);
int  USBH_USR_Application(void);
void USBH_USR_DeInit(void);
void USBH_USR_DeviceNotSupported(void);
void USBH_USR_UnrecoveredError(void);


//void CDC_DEMO_ProbeKey (JOYState_TypeDef state);
/**
  * @}
  */ 

#endif /* __USBH_USR_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
