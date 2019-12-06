/**
  ******************************************************************************
  * @file    usbh_cdc_core.h
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file contains all the prototypes for the usbh_cdc_core.c
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_CDC_CORE_H
#define __USBH_CDC_CORE_H

/* Includes ------------------------------------------------------------------*/
#include "usbh_cdc_funct.h"


/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_CLASS
* @{
*/

/** @addtogroup USBH_CDC_CLASS
* @{
*/

/** @defgroup USBH_CDC_CORE
* @brief This file is the Header file for USBH_CDC_CORE.c
* @{
*/ 




/*Comuncation Calss codes*/
#define COMMUNICATION_DEVICE_CLASS_CODE                         0x02
#define COMMUNICATION_INTERFACE_CLASS_CODE                      0x02

/*Data Interface Class Codes*/
#define DATA_INTERFACE_CLASS_CODE                               0x0A

/*Communcation sub class codes*/
#define RESERVED                                                0x00
#define DIRECT_LINE_CONTROL_MODEL                               0x01
#define ABSTRACT_CONTROL_MODEL                                  0x02
#define TELEPHONE_CONTROL_MODEL                                 0x03
#define MULTICHANNEL_CONTROL_MODEL                              0x04   
#define CAPI_CONTROL_MODEL                                      0x05
#define ETHERNET_NETWORKING_CONTROL_MODEL                       0x06
#define ATM_NETWORKING_CONTROL_MODEL                            0x07


/*Communication Interface Class Control Protocol Codes*/
#define NO_CLASS_SPECIFIC_PROTOCOL_CODE                         0x00
#define COMMON_AT_COMMAND                                       0x01
#define VENDOR_SPECIFIC                                         0xFF


#define CS_INTERFACE                                            0x24
#define CDC_PAGE_SIZE_64                                        0x40




/** @defgroup USBH_CDC_CORE_Exported_Types
* @{
*/ 






/* States for CDC State Machine */
typedef enum
{
  CDC_IDLE= 0,
  CDC_READ_DATA,
  CDC_SEND_DATA,
  CDC_DATA_SENT,
  CDC_BUSY,
  CDC_GET_DATA,   
  CDC_POLL,
  CDC_CTRL_STATE
}
CDC_State;

/* CDC Transfer State */
typedef struct _CDCXfer
{
  volatile CDC_State CDCState;
  uint8_t* pRxTxBuff;
  uint8_t* pFillBuff;
  uint8_t* pEmptyBuff;
  uint32_t BufferLen;
  uint16_t DataLength;
} CDC_Xfer_TypeDef;

typedef struct CDC_UserCb
{
  void  (*Send)       (uint8_t  *);             
  void  (*Receive)    (uint8_t *);       
  
} CDC_Usercb_TypeDef;

/* Structure for CDC process */
typedef struct _CDC_CommInterface
{
  uint8_t              hc_num_in; 
  uint8_t              hc_num_out;
  uint8_t              notificationEp;
  CDC_State            state; 
  uint8_t              buff[8];
  uint16_t             length;
  uint8_t              ep_addr;  
}
CDC_CommInterface_Typedef ;

typedef struct _CDC_DataInterface
{
  uint8_t              hc_num_in; 
  uint8_t              hc_num_out;
  uint8_t              cdcOutEp;
  uint8_t              cdcInEp;
  CDC_State            state; 
  uint8_t              buff[8];
  uint16_t             length;
  uint8_t              ep_addr;
}
CDC_DataInterface_Typedef ;

/* Structure for CDC process */
typedef struct _CDC_Process
{
  CDC_CommInterface_Typedef CDC_CommItf;
  CDC_DataInterface_Typedef CDC_DataItf;
}
CDC_Machine_TypeDef;

/**
* @}
*/ 

/** @defgroup USBH_CDC_CORE_Exported_Defines
* @{
*/ 

/**
* @}
*/ 

/** @defgroup USBH_CDC_CORE_Exported_Macros
* @{
*/ 
/**
* @}
*/ 

/** @defgroup USBH_CDC_CORE_Exported_Variables
* @{
*/ 
extern USBH_Class_cb_TypeDef  CDC_cb;

/**
* @}
*/ 

/** @defgroup USBH_CDC_CORE_Exported_FunctionsPrototype
* @{
*/ 
void  CDC_SendData(uint8_t *data, uint16_t length);
void  CDC_StartReception( USB_OTG_CORE_HANDLE *pdev);
void  CDC_StopReception( USB_OTG_CORE_HANDLE *pdev);
/**
* @}
*/ 


#endif /* __USBH_CDC_CORE_H */

/**
* @}
*/ 

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

