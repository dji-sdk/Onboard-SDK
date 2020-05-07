/**
  ******************************************************************************
  * @file    usbh_usr.cpp
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
#ifdef __cplusplus
extern "C" {
#endif
#include "osdkhal_stm32.h"
#include "usbh_usr.h"
#include "usbh_cdc_core.h"
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#ifdef __cplusplus
}
#endif
#include "dji_log.hpp"

#ifdef OSDK_USB_DEBUG
#define USB_DEBUG DSTATUS
#else
#define USB_DEBUG
#endif

extern USB_OTG_CORE_HANDLE USB_OTG_Core;
extern USBH_HOST  USB_Host;
extern CDC_LineCodingTypeDef         CDC_GetLineCode;
extern CDC_LineCodingTypeDef         CDC_SetLineCode;
extern CDC_Usercb_TypeDef            UserCb;

CDC_Demo_State 						           CDCUsrState;

extern QueueHandle_t ACMDataSendQueue;

USBH_Usr_cb_TypeDef USR_Callbacks =
{
  USBH_USR_Init,
  USBH_USR_DeInit,
  USBH_USR_DeviceAttached,
  USBH_USR_ResetDevice,
  USBH_USR_DeviceDisconnected,
  USBH_USR_OverCurrentDetected,
  USBH_USR_DeviceSpeedDetected,
  USBH_USR_Device_DescAvailable,
  USBH_USR_DeviceAddressAssigned,
  USBH_USR_Configuration_DescAvailable,
  USBH_USR_Manufacturer_String,
  USBH_USR_Product_String,
  USBH_USR_SerialNum_String,
  USBH_USR_EnumerationDone,
  USBH_USR_UserInput,
  USBH_USR_Application,
  USBH_USR_DeviceNotSupported,
  USBH_USR_UnrecoveredError
};

static void CDC_OutputData (uint8_t *ptr);
static void CDC_SetInitialValue( void);

void USBH_USR_Init(void)
{
	USB_DEBUG("USB OTG FS MSC Host");
	USB_DEBUG("> USB Host library started.");
	USB_DEBUG("  USB Host Library v2.2.0");

}

void USBH_USR_DeviceAttached(void)
{
	USB_DEBUG("USBH_USR_DeviceAttached!");
}

void USBH_USR_DeviceDisconnected (void)
{
	USB_DEBUG("USBH_USR_DeviceDisconnected!");
}

void USBH_USR_ResetDevice(void)
{
	USB_DEBUG("USBH_USR_ResetDevice");
}

void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
	if(DeviceSpeed==HPRT0_PRTSPD_HIGH_SPEED)
	{
		USB_DEBUG("(HS)USB Device Found!");
 	}
	else if(DeviceSpeed==HPRT0_PRTSPD_FULL_SPEED)
	{
		USB_DEBUG("(FS)USB Device Found!");
	}
	else if(DeviceSpeed==HPRT0_PRTSPD_LOW_SPEED)
	{
		USB_DEBUG("(LS)USB Device Found!");
	}
	else
	{
		USB_DEBUG("Device Error");
	}
}

void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
	USBH_DevDesc_TypeDef DevDesc = *(USBH_DevDesc_TypeDef *)DeviceDesc;
	USB_DEBUG("bDescriptorType    : %02Xh" , (uint8_t)DevDesc.bDescriptorType);
	USB_DEBUG("VID                : %04Xh" , (uint16_t)DevDesc.idVendor);
	USB_DEBUG("PID                : %04Xh" , (uint16_t)DevDesc.idProduct);
	USB_DEBUG("bcdUSB             : %04Xh" , (uint16_t)DevDesc.bcdUSB);
	USB_DEBUG("bDeviceClass       : %04Xh" , (uint8_t)DevDesc.bDeviceClass);
	USB_DEBUG("bDeviceSubClass    : %04Xh" , (uint8_t)DevDesc.bDeviceSubClass);
	USB_DEBUG("bDeviceProtocol    : %04Xh" , (uint8_t)DevDesc.bDeviceProtocol);
	USB_DEBUG("bMaxPacketSize     : %04Xh" , (uint8_t)DevDesc.bMaxPacketSize);
	USB_DEBUG("iManufacturer      : %04Xh" , (uint8_t)DevDesc.iManufacturer);
	USB_DEBUG("bNumConfigurations : %04Xh" , (uint8_t)DevDesc.bNumConfigurations);
}

void USBH_USR_DeviceAddressAssigned(void)
{
	USB_DEBUG("USBH_USR_DeviceAddressAssigned!");
}

void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{

	USBH_InterfaceDesc_TypeDef *id;
	id = itfDesc;
	USB_DEBUG("USBH_USR_Configuration_DescAvailable ...");
	if((*id).bInterfaceClass==0x08)
	{
		USB_DEBUG("MSC Interface!");
	}else if((*id).bInterfaceClass==0x03)
	{
		USB_DEBUG("HID Interface!");
	}
	else if((*id).bInterfaceClass  == COMMUNICATION_DEVICE_CLASS_CODE)
	{
		USB_DEBUG("CDC Interface!");
	}
	else
	{
		USB_DEBUG("Unsupport Device");
		USB_DEBUG("bInterfaceClass is %04Xh",(*id).bInterfaceClass);
	}
}

void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
	USB_DEBUG("Manufacturer: %s",(char *)ManufacturerString);
}

void USBH_USR_Product_String(void *ProductString)
{
	USB_DEBUG("Product: %s",(char *)ProductString);
}

void USBH_USR_SerialNum_String(void *SerialNumString)
{
	USB_DEBUG("Serial Number: %s",(char *)SerialNumString);
}

void USBH_USR_EnumerationDone(void)
{
	USB_DEBUG("EnumerationDone!");
	UserCb.Receive = CDC_OutputData;
}

void USBH_USR_DeviceNotSupported(void)
{
	USB_DEBUG("DeviceNotSupported!");
}

USBH_USR_Status USBH_USR_UserInput(void)
{
	USB_DEBUG("USBH_USR_UserInput!");
	CDCUsrState = CDC_DEMO_IDLE;
	return USBH_USR_RESP_OK;
}

void USBH_USR_OverCurrentDetected (void)
{
	USB_DEBUG("USBH_USR_OverCurrentDetected");
}

void USBH_USR_DeInit(void)
{
	USB_DEBUG("USBH_USR_DeInit!");
}

void USBH_USR_UnrecoveredError (void)
{
	USB_DEBUG("USBH_USR_UnrecoveredError!!");
}
int USBH_USR_Application(void)
{
	if(CDCUsrState == CDC_DEMO_IDLE)
	{
		USB_DEBUG("USBH_USR_Application CDC_DEMO_IDLE!!!");
		CDCUsrState = CDC_DEMO_CONFIGURATION;
		CDC_StartReception(&USB_OTG_Core);
	}
	else if(CDCUsrState == CDC_DEMO_CONFIGURATION)
	{
		USB_DEBUG("USBH_USR_Application CDC_DEMO_CONFIGURATION!!!");
		CDC_SetInitialValue();
		CDCUsrState = CDC_DEMO_SEND;
	}
	else if(CDCUsrState == CDC_DEMO_SEND)
	{
    static uint8_t bufferData[HAL_ONCE_READ_LEN] = {0};
    uint8_t recvByte = 0;
    uint16_t bufferLen = 0;

    for (int cnt = 0; cnt < HAL_ONCE_READ_LEN; cnt++) {
    if(xQueueReceive(ACMDataSendQueue, &recvByte, 0)) {
        bufferData[bufferLen] = recvByte;
        bufferLen++;
      } else {
        break;
      }
    }
    if (bufferLen) CDC_SendData(bufferData, bufferLen);
	  CDCUsrState = CDC_DEMO_SEND;
	}

  return 0;
}

extern QueueHandle_t ACMDataRecvQueue;
static void CDC_OutputData(uint8_t *ptr)
{
  BaseType_t xHigherPriorityTaskWoken = 0;
  CDC_Xfer_TypeDef *cdc_Data = (CDC_Xfer_TypeDef *)ptr;
  #if 0
	USB_DEBUG("receive cdc data:");
	for (int i = 0; i < cdc_Data->DataLength; i++)
  {
    USB_DEBUG(" %02X", cdc_Data->pEmptyBuff[i]);
  }
  USB_DEBUG("");
  #endif
  for (int i = 0; i < cdc_Data->DataLength; i++)
  {
    xQueueSend(ACMDataRecvQueue, cdc_Data->pEmptyBuff + i, (TickType_t)0);
  }
}

static void CDC_SetInitialValue(void)
{
	/*Set the initial value*/
	CDC_SetLineCode.b.dwDTERate = 921600;
	CDC_SetLineCode.b.bDataBits = 8;
	CDC_SetLineCode.b.bParityType = 0;
	CDC_SetLineCode.b.bCharFormat = 0;
	USB_DEBUG("dwDTERate   : %d", CDC_SetLineCode.b.dwDTERate);
	USB_DEBUG("bDataBits   : %d", CDC_SetLineCode.b.bDataBits);
	USB_DEBUG("bParityType : %d", CDC_SetLineCode.b.bParityType);
	USB_DEBUG("bCharFormat : %d", CDC_SetLineCode.b.bCharFormat);
	CDC_ChangeStateToIssueSetConfig(&USB_OTG_Core, &USB_Host);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
