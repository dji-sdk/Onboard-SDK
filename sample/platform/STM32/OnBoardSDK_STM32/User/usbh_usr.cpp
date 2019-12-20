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
#include "usbh_usr.h"
#include "usbh_cdc_core.h"
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "osdk_hal.h"
#ifdef __cplusplus
}
#endif
#include "dji_log.hpp"

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
	DSTATUS("USB OTG FS MSC Host");
	DSTATUS("> USB Host library started.");
	DSTATUS("  USB Host Library v2.2.0");

}

void USBH_USR_DeviceAttached(void)
{
	DSTATUS("USBH_USR_DeviceAttached!");
}

void USBH_USR_DeviceDisconnected (void)
{
	DSTATUS("USBH_USR_DeviceDisconnected!");
}

void USBH_USR_ResetDevice(void)
{
	DSTATUS("USBH_USR_ResetDevice");
}

void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
	if(DeviceSpeed==HPRT0_PRTSPD_HIGH_SPEED)
	{
		DSTATUS("(HS)USB Device Found!");
 	}
	else if(DeviceSpeed==HPRT0_PRTSPD_FULL_SPEED)
	{
		DSTATUS("(FS)USB Device Found!");
	}
	else if(DeviceSpeed==HPRT0_PRTSPD_LOW_SPEED)
	{
		DSTATUS("(LS)USB Device Found!");
	}
	else
	{
		DSTATUS("Device Error");
	}
}

void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
	USBH_DevDesc_TypeDef DevDesc = *(USBH_DevDesc_TypeDef *)DeviceDesc;
	DSTATUS("bDescriptorType    : %02Xh" , (uint8_t)DevDesc.bDescriptorType);
	DSTATUS("VID                : %04Xh" , (uint16_t)DevDesc.idVendor);
	DSTATUS("PID                : %04Xh" , (uint16_t)DevDesc.idProduct);
	DSTATUS("bcdUSB             : %04Xh" , (uint16_t)DevDesc.bcdUSB);
	DSTATUS("bDeviceClass       : %04Xh" , (uint8_t)DevDesc.bDeviceClass);
	DSTATUS("bDeviceSubClass    : %04Xh" , (uint8_t)DevDesc.bDeviceSubClass);
	DSTATUS("bDeviceProtocol    : %04Xh" , (uint8_t)DevDesc.bDeviceProtocol);
	DSTATUS("bMaxPacketSize     : %04Xh" , (uint8_t)DevDesc.bMaxPacketSize);
	DSTATUS("iManufacturer      : %04Xh" , (uint8_t)DevDesc.iManufacturer);
	DSTATUS("bNumConfigurations : %04Xh" , (uint8_t)DevDesc.bNumConfigurations);
}

void USBH_USR_DeviceAddressAssigned(void)
{
	DSTATUS("USBH_USR_DeviceAddressAssigned!");
}

void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{

	USBH_InterfaceDesc_TypeDef *id;
	id = itfDesc;
	DSTATUS("USBH_USR_Configuration_DescAvailable ...");
	if((*id).bInterfaceClass==0x08)
	{
		DSTATUS("MSC Interface!");
	}else if((*id).bInterfaceClass==0x03)
	{
		DSTATUS("HID Interface!");
	}
	else if((*id).bInterfaceClass  == COMMUNICATION_DEVICE_CLASS_CODE)
	{
		DSTATUS("CDC Interface!");
	}
	else
	{
		DSTATUS("Unsupport Device");
		DSTATUS("bInterfaceClass is %04Xh",(*id).bInterfaceClass);
	}
}

void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
	DSTATUS("Manufacturer: %s",(char *)ManufacturerString);
}

void USBH_USR_Product_String(void *ProductString)
{
	DSTATUS("Product: %s",(char *)ProductString);
}

void USBH_USR_SerialNum_String(void *SerialNumString)
{
	DSTATUS("Serial Number: %s",(char *)SerialNumString);
}

void USBH_USR_EnumerationDone(void)
{
	DSTATUS("EnumerationDone!");
	UserCb.Receive = CDC_OutputData;
}

void USBH_USR_DeviceNotSupported(void)
{
	DSTATUS("DeviceNotSupported!");
}

USBH_USR_Status USBH_USR_UserInput(void)
{
	DSTATUS("USBH_USR_UserInput!");
	CDCUsrState = CDC_DEMO_IDLE;
	return USBH_USR_RESP_OK;
}

void USBH_USR_OverCurrentDetected (void)
{
	DSTATUS("USBH_USR_OverCurrentDetected");
}

void USBH_USR_DeInit(void)
{
	DSTATUS("USBH_USR_DeInit!");
}

void USBH_USR_UnrecoveredError (void)
{
	DSTATUS("USBH_USR_UnrecoveredError!!");
}
int USBH_USR_Application(void)
{
	if(CDCUsrState == CDC_DEMO_IDLE)
	{
		DSTATUS("USBH_USR_Application CDC_DEMO_IDLE!!!");
		CDCUsrState = CDC_DEMO_CONFIGURATION;
		CDC_StartReception(&USB_OTG_Core);
	}
	else if(CDCUsrState == CDC_DEMO_CONFIGURATION)
	{
		DSTATUS("USBH_USR_Application CDC_DEMO_CONFIGURATION!!!");
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
	DSTATUS("receive cdc data:");
	for (int i = 0; i < cdc_Data->DataLength; i++)
  {
    DSTATUS(" %02X", cdc_Data->pEmptyBuff[i]);
  }
  DSTATUS("");
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
	DSTATUS("dwDTERate   : %d", CDC_SetLineCode.b.dwDTERate);
  DSTATUS("bDataBits   : %d", CDC_SetLineCode.b.bDataBits);
  DSTATUS("bParityType : %d", CDC_SetLineCode.b.bParityType);
  DSTATUS("bCharFormat : %d", CDC_SetLineCode.b.bCharFormat);
	CDC_ChangeStateToIssueSetConfig(&USB_OTG_Core, &USB_Host);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
