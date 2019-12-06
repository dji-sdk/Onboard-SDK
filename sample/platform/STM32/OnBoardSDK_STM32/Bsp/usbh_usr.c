/* Includes ------------------------------------------------------------------ */
#include "usbh_usr.h"
#include "usbh_cdc_core.h"
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "osdk_hal.h"

vu8 bDeviceState = 0;

extern USB_OTG_CORE_HANDLE USB_OTG_Core;
extern USBH_HOST  USB_Host;
extern CDC_LineCodingTypeDef         CDC_GetLineCode;
extern CDC_LineCodingTypeDef         CDC_SetLineCode;
extern CDC_Usercb_TypeDef            UserCb;

uint8_t                              prev_select = 0;
uint8_t                              enable_display_received_data = 0;

CDC_DEMO_SETTING_StateMachine        cdc_settings_state;
CDC_Demo_State 						           cdc_usr_state;
u8 USB_FIRST_PLUGIN_FLAG = 0;

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
	printf("USB OTG FS MSC Host\r\n");
	printf("> USB Host library started.\r\n");
	printf("  USB Host Library v2.2.0\r\n\r\n");

}

void USBH_USR_DeviceAttached(void)
{
	printf("USBH_USR_DeviceAttached!\r\n");
}

void USBH_USR_DeviceDisconnected (void)
{
	printf("USBH_USR_DeviceDisconnected!\r\n");
	bDeviceState=0;
}

void USBH_USR_ResetDevice(void)
{
	printf("USBH_USR_ResetDevice\r\n");
}
//???????
//DeviceSpeed:????(0,1,2 / ??)
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
	if(DeviceSpeed==HPRT0_PRTSPD_HIGH_SPEED)
	{
		printf("(HS)USB Device Found!\r\n");
 	}
	else if(DeviceSpeed==HPRT0_PRTSPD_FULL_SPEED)
	{
		printf("(FS)USB Device Found!\r\n");
	}
	else if(DeviceSpeed==HPRT0_PRTSPD_LOW_SPEED)
	{
		printf("(LS)USB Device Found!\r\n");
	}
	else
	{
		printf("Device Error\r\n");
	}
}

void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{
	USBH_DevDesc_TypeDef *hs;
	hs=DeviceDesc;
	printf("bDescriptorType: %02Xh\r\n" , (uint8_t)(*hs).bDescriptorType);
	printf("VID: %04Xh\r\n" , (uint16_t)(*hs).idVendor);
	printf("PID: %04Xh\r\n" , (uint16_t)(*hs).idProduct);
	printf("bcdUSB: %04Xh\r\n" , (uint16_t)(*hs).bcdUSB);
	printf("bDeviceClass: %04Xh\r\n" , (uint8_t)(*hs).bDeviceClass);
	printf("bDeviceSubClass: %04Xh\r\n" , (uint8_t)(*hs).bDeviceSubClass);
	printf("bDeviceProtocol: %04Xh\r\n" , (uint8_t)(*hs).bDeviceProtocol);
	printf("bMaxPacketSize: %04Xh\r\n" , (uint8_t)(*hs).bMaxPacketSize);
	printf("iManufacturer: %04Xh\r\n" , (uint8_t)(*hs).iManufacturer);
	printf("bNumConfigurations: %04Xh\r\n" , (uint8_t)(*hs).bNumConfigurations);
}

void USBH_USR_DeviceAddressAssigned(void)
{
	printf("USBH_USR_DeviceAddressAssigned!\r\n");
}

void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{

	USBH_InterfaceDesc_TypeDef *id;
	id = itfDesc;
	printf("USBH_USR_Configuration_DescAvailable ...\r\n");
	if((*id).bInterfaceClass==0x08)
	{
		printf("MSC Interface!\r\n");
	}else if((*id).bInterfaceClass==0x03)
	{
		printf("HID Interface!\r\n");
	}
	else if((*id).bInterfaceClass  == COMMUNICATION_DEVICE_CLASS_CODE)
	{
		printf("CDC Interface!\r\n");
	}
	else
	{
		printf("Unsupport Device\r\n");
		printf("bInterfaceClass is %04Xh\r\n",(*id).bInterfaceClass);
	}
}

void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
	printf("Manufacturer: %s\r\n",(char *)ManufacturerString);
}

void USBH_USR_Product_String(void *ProductString)
{
	printf("Product: %s\r\n",(char *)ProductString);
}

void USBH_USR_SerialNum_String(void *SerialNumString)
{
	printf("Serial Number: %s\r\n",(char *)SerialNumString);
}

void USBH_USR_EnumerationDone(void)
{
	printf("EnumerationDone!\r\n\r\n");
	UserCb.Receive = CDC_OutputData;
}

void USBH_USR_DeviceNotSupported(void)
{
	printf("DeviceNotSupported!\r\n\r\n");
}

USBH_USR_Status USBH_USR_UserInput(void)
{
	printf("USBH_USR_UserInput!\r\n");
	bDeviceState=1;
	cdc_usr_state = CDC_DEMO_IDLE;
	return USBH_USR_RESP_OK;
}

void USBH_USR_OverCurrentDetected (void)
{
	printf("USBH_USR_OverCurrentDetected");
}

void USBH_USR_DeInit(void)
{
	printf("USBH_USR_DeInit!\r\n");
}

void USBH_USR_UnrecoveredError (void)
{
	printf("USBH_USR_UnrecoveredError!!\r\n\r\n");
}
int USBH_USR_Application(void)
{
	if(cdc_usr_state == CDC_DEMO_IDLE)
	{
		printf("USBH_USR_Application CDC_DEMO_IDLE!!!\r\n");
		cdc_usr_state = CDC_DEMO_CONFIGURATION;
		CDC_StartReception(&USB_OTG_Core);
	}
	else if(cdc_usr_state == CDC_DEMO_CONFIGURATION)
	{
		printf("USBH_USR_Application CDC_DEMO_CONFIGURATION!!!\r\n");
		CDC_SetInitialValue();
		cdc_usr_state = CDC_DEMO_SEND;
	}
	else if(cdc_usr_state == CDC_DEMO_SEND)
	{
#if 0
    uint16_t i = 0;
    if (i++ >= 1000) {
      printf("------>Send ping FC test\n");
      static uint8_t pingArray[] = {0x55, 0x0E, 0x04, 0x66, 0x0A, 0x03, 0x13, 0x27, 0x40, 0x00, 0x01, 0xFF, 0xAE, 0xC1};
      CDC_SendData(pingArray, sizeof(pingArray));
      i = 0;
    }
#endif
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
	  cdc_usr_state = CDC_DEMO_SEND;
	}

  return 0;
}

extern QueueHandle_t ACMDataRecvQueue;
static void CDC_OutputData(uint8_t *ptr)
{
  BaseType_t xHigherPriorityTaskWoken = 0;
  CDC_Xfer_TypeDef *cdc_Data = (CDC_Xfer_TypeDef *)ptr;
  #if 0
	printf("receive cdc data:\r\n");
	for (int i = 0; i < cdc_Data->DataLength; i++)
  {
    printf(" %02X", cdc_Data->pEmptyBuff[i]);
  }
  printf("\r\n");
  #endif
  for (int i = 0; i < cdc_Data->DataLength; i++)
  {
    xQueueSend(ACMDataRecvQueue, cdc_Data->pEmptyBuff + i, (TickType_t)0);
  }
}

/**
* @brief  Set the cdc demo intial values
* @param  None
* @retval None
*/
static void CDC_SetInitialValue( void)
{
	/*Set the initial value*/
	CDC_SetLineCode.b.dwDTERate = 115200;
	CDC_SetLineCode.b.bDataBits = 8;
	CDC_SetLineCode.b.bParityType = 0;
	CDC_SetLineCode.b.bCharFormat = 0;
	printf("dwDTERate is %d bDataBits is %d bParityType is %d bCharFormat is %d\n",
		CDC_SetLineCode.b.dwDTERate,CDC_SetLineCode.b.bDataBits,CDC_SetLineCode.b.bParityType
		,CDC_SetLineCode.b.bCharFormat);
	CDC_ChangeStateToIssueSetConfig(&USB_OTG_Core, &USB_Host);

  /*Initialize baud rate index accordingtly */
  switch(CDC_SetLineCode.b.dwDTERate)
  {
  case 2400 :
    cdc_settings_state.settings.BaudRateIdx = 0;
    break;
  case 4800 :
    cdc_settings_state.settings.BaudRateIdx = 1;
    break;

  case 9600 :
    cdc_settings_state.settings.BaudRateIdx = 2;
    break;

  case 19200 :
    cdc_settings_state.settings.BaudRateIdx = 3;
    break;

  case 38400 :
    cdc_settings_state.settings.BaudRateIdx = 4;
    break;

  case 57600 :
    cdc_settings_state.settings.BaudRateIdx = 5;
    break;

  case 115200 :
    cdc_settings_state.settings.BaudRateIdx = 6;
    break;

  case 230400 :
    cdc_settings_state.settings.BaudRateIdx = 7;
    break;

  case 460800 :
    cdc_settings_state.settings.BaudRateIdx = 8;
    break;

  case 921600 :
    cdc_settings_state.settings.BaudRateIdx = 9;
    break;
  default:
    break;
  }
  /*Initialize data bits index accordingtly */
  switch(CDC_SetLineCode.b.bDataBits)
  {
  case 5 :
    cdc_settings_state.settings.DataBitsIdx = 0;
    break;
  case 6 :
    cdc_settings_state.settings.DataBitsIdx = 1;
    break;

  case 7 :
    cdc_settings_state.settings.DataBitsIdx = 2;
    break;

  case 8 :
    cdc_settings_state.settings.DataBitsIdx = 3;
    break;
  default:
    break;
  }


  /*Initialize stop bits index accordingtly */
  switch(CDC_SetLineCode.b.bCharFormat)
  {
  case 1 :
    cdc_settings_state.settings.StopBitsIdx = 0;
    break;
  case 2 :
    cdc_settings_state.settings.StopBitsIdx = 1;
    break;
  default:
    break;
  }

  /*Initialize parity index accordingtly */
  switch(CDC_SetLineCode.b.bParityType)
  {
  case 0 :
    cdc_settings_state.settings.ParityIdx = 0;
    break;
  case 1 :
    cdc_settings_state.settings.ParityIdx = 1;
    break;

  case 2 :
    cdc_settings_state.settings.ParityIdx = 2;
    break;

  default:
    break;
  }

}
//////////////////////////////////////////////////////////////////////////////////////////
//??????,?ALIENTEK??,???USB??

//USB????????,??USB?????????
//phost:USB_HOST?????
//???:0,????
//       1,???,????????USB??.
u8 USBH_Check_EnumeDead(USBH_HOST *phost)
{
	static u16 errcnt=0;
	//????,??????,???USB???.
	if(phost->gState==HOST_CTRL_XFER&&(phost->EnumState==ENUM_IDLE||phost->EnumState==ENUM_GET_FULL_DEV_DESC))
	{
		errcnt++;
		if(errcnt>2000)//???
		{
			errcnt=0;
			RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS,ENABLE);//USB OTG FS ??
			USB_OTG_BSP_mDelay(5);
			RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS,DISABLE);	//????
			return 1;
		}
	}else errcnt=0;
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
//USB????????

//?????
void USR_MOUSE_Init	(void)
{
 	printf("USR_MOUSE_Init\n");
	USB_FIRST_PLUGIN_FLAG=1;//???????
}
//?????
void  USR_KEYBRD_Init(void)
{
 	printf("USR_KEYBRD_Init\n");
	USB_FIRST_PLUGIN_FLAG=1;//???????
}

















/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
