/**
  ******************************************************************************
  * @file    usbh_cdc_core.c
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   This file is the CDC Layer Handlers for USB Host CDC class.
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

/* Includes ------------------------------------------------------------------*/
#include "usbh_cdc_core.h"

/** @addtogroup USBH_LIB
* @{
*/

/** @addtogroup USBH_CLASS
* @{
*/

/** @addtogroup CDC_CLASS
* @{
*/

/** @defgroup CDC_CORE 
  * @brief    This file includes CDC Layer Handlers for USB Host CDC class.
* @{
*/ 

/** @defgroup CDC_CORE_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 


/** @defgroup CDC_CORE_Private_Defines
* @{
*/ 
#define CDC_BUFFER_SIZE                 1024
/**
* @}
*/ 


/** @defgroup CDC_CORE_Private_Macros
* @{
*/ 
/**
* @}
*/ 


/** @defgroup CDC_CORE_Private_Variables
* @{
*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4   
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN CDC_Machine_TypeDef   CDC_Machine __ALIGN_END ;

CDC_Requests                        CDC_ReqState;
CDC_Xfer_TypeDef                    CDC_TxParam;
CDC_Xfer_TypeDef                    CDC_RxParam;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4   
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t               TxBuf [CDC_BUFFER_SIZE] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma data_alignment=4   
#endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t               RxBuf [CDC_BUFFER_SIZE] __ALIGN_END ;

CDC_Usercb_TypeDef                  UserCb;
uint8_t                             RX_Enabled = 0;
/**
* @}
*/ 


/** @defgroup CDC_CORE_Private_FunctionPrototypes
* @{
*/ 
static void CDC_InitTxRxParam(void);

static void CDC_ReceiveData(CDC_Xfer_TypeDef *cdc_Data);

static void CDC_ProcessTransmission(USB_OTG_CORE_HANDLE *pdev, USBH_HOST  *phost);

static void CDC_ProcessReception(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);

static USBH_Status CDC_InterfaceInit  (USB_OTG_CORE_HANDLE *pdev , 
                                       void *phost);

void CDC_InterfaceDeInit  (USB_OTG_CORE_HANDLE *pdev , 
                                  void *phost);

static USBH_Status CDC_Handle(USB_OTG_CORE_HANDLE *pdev , 
                              void *phost);

static USBH_Status CDC_ClassRequest(USB_OTG_CORE_HANDLE *pdev , 
                                    void *phost);


USBH_Class_cb_TypeDef  CDC_cb = 
{
  CDC_InterfaceInit,
  CDC_InterfaceDeInit,
  CDC_ClassRequest,
  CDC_Handle
};
/**
* @}
*/ 


/** @defgroup CDC_CORE_Private_Functions
* @{
*/ 

/**
  * @brief  CDC_InterfaceInit 
  *         The function init the CDC class.
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval  USBH_Status :Response for USB CDC driver intialization
  */
static USBH_Status CDC_InterfaceInit ( USB_OTG_CORE_HANDLE *pdev, 
                                      void *phost)
{	
  USBH_HOST *pphost = phost;
  USBH_Status status = USBH_OK ;
  
  /* Communication Interface */
  for (int i=0; i<USBH_MAX_NUM_INTERFACES; i++)
  {
#ifdef OSDK_USB_DEBUG
    printf("bInterfaceClass: %04Xh,bInterfaceSubClass: %04Xh,bInterfaceProtocol: %04Xh\r\n",
           pphost->device_prop.Itf_Desc[i].bInterfaceClass,pphost->device_prop.Itf_Desc[i].bInterfaceSubClass,
           pphost->device_prop.Itf_Desc[i].bInterfaceProtocol
          );
#endif
  }
  
  /* Communication Interface */
  if((pphost->device_prop.Itf_Desc[4].bInterfaceClass  == COMMUNICATION_DEVICE_CLASS_CODE)&& \
    (pphost->device_prop.Itf_Desc[4].bInterfaceSubClass  == ABSTRACT_CONTROL_MODEL) && \
      (pphost->device_prop.Itf_Desc[4].bInterfaceProtocol == COMMON_AT_COMMAND))
  {
    /*Collect the notification endpoint address and length*/
    CDC_Machine.CDC_CommItf.ep_addr = pphost->device_prop.Ep_Desc[4][0].bEndpointAddress;
    CDC_Machine.CDC_CommItf.length  = pphost->device_prop.Ep_Desc[4][0].wMaxPacketSize;
    
    if(pphost->device_prop.Ep_Desc[4][0].bEndpointAddress & 0x80)
    {
      CDC_Machine.CDC_CommItf.notificationEp =\
        (pphost->device_prop.Ep_Desc[4][0].bEndpointAddress);
    }
    /*Allocate the length for host channel number in*/
    CDC_Machine.CDC_CommItf.hc_num_in = USBH_Alloc_Channel(pdev, 
                                                           CDC_Machine.CDC_CommItf.notificationEp );
    
    /* Open channel for IN endpoint */
    USBH_Open_Channel  (pdev,
                        CDC_Machine.CDC_CommItf.hc_num_in,
                        pphost->device_prop.address,
                        pphost->device_prop.speed,
                        EP_TYPE_INTR,
                        CDC_Machine.CDC_CommItf.length); 
  }
  else
  {
    pphost->usr_cb->DeviceNotSupported();   
  }
  
  
  /* Data Interface */
  if((pphost->device_prop.Itf_Desc[5].bInterfaceClass  == DATA_INTERFACE_CLASS_CODE)&& \
    (pphost->device_prop.Itf_Desc[5].bInterfaceSubClass  == RESERVED) && \
      (pphost->device_prop.Itf_Desc[5].bInterfaceProtocol == NO_CLASS_SPECIFIC_PROTOCOL_CODE))
  {
    /*Collect the class specific endpoint address and length*/
    CDC_Machine.CDC_DataItf.ep_addr = pphost->device_prop.Ep_Desc[5][0].bEndpointAddress;
    CDC_Machine.CDC_DataItf.length  = pphost->device_prop.Ep_Desc[5][0].wMaxPacketSize;
    
    if(pphost->device_prop.Ep_Desc[5][0].bEndpointAddress & 0x80)
    {      
      CDC_Machine.CDC_DataItf.cdcInEp = (pphost->device_prop.Ep_Desc[5][0].bEndpointAddress);
    }
    else
    {
      CDC_Machine.CDC_DataItf.cdcOutEp = (pphost->device_prop.Ep_Desc[5][0].bEndpointAddress);
    }
    
    if(pphost->device_prop.Ep_Desc[5][1].bEndpointAddress & 0x80)
    {
      CDC_Machine.CDC_DataItf.cdcInEp = (pphost->device_prop.Ep_Desc[5][1].bEndpointAddress);
    }
    else
    {
      CDC_Machine.CDC_DataItf.cdcOutEp = (pphost->device_prop.Ep_Desc[5][1].bEndpointAddress);
    }
    
    /*Allocate the length for host channel number out*/
    CDC_Machine.CDC_DataItf.hc_num_out = USBH_Alloc_Channel(pdev, 
                                                            CDC_Machine.CDC_DataItf.cdcOutEp);
    /*Allocate the length for host channel number in*/
    CDC_Machine.CDC_DataItf.hc_num_in = USBH_Alloc_Channel(pdev, 
                                                           CDC_Machine.CDC_DataItf.cdcInEp);  
    
    /* Open channel for OUT endpoint */
    USBH_Open_Channel  (pdev,
                        CDC_Machine.CDC_DataItf.hc_num_out,
                        pphost->device_prop.address,
                        pphost->device_prop.speed,
                        EP_TYPE_BULK,
                        CDC_Machine.CDC_DataItf.length);  
    /* Open channel for IN endpoint */
    USBH_Open_Channel  (pdev,
                        CDC_Machine.CDC_DataItf.hc_num_in,
                        pphost->device_prop.address,
                        pphost->device_prop.speed,
                        EP_TYPE_BULK,
                        CDC_Machine.CDC_DataItf.length);
    
    /*Initilise the Tx/Rx Params*/
    CDC_InitTxRxParam();
    
    
    /*Initialize the class specific request with "GET_LINE_CODING"*/
    CDC_ReqState = CDC_GET_LINE_CODING_RQUEST ;
  }
  else
  {
    pphost->usr_cb->DeviceNotSupported();   
  }  
  return status;
  
}



/**
* @brief  CDC_InterfaceDeInit 
*         The function DeInit the Host Channels used for the CDC class.
* @param  pdev: Selected device
* @param  hdev: Selected device property
* @retval None
*/
void CDC_InterfaceDeInit ( USB_OTG_CORE_HANDLE *pdev,
                          void *phost)
{
  if ( CDC_Machine.CDC_CommItf.hc_num_in)
  {
    USB_OTG_HC_Halt(pdev, CDC_Machine.CDC_CommItf.hc_num_in);
    USBH_Free_Channel  (pdev,CDC_Machine.CDC_CommItf.hc_num_in);
    CDC_Machine.CDC_CommItf.hc_num_in = 0;     /* Reset the Channel as Free */
  }
  
  if ( CDC_Machine.CDC_DataItf.hc_num_out)
  {
    USB_OTG_HC_Halt(pdev, CDC_Machine.CDC_DataItf.hc_num_out);
    USBH_Free_Channel  (pdev,CDC_Machine.CDC_DataItf.hc_num_out);
    CDC_Machine.CDC_DataItf.hc_num_out = 0;     /* Reset the Channel as Free */
  }
  
  if ( CDC_Machine.CDC_DataItf.hc_num_in)
  {
    USB_OTG_HC_Halt(pdev, CDC_Machine.CDC_DataItf.hc_num_in);
    USBH_Free_Channel  (pdev,CDC_Machine.CDC_DataItf.hc_num_in);
    CDC_Machine.CDC_DataItf.hc_num_in = 0;     /* Reset the Channel as Free */
  } 
//  return USBH_OK;
}

/**
  * @brief  CDC_ClassRequest 
  *         The function is responsible for handling CDC Class requests
  *         for CDC class.
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval  USBH_Status :Response for USB Set Protocol request
  */
static USBH_Status CDC_ClassRequest(USB_OTG_CORE_HANDLE *pdev , 
                                    void *phost)
{   
  USBH_HOST *pphost = phost;
  
  USBH_Status status         = USBH_BUSY;
  USBH_Status ClassReqStatus = USBH_BUSY;
  
  switch(CDC_ReqState) 
  {
    
  case CDC_GET_LINE_CODING_RQUEST: 
    /*Issue the get line coding request*/
    ClassReqStatus = CDC_GETLineCoding(pdev, phost);
    if(ClassReqStatus == USBH_OK)
    {          /*Change the state */
      CDC_ReqState = CDC_SET_CONTROL_LINE_STATE_REQUEST;
    }
    break;
    
  case CDC_SET_LINE_CODING_RQUEST: 
    
    /*Issue the set line coding request*/
    ClassReqStatus = CDC_SETLineCoding(pdev, phost);
    if( ClassReqStatus == USBH_OK )
    {
      /*Change the state */
      CDC_ReqState = CDC_GET_LINE_CODING_RQUEST ;
    }
    if(ClassReqStatus == USBH_NOT_SUPPORTED )
    {
      /* a Clear Feature should be issued here */
      CDC_ReqState = CDC_ERROR_STATE;
    }
    break;
    
  case CDC_SET_CONTROL_LINE_STATE_REQUEST:
    /*Issue the set control line coding */
    ClassReqStatus = CDC_SETControlLineState(pdev, phost);
    if( ClassReqStatus == USBH_OK )
    {
      /*Change the state */
      CDC_ReqState = CDC_SET_CONTROL_LINE_STATE_REQUEST;
      /*Also set the state of receive CDCRxParam to IDLE*/
      CDC_RxParam.CDCState = CDC_IDLE; 
      
      status = USBH_OK; /*This return from class specific routinues request*/
    }
    break;
    
  case CDC_ERROR_STATE:
    
    ClassReqStatus = USBH_ClrFeature(pdev,
                                     phost,
                                     0x00,
                                     pphost->Control.hc_num_out);
    
    if(ClassReqStatus == USBH_OK )
    {        
      /*Change the state to waiting*/
      CDC_ReqState = CDC_GET_LINE_CODING_RQUEST ;
    }
    break;      
  }
  
  return status; 
}


/**
  * @brief  CDC_Handle 
  *         The function is for managing state machine for CDC data transfers 
  * @param  pdev: Selected device
  * @param  hdev: Selected device property
  * @retval USBH_Status
  */
static USBH_Status CDC_Handle(USB_OTG_CORE_HANDLE *pdev , 
                              void   *phost)
{
  USBH_Status status = USBH_OK;
  USBH_HOST *pphost = phost;
  
  /* Call Application process */
  pphost->usr_cb->UserApplication();  
  
  /*Handle the transmission */
  CDC_ProcessTransmission(pdev, pphost);
  
  /*Always send in packet to device*/    
  CDC_ProcessReception(pdev, pphost);
  
  return status;
}


/**
  * @brief  The function is responsible for sending data to the device
  * @param  pdev: Selected device
  * @retval None
  */
void CDC_ProcessTransmission(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  static uint32_t len ;
  URB_STATE URB_StatusTx = URB_IDLE;
  
  URB_StatusTx =   HCD_GetURB_State(pdev , CDC_Machine.CDC_DataItf.hc_num_out);
  
  switch(CDC_TxParam.CDCState)
  {
  case CDC_IDLE:
    break;
    
  case CDC_SEND_DATA:
    
    if(( URB_StatusTx == URB_DONE ) || (URB_StatusTx == URB_IDLE))
    {
      /*Check the data length is more then the CDC_Machine.CDC_DataItf.CDC_DataItf.length */
      if(CDC_TxParam.DataLength > CDC_Machine.CDC_DataItf.length)
      {
        
        len = CDC_Machine.CDC_DataItf.length ;
        /*Send the data */
        USBH_BulkSendData (pdev,
                           CDC_TxParam.pRxTxBuff, 
                           len , 
                           CDC_Machine.CDC_DataItf.hc_num_out);    
      }
      else
      {
        len = CDC_TxParam.DataLength ;
        /*Send the remaining data */
        USBH_BulkSendData (pdev,
                           CDC_TxParam.pRxTxBuff, 
                           len, 
                           CDC_Machine.CDC_DataItf.hc_num_out);
        
      }
      CDC_TxParam.CDCState = CDC_DATA_SENT;
      
    }
    
    break;
    
  case CDC_DATA_SENT:
    /*Check the status done for transmssion*/
    if(URB_StatusTx == URB_DONE )
    {         
      /*Point to next chunc of data*/
      CDC_TxParam.pRxTxBuff += len ;
      
      /*decrease the data length*/
      CDC_TxParam.DataLength -= len;    
      
      if(CDC_TxParam.DataLength == 0)
      {
        CDC_TxParam.CDCState = CDC_IDLE;
      }
      else
      {
        CDC_TxParam.CDCState = CDC_SEND_DATA; 
      }
    }
    else if( URB_StatusTx == URB_NOTREADY )
    {
      /*Send the same data */
      USBH_BulkSendData (pdev,
                         (CDC_TxParam.pRxTxBuff), 
                         len, 
                         CDC_Machine.CDC_DataItf.hc_num_out);
    }
    
    break;

  case CDC_READ_DATA:
	  break;

  case CDC_BUSY:
	  break;

  case CDC_GET_DATA:
	  break;

  case CDC_POLL:
	  break;

  case CDC_CTRL_STATE:
	  break;
    
  }
}
/**
  * @brief  This function responsible for reception of data from the device
  * @param  pdev: Selected device
  * @retval None
  */
static void CDC_ProcessReception(USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
  
  if(RX_Enabled == 1)
  {
    URB_STATE URB_StatusRx =   HCD_GetURB_State(pdev , CDC_Machine.CDC_DataItf.hc_num_in);  
    
    switch(CDC_RxParam.CDCState)
    {
      
    case CDC_IDLE:
      
      
      /*check the received length lesser then the remaining space available in the 
      buffer */
      if(CDC_RxParam.DataLength < (CDC_RxParam.BufferLen - CDC_Machine.CDC_DataItf.length))
      {
        /*Receive the data */
        USBH_BulkReceiveData(pdev,
                             CDC_RxParam.pFillBuff,
                             CDC_Machine.CDC_DataItf.length, 
                             CDC_Machine.CDC_DataItf.hc_num_in);
        
        /*change the cdc state to USBH_CDC_GET_DATA*/
        CDC_RxParam.CDCState = CDC_GET_DATA;
      }
      break;
      
    case CDC_GET_DATA:
      /*Check the last state of the device is URB_DONE */
      if(URB_StatusRx == URB_DONE)
      {
        /* Move the pointer as well as datalength */
        CDC_RxParam.DataLength += pdev->host.hc[CDC_Machine.CDC_DataItf.hc_num_in].xfer_count ;
        CDC_RxParam.pFillBuff += pdev->host.hc[CDC_Machine.CDC_DataItf.hc_num_in].xfer_count ;
        
        
        /* Process the recived data */
        CDC_ReceiveData(&CDC_RxParam);
        
        /*change the state od the CDC state*/
        CDC_RxParam.CDCState = CDC_IDLE;
        
      }
      break;

    case CDC_READ_DATA:

    	break;

    case CDC_BUSY:

    	break;

    case CDC_SEND_DATA:

    	break;

    case CDC_DATA_SENT:

    	break;

    case CDC_POLL:

    	break;

    case CDC_CTRL_STATE:

    	break;
    }
  }
}

/**
  * @brief  Initialize the transmit and receive buffer and its parameter
  * @param  None
  * @retval None
  */
static void CDC_InitTxRxParam(void)
{
  /*Initialize the Transmit buffer and its parameter*/
  CDC_TxParam.CDCState = CDC_IDLE;
  CDC_TxParam.DataLength = 0;
  CDC_TxParam.pRxTxBuff = TxBuf;
  
  /*Initialize the Receive buffer and its parameter*/
  CDC_RxParam.CDCState = CDC_IDLE;
  CDC_RxParam.DataLength = 0;
  CDC_RxParam.pFillBuff = RxBuf;  
  CDC_RxParam.pEmptyBuff = RxBuf;
  CDC_RxParam.BufferLen = sizeof(RxBuf);      
}

/**
  * @brief  This is a call back function from cdc core layer to redirect the 
  *         received data on the user out put system
  * @param  cdc_Data: type of USBH_CDCXfer_TypeDef
  * @retval None
  */
static void CDC_ReceiveData(CDC_Xfer_TypeDef *cdc_Data)
{
  uint8_t *ptr; 
  
  if(cdc_Data->pEmptyBuff < cdc_Data->pFillBuff)
  {
    ptr = cdc_Data->pFillBuff;
    *ptr = 0x00;
    
    /* redirect the received data on the user out put system */
    UserCb.Receive((uint8_t *)cdc_Data);
    
    cdc_Data->pFillBuff  = cdc_Data->pEmptyBuff ; 
    cdc_Data->DataLength = 0;    /*Reset the data length to zero*/
  }
}

/**
  * @brief  This function send data to the device.
  * @param  fileName : name of the file 
  * @retval the filestate will be returned 
  * FS_SUCCESS : returned to the parent function when the file length become to zero
  */
void  CDC_SendData(uint8_t *data, uint16_t length)
{
  if(CDC_TxParam.CDCState == CDC_IDLE)
  {
    CDC_TxParam.pRxTxBuff = data; 
    CDC_TxParam.DataLength = length;
    CDC_TxParam.CDCState = CDC_SEND_DATA;  
  }    
}

/**
  * @brief  This function send data to the device.
  * @param  fileName : name of the file 
  * @retval the filestate will be returned 
  * FS_SUCCESS : returned to the parent function when the file length become to zero
  */
void  CDC_StartReception( USB_OTG_CORE_HANDLE *pdev)
{
  RX_Enabled = 1;
}

/**
  * @brief  This function send data to the device.
  * @param  fileName : name of the file 
  * @retval the filestate will be returned 
  * FS_SUCCESS : returned to the parent function when the file length become to zero
  */
void  CDC_StopReception( USB_OTG_CORE_HANDLE *pdev)
{
  RX_Enabled = 0; 
  USB_OTG_HC_Halt(pdev, CDC_Machine.CDC_DataItf.hc_num_in);
  USBH_Free_Channel  (pdev,CDC_Machine.CDC_DataItf.hc_num_in);
}

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


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
