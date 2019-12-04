/**
  ******************************************************************************
  * @file    usbh_conf_template
  * @author  MCD Application Team
  * @version V2.2.1
  * @date    17-March-2018
  * @brief   General USB Host library configuration
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
#ifndef __USBH_CONF__H__
#define __USBH_CONF__H__

/* Includes ------------------------------------------------------------------*/

/** @addtogroup USBH_OTG_DRIVER
  * @{
  */
  
/** @defgroup USBH_CONF
  * @brief usb otg low level driver configuration file
  * @{
  */ 

/** @defgroup USBH_CONF_Exported_Defines
  * @{
  */ 

#define USBH_MAX_NUM_ENDPOINTS                2
#define USBH_MAX_NUM_INTERFACES               2
#ifdef USE_USB_OTG_FS 
#define USBH_MSC_MPS_SIZE                 0x40
#else
#define USBH_MSC_MPS_SIZE                 0x200
#endif

/**
  * @}
  */ 


/** @defgroup USBH_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_CONF_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 


#endif //__USBH_CONF__H__


/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

