/**
 ********************************************************************
 * @file    osdk_command_instance.c
 * @version V1.0.0
 * @date    2019/09/15
 * @brief
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "osdk_command_instance.h"


/* Private constants ---------------------------------------------------------*/
#ifdef STM32
#include "stm32f4xx.h"
/*! STM32F4 family includes a 64-Kbyte of CCM (core coupled memory) data RAM.
 *  Start Address : CCMDATARAM_BASE(0x10000000)
 *  Should be paid attention that only STM32F4XX support this defination */
static T_CmdHandle s_commandHandle __attribute__((at(CCMDATARAM_BASE)));
#else
static T_CmdHandle s_commandHandle;
#endif
/* Private types -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkCommand_InitInstance(void) {
  T_CmdInitConf conf = {0};
  memset(&s_commandHandle, 0, sizeof(s_commandHandle));
  return OsdkCommand_Init(&s_commandHandle, &conf);
}

T_CmdHandle *OsdkCommand_GetInstance(void) { return &s_commandHandle; }

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
