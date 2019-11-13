/**
 ********************************************************************
 * @file    osdk_protocol.c
 * @version V1.0.0
 * @date    2019/09/25
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
#include "osdk_protocol.h"

/* Private functions definition ---------------------------------------------*/

/* Private constants --------------------------------------------------------*/
const static T_ProtocolOps protocolOps[PROTOCOL_MAX_NUM] = {
    {PROTOCOL_SDK, OsdkProtocol_sdkInit, OsdkProtocol_sdkPack,
     OsdkProtocol_sdkParse, OsdkProtocol_sdkUnpack,
     OsdkProtocol_sdkGetLen, OsdkProtocol_setSdkKey},
    {PROTOCOL_V1, OsdkProtocol_v1Init, OsdkProtocol_v1Pack,
     OsdkProtocol_v1Parse, OsdkProtocol_v1Unpack,
     OsdkProtocol_v1GetLen, NULL}};
/* Private types ------------------------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkProtocol_getProtocolOps(E_ProtocolType type,
                                       T_ProtocolOps *ops) {
  int i;
  if(!ops) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL,
                   "OsdkProtocol_getProtocolOps param check failed");
    return OSDK_STAT_ERR_PARAM;
  }
  for (i = 0; i < PROTOCOL_MAX_NUM; ++i) {
    if (protocolOps[i].type == type) {
      *ops = protocolOps[i];
      return OSDK_STAT_OK;
    }
  }
  return OSDK_STAT_ERR;
}
/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
