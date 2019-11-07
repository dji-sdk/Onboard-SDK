/**
 ********************************************************************
 * @file    osdk_protocol.h
 * @version V1.0.0
 * @date    2019/09/25
 * @brief   This is the header file for "osdk_protocol.c", defining the
 *structure and
 * (exported) function prototypes.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OSDK_PROTOCOL_H
#define OSDK_PROTOCOL_H

/* Includes ------------------------------------------------------------------*/

#include "osdk_protocol_sdk.h"
#include "osdk_protocol_v1.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define PROTOCOL_MAX_NUM 2

/* Exported types ------------------------------------------------------------*/

typedef E_OsdkStat (*Protocol_Init)(void **pProtocolExtData);

typedef E_OsdkStat (*Protocol_Pack)(void *protocolExtData, uint8_t *pFrame,
                                    uint16_t *len, const T_CmdInfo *pInfo,
                                    const uint8_t *cmdData);

typedef E_OsdkStat (*Protocol_Parse)(T_CmdParse *protParse, uint8_t byte,
                                     uint8_t **pParseFrame, uint16_t *parseLen);

typedef E_OsdkStat (*Protocol_Unpack)(void *protocolExtData, uint8_t *pFrame,
                                      T_CmdInfo *pInfo, uint8_t *cmdData);

typedef E_OsdkStat (*Protocol_GetFrameLen)(char *buffer, uint32_t *length);

typedef struct {
  E_ProtocolType type;
  Protocol_Init Init;
  Protocol_Pack Pack;
  Protocol_Parse Parse;
  Protocol_Unpack Unpack;
  Protocol_GetFrameLen GetFrameLen;
} T_ProtocolOps;

/* Exported functions --------------------------------------------------------*/
E_OsdkStat OsdkProtocol_getProtocolOps(E_ProtocolType type, T_ProtocolOps *ops);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_PROTOCOL_H
        /************************ (C) COPYRIGHT DJI Innovations *******END OF
         * FILE******/
