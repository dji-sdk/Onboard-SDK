/**
 ********************************************************************
 * @file    osdk_protocol_usbmc.c
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
#include <string.h>
#include "osdk_protocol_usbmc.h"

/* Private functions definition ---------------------------------------------*/
static E_OsdkStat OsdkProtocol_usbmcGetCmdFromChnId(int channelId, uint8_t *cmdSet, uint8_t *cmdId);
static E_OsdkStat OsdkProtocol_usbmcGetChnIdFromCmd(int *channelId, uint8_t cmdSet, uint8_t cmdId);

/* Private constants --------------------------------------------------------*/
#define USBMC_OVFL_MAGIC           (0x4F56464C)
#define USBMC_NORMAL_MAGIC         (0x55055055)

#define USB_BIG_DATA_MAX_BUF_SIZE (1024*512)

/* Private types ------------------------------------------------------------*/
#pragma pack(1)

typedef struct {
  uint32_t magic;
  int channelId;
  int headerLen;
  int dataLen;
  int seqNum;
} T_UsbmcProtocolHeader;

typedef struct {
  int channelId;
  uint8_t cmdSet;
  uint8_t cmdId;
} T_UsbmcMapKey;

#pragma pack()

const static T_UsbmcMapKey mapTable[] = {{1,  0x24, 0x13},
                                         {81, 0x65, 0x54},
                                         {82, 0x65, 0x55},
                                         {85, 0x65, 0x56},
                                         {90, 0x65, 0x57}};

/* Exported functions definition ---------------------------------------------*/

E_OsdkStat OsdkProtocol_usbmcInit(void **pProtocolExtData) {
  if(!pProtocolExtData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_usbmcInit param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  *pProtocolExtData =
      OsdkOsal_Malloc(sizeof(T_recvBufferCtx));
  if (*pProtocolExtData == NULL) {
    return OSDK_STAT_ERR_ALLOC;
  }
  T_recvBufferCtx *bufferCtx = *pProtocolExtData;
  bufferCtx->buffer = OsdkOsal_Malloc(USB_BIG_DATA_MAX_BUF_SIZE);
  if (bufferCtx->buffer == NULL) {
    OsdkOsal_Free(*pProtocolExtData);
    return OSDK_STAT_ERR_ALLOC;
  }
  memset(bufferCtx->buffer, 0, USB_BIG_DATA_MAX_BUF_SIZE);
  bufferCtx->bufferLen = 0;
  bufferCtx->useIdx = 0;
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_usbmcDeinit(void *pProtocolExtData) {
  if(!pProtocolExtData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_usbmcInit param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  T_recvBufferCtx *bufferCtx = (T_recvBufferCtx *)pProtocolExtData;
  OsdkOsal_Free(bufferCtx->buffer);
  OsdkOsal_Free(pProtocolExtData);
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_usbmcPack(void *protocolExtData, uint8_t *pFrame,
                                uint32_t *len, const T_CmdInfo *pInfo,
                                const uint8_t *cmdData) {
  E_OsdkStat osdkStat = OSDK_STAT_OK;
  T_UsbmcProtocolHeader *pHeader = (T_UsbmcProtocolHeader *)pFrame;
  pHeader->magic = 0;
  pHeader->headerLen = sizeof(T_UsbmcProtocolHeader);
  pHeader->seqNum = pInfo->seqNum;
  pHeader->dataLen = pInfo->dataLen;
	osdkStat = OsdkProtocol_usbmcGetChnIdFromCmd(&pHeader->channelId, pInfo->cmdSet, pInfo->cmdId);
  if(osdkStat != OSDK_STAT_OK) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_usbmcPack failed: unknown cmd set %d, cmd id %d", pInfo->cmdSet, pInfo->cmdId);
    return osdkStat;
  }
  memcpy(pFrame + sizeof(T_UsbmcProtocolHeader), cmdData, pInfo->dataLen);
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_usbmcGetLen(char *buffer, uint32_t *length) {
  T_UsbmcProtocolHeader *pHeader = (T_UsbmcProtocolHeader *)buffer;
  *length = pHeader->headerLen + pHeader->dataLen;
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_usbmcParse(T_CmdParse *protParse, uint8_t byte,
                                 uint8_t **pParseFrame, uint32_t *parseLen) {
  if(!protParse || !pParseFrame || !parseLen) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_v1Parse param check failed");
    return OSDK_STAT_ERR_PARAM;
  }
  if(protParse->parseIndex == 0 && byte != 0x55) {
    return OSDK_STAT_NOT_READY;
  }
  if(protParse->parseIndex == 1 && byte != 0x50) {
    protParse->parseIndex == 0;
    return OSDK_STAT_NOT_READY;
  }
  protParse->parseBuff[protParse->parseIndex] = byte;
  protParse->parseIndex++;

  if (protParse->parseIndex < sizeof(T_UsbmcProtocolHeader)) {
    return OSDK_STAT_NOT_READY;
  }

  T_UsbmcProtocolHeader *header = 
                             (T_UsbmcProtocolHeader *)protParse->parseBuff;
  if(header->magic != USBMC_NORMAL_MAGIC) {
    protParse->parseIndex = 0;
    return OSDK_STAT_NOT_READY;
  }

  if(protParse->parseIndex == header->headerLen + header->dataLen) {
    *pParseFrame = protParse->parseBuff;
    *parseLen = protParse->parseIndex;
    protParse->parseIndex = 0;
    return OSDK_STAT_OK;
  }

  return OSDK_STAT_NOT_READY;
}

E_OsdkStat OsdkProtocol_usbmcUnpack(void *protocolExtData, uint8_t *pFrame,
                                  T_CmdInfo *pInfo, uint8_t *cmdData) {
  E_OsdkStat osdkStat = OSDK_STAT_OK;
  if(!pFrame || !pInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_usbmcUnpack param check failed");
    return OSDK_STAT_ERR_PARAM;
  }
  T_UsbmcProtocolHeader *pHeader = (T_UsbmcProtocolHeader *)pFrame;
  pInfo->protoType = PROTOCOL_USBMC;
  pInfo->seqNum = pHeader->seqNum;
  pInfo->dataLen = pHeader->dataLen;
  pInfo->sender = pHeader->channelId;
  pInfo->receiver = 0;
  if(pHeader->magic == USBMC_OVFL_MAGIC) {
    pInfo->cmdSet = 0;
    pInfo->cmdId = 0;
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_usbmcUnpack() recv overflow data, channel id: %d", pHeader->channelId);
  } else {
    osdkStat = OsdkProtocol_usbmcGetCmdFromChnId(pHeader->channelId, &pInfo->cmdSet, &pInfo->cmdId);
    if(osdkStat != OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_usbmcUnpack failed: unknown channel id %d", pHeader->channelId);
      return osdkStat;
    }
  }
  memcpy(cmdData, pFrame + pHeader->headerLen, pHeader->dataLen);
  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/
static E_OsdkStat OsdkProtocol_usbmcGetCmdFromChnId(int channelId, uint8_t *cmdSet, uint8_t *cmdId) {
  int num = sizeof(mapTable)/sizeof(T_UsbmcMapKey);
  int i;
  for(i = 0; i < num; i++) {
    if(mapTable[i].channelId == channelId) {
      *cmdSet = mapTable[i].cmdSet;
      *cmdId = mapTable[i].cmdId;
      break;
    }
  }
  if(i == num) {
    return OSDK_STAT_ERR;
  }
  return OSDK_STAT_OK;
}

static E_OsdkStat OsdkProtocol_usbmcGetChnIdFromCmd(int *channelId, uint8_t cmdSet, uint8_t cmdId) {
  int num = sizeof(mapTable)/sizeof(T_UsbmcMapKey);
  int i;
  for(i = 0; i < num; i++) {
    if(mapTable[i].cmdSet == cmdSet && mapTable[i].cmdId == cmdId) {
      *channelId = mapTable[i].channelId;
      break;
    }
  }
  if(i == num) {
    return OSDK_STAT_ERR;
  }
  return OSDK_STAT_OK;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
