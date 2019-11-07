/**
 ********************************************************************
 * @file    osdk_protocol_sdk.c
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
#include "osdk_protocol_sdk.h"

/* Private functions definition ---------------------------------------------*/
#define _SDK_U32_SET(_addr, _val) (*((uint32_t *)(_addr)) = (_val))

/* Private constants --------------------------------------------------------*/
#define SDK_COMMAND_LENGTH_MAX 1024
#define SDK_COMMAND_LENGTH_MIN \
  (sizeof(T_SdkProtocolHeader) + SDK_COMMAND_CRCDATA_LEN)
#define SDK_COMMAND_CRCHEAD_POSITION \
  (sizeof(T_SdkProtocolHeader) - SDK_COMMAND_CRCHEAD_LEN)
#define SDK_COMMAND_CRCDATA_LEN (sizeof(uint32_t))
#define SDK_COMMAND_CRCHEAD_LEN (sizeof(uint16_t))
#define SDK_COMMAND_SOF 0xAA
#define SDK_COMMAND_SET_LEN 2
#define SDK_COMMAND_MIN_HEADER_LEN (sizeof(T_SdkProtocolHeader))
#define SDK_COMMAND_VERSION 0
#define SDK_ACK_SIZE 10

#define SDK_PROTOCOL_ALIGN (16u)
#define SDK_SESSION_START_NUM 2
#define SDK_MAX_SESSION_NUM (32 - SDK_SESSION_START_NUM)
#define SDK_SESSION_ID_NO_ACK 0
#define SDK_SESSION_ID_IS_USE 1
#define SDK_SESSION_ID_IS_IDLE 0

#define SDK_COMMAND_GET_FRAME_DATAPTR(frame) \
  ((uint8_t *)frame + sizeof(T_SdkProtocolHeader))
#define SDK_COMMAND_GET_FRAME_CRC32(frame)                     \
  ((uint8_t *)frame + ((T_SdkProtocolHeader *)frame)->length - \
   SDK_COMMAND_CRCDATA_LEN)

/* Private types ------------------------------------------------------------*/
// ack code define
#pragma pack(1)

typedef struct {
  uint32_t sof : 8;
  uint32_t length : 10;
  uint32_t version : 6;
  uint32_t sessionId : 5;
  uint32_t isAck : 1;
  uint32_t reserved0 : 2;  // always 0
  uint32_t padding : 5;
  uint32_t enc : 3;
  uint32_t reserved1 : 24;
  uint32_t seqNum : 16;
  uint32_t crc : 16;
} T_SdkProtocolHeader;

typedef struct {
  uint32_t sof : 8;
  uint32_t length : 10;
  uint32_t version : 6;
  uint32_t sessionId : 5;
  uint32_t isAck : 1;
  uint32_t reserved0 : 2;  // always 0
  uint32_t padding : 5;
  uint32_t enc : 3;
  uint32_t reserved1 : 24;
  uint32_t seqNum : 16;
  uint32_t crc : 16;
  uint8_t cmdSet;
  uint8_t cmdId;
  uint8_t pData[1];
} T_SdkProtocolAll;

typedef struct {
  uint8_t sessionId;
  uint8_t flag;
  uint8_t cmdSet;
  uint8_t cmdId;
  uint16_t seqNum;
} T_sdkSessionCtx;

#pragma pack()

typedef struct {
  uint8_t sdkKey[32];
  uint8_t encode;
} SDKFilter;

static SDKFilter s_sdkFilter = {0};

static void OsdkProtocol_sdkEncodeData(T_SdkProtocolHeader *pHeader,
                                       ptr_aes256_codec codecFunc);
static void OsdkProtocol_sdkCalculateCRC(void *pFrame);
static E_OsdkStat OsdkProtocol_sdkGetSessionId(void *protocolExtData,
                                               uint8_t *sessionId,
                                               uint16_t seqNum, uint8_t cmdSet,
                                               uint8_t cmdId);
static E_OsdkStat OsdkProtocol_sdkFindSessionId(void *protocolExtData,
                                                uint8_t sessionId,
                                                uint16_t seqNum,
                                                uint8_t *cmdSet,
                                                uint8_t *cmdId);

/* Exported functions definition ---------------------------------------------*/

E_OsdkStat OsdkProtocol_setSdkKey(const char *key) {
  int i;
  char temp_area[3];
  uint32_t temp8;
  temp_area[0] = temp_area[1] = temp_area[2] = 0;

  for (i = 0; i < 32; i++) {
    temp_area[0] = key[0];
    temp_area[1] = key[1];
    sscanf(temp_area, "%x", &temp8);
    s_sdkFilter.sdkKey[i] = temp8;
    key += 2;
  }
  s_sdkFilter.encode = 1;
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_sdkInit(void **pProtocolExtData) {
  T_sdkSessionCtx *sessionData;
  int i = 0;

  if(!pProtocolExtData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_sdkInit param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  *pProtocolExtData =
      OsdkOsal_Malloc(sizeof(T_sdkSessionCtx) * SDK_MAX_SESSION_NUM);
  if (*pProtocolExtData == NULL) {
    return OSDK_STAT_ERR_ALLOC;
  }

  sessionData = (T_sdkSessionCtx *)(*pProtocolExtData);

  for (i = 0; i < SDK_MAX_SESSION_NUM; i++) {
    sessionData[i].sessionId = (i + SDK_SESSION_START_NUM);
    sessionData[i].seqNum = 0;
    sessionData[i].cmdSet = 0xFF;
    sessionData[i].cmdId = 0xFF;
    sessionData[i].flag = SDK_SESSION_ID_IS_IDLE;
  }

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_sdkPack(void *protocolExtData, uint8_t *pFrame,
                                uint16_t *len, const T_CmdInfo *pInfo,
                                const uint8_t *cmdData) {
  E_OsdkStat osdkStat = OSDK_STAT_OK;
  T_SdkProtocolHeader *pHeader = (T_SdkProtocolHeader *)pFrame;
  T_SdkProtocolAll *pAll = (T_SdkProtocolAll *)pFrame;
  uint16_t frameLen;
  uint16_t frameDataLen;
  uint8_t padding = 0;
  uint8_t sessionId = 0;

  if(!protocolExtData || !pFrame || !len || !pInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_sdkPack param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  frameDataLen = pInfo->dataLen;
  if (frameDataLen + sizeof(T_SdkProtocolHeader) + sizeof(uint32_t) >
      SDK_COMMAND_LENGTH_MAX) {
    OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "frame length error");
    return OSDK_STAT_ERR;
  }

  if ((pInfo->encType) && (frameDataLen > 0)) {
    padding = 16 - (frameDataLen % SDK_PROTOCOL_ALIGN);
    frameDataLen += padding;
  }
  frameLen = frameDataLen + sizeof(T_SdkProtocolHeader);

  if (pInfo->packetType) {
    if (frameDataLen) {
      frameLen += SDK_COMMAND_CRCDATA_LEN;
    }
    memcpy(&pAll->cmdSet, cmdData, pInfo->dataLen);
  } else {
    frameLen += SDK_COMMAND_SET_LEN;
    if (pInfo->dataLen) {
      frameLen += SDK_COMMAND_CRCDATA_LEN;
    }

    pAll->cmdSet = (uint8_t)(pInfo->cmdSet & 0xff);
    pAll->cmdId = (uint8_t)(pInfo->cmdId & 0xff);
    memcpy(pAll->pData, cmdData, pInfo->dataLen);
  }

  if (pInfo->needAck) {
    osdkStat =
        OsdkProtocol_sdkGetSessionId(protocolExtData, &sessionId, pInfo->seqNum,
                                     pInfo->cmdSet, pInfo->cmdId);
    if (osdkStat != OSDK_STAT_OK) {
      OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "get session id error");
      return osdkStat;
    }
    pHeader->sessionId = sessionId;
  } else {
    pHeader->sessionId = SDK_SESSION_ID_NO_ACK;
  }
  pHeader->sof = SDK_COMMAND_SOF;
  pHeader->length = frameLen;
  pHeader->version = SDK_COMMAND_VERSION;
  if (pInfo->packetType)
    pHeader->isAck = 1;
  else
    pHeader->isAck = 0;
  pHeader->enc = pInfo->encType;
  pHeader->seqNum = pInfo->seqNum;
  pHeader->padding = padding;
  pHeader->reserved0 = 0;
  *len = frameLen;
  if (pHeader->enc && s_sdkFilter.encode) {
    OsdkProtocol_sdkEncodeData(pHeader, aes256_encrypt_ecb);
  } else if (pHeader->enc) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "app key not set");
    return OSDK_STAT_ERR;
  }

  OsdkProtocol_sdkCalculateCRC(pFrame);

  OSDK_LOG_DEBUG(
      MODULE_NAME_PROTOCOL,
      "Pack - sender:0x%02X receiver:0x%02X cmdset:0x%02X cmdid:0x%02X",
      pInfo->sender, pInfo->receiver, pInfo->cmdSet, pInfo->cmdId);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_sdkGetLen(char *buffer, uint32_t *length) {
  T_SdkProtocolHeader *pHeader = (T_SdkProtocolHeader *)buffer;
  *length = pHeader->length;
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_sdkParse(T_CmdParse *protParse, uint8_t byte,
                                 uint8_t **pParseFrame, uint16_t *parseLen) {
  uint16_t frameLen;
  uint16_t frameVer;

  if(!protParse || !pParseFrame || !parseLen) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_sdkParse param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (protParse->parseIndex == 0) {
    if (byte != SDK_COMMAND_SOF) {
      return OSDK_STAT_ERR;
    }
  }

  protParse->parseBuff[protParse->parseIndex] = byte;
  protParse->parseIndex++;

  if (protParse->parseIndex >= 3) {
    frameLen = ((T_SdkProtocolHeader *)(protParse->parseBuff))->length;
    frameVer = ((T_SdkProtocolHeader *)(protParse->parseBuff))->version;

    if (frameLen < SDK_COMMAND_LENGTH_MIN ||
        frameLen > SDK_COMMAND_LENGTH_MAX || frameVer != SDK_COMMAND_VERSION) {
      protParse->parseIndex = 0;
      OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol frame length error");
      return OSDK_STAT_ERR;
    }

    if (protParse->parseIndex == SDK_COMMAND_MIN_HEADER_LEN) {
      uint16_t calCrc16;
      calCrc16 = OsdkCrc_sdkCrc16Calc(
          protParse->parseBuff,
          SDK_COMMAND_MIN_HEADER_LEN - SDK_COMMAND_CRCHEAD_LEN);
      if (((T_SdkProtocolHeader *)(protParse->parseBuff))->crc != calCrc16) {
        protParse->parseIndex = 0;
        OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol frame crc16 error");
        return OSDK_STAT_ERR;
      }
    }

    if (protParse->parseIndex == frameLen) {
      uint32_t calCrc32;
      calCrc32 = OsdkCrc_sdkCrc32Calc(protParse->parseBuff,
                                      (frameLen - SDK_COMMAND_CRCDATA_LEN));
      if (*(uint32_t *)SDK_COMMAND_GET_FRAME_CRC32(protParse->parseBuff) !=
          calCrc32) {
        protParse->parseIndex = 0;
        OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol frame crc32 error");
        return OSDK_STAT_ERR;
      } else {
        *pParseFrame = protParse->parseBuff;
        protParse->parseIndex = 0;
        *parseLen = frameLen;
        return OSDK_STAT_OK;
      }
    }

    if (protParse->parseIndex > frameLen) {
      protParse->parseIndex = 0;
      OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol frame length error");
      return OSDK_STAT_ERR;
    }
  }

  return OSDK_STAT_NOT_READY;
}

E_OsdkStat OsdkProtocol_sdkUnpack(void *protocolExtData, uint8_t *pFrame,
                                  T_CmdInfo *pInfo, uint8_t *cmdData) {

  if(!protocolExtData || !pFrame || !pInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_sdkUnpack param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  T_SdkProtocolHeader *pHeader = (T_SdkProtocolHeader *)pFrame;
  if (pHeader->enc) OsdkProtocol_sdkEncodeData(pHeader, aes256_decrypt_ecb);
  uint16_t frameDataLen = pHeader->length - SDK_COMMAND_LENGTH_MIN;
  pInfo->packetType = pHeader->isAck;
  pInfo->encType = pHeader->enc;
  pInfo->protoType = PROTOCOL_SDK;
  pInfo->seqNum = pHeader->seqNum;
  pInfo->sessionId = pHeader->sessionId;
  pInfo->dataLen = frameDataLen;
  pInfo->sender = 0;
  pInfo->receiver = 0;

  if (pHeader->isAck) {
    if (OsdkProtocol_sdkFindSessionId(protocolExtData, pInfo->sessionId,
                                      pInfo->seqNum, &pInfo->cmdSet,
                                      &pInfo->cmdId) != OSDK_STAT_OK) {
      OSDK_LOG_WARN(MODULE_NAME_PROTOCOL,
                    "Unpack error:unknown ack session id");
      return OSDK_STAT_ERR;
    }
  } else {
    T_SdkProtocolAll *pAll = (T_SdkProtocolAll *)pFrame;
    pInfo->cmdSet = pAll->cmdSet;
    pInfo->cmdId = pAll->cmdId;
  }

  if (frameDataLen != 0) {
    memcpy(cmdData, SDK_COMMAND_GET_FRAME_DATAPTR(pFrame), frameDataLen);
  }

  OSDK_LOG_DEBUG(
      MODULE_NAME_PROTOCOL,
      "Unpack - sender:0x%02X receiver:0x%02X cmdset:0x%02X cmdid:0x%02X",
      pInfo->sender, pInfo->receiver, pInfo->cmdSet, pInfo->cmdId);

  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/

static void OsdkProtocol_sdkEncodeData(T_SdkProtocolHeader *pHeader,
                                       ptr_aes256_codec codecFunc) {
  aes256_context ctx;
  uint32_t buf_i;
  uint32_t loop_blk;
  uint32_t data_len;
  uint32_t data_idx;
  uint8_t *data_ptr;

  if (pHeader->enc == 0) return;
  if (pHeader->length <= SDK_COMMAND_LENGTH_MIN) return;

  data_ptr = (uint8_t *)pHeader + sizeof(T_SdkProtocolHeader);
  data_len = pHeader->length - SDK_COMMAND_LENGTH_MIN;

  loop_blk = data_len / 16;
  data_idx = 0;

  aes256_init(&ctx, s_sdkFilter.sdkKey);
  for (buf_i = 0; buf_i < loop_blk; buf_i++) {
    codecFunc(&ctx, data_ptr + data_idx);
    data_idx += 16;
  }
  aes256_done(&ctx);

  if (codecFunc == aes256_decrypt_ecb)
    pHeader->length =
        pHeader->length - pHeader->padding;  // minus padding length;
}

static void OsdkProtocol_sdkCalculateCRC(void *pFrame) {
  T_SdkProtocolHeader *pHeader = (T_SdkProtocolHeader *)pFrame;
  uint8_t *pData = (uint8_t *)pFrame;
  uint32_t crc32Index;

  if (pHeader->sof != SDK_COMMAND_SOF) return;
  if (pHeader->version != SDK_COMMAND_VERSION) return;
  if (pHeader->length > sizeof(T_SdkProtocolHeader) &&
      pHeader->length < SDK_COMMAND_LENGTH_MIN)
    return;

  pHeader->crc = OsdkCrc_sdkCrc16Calc(pData, SDK_COMMAND_CRCHEAD_POSITION);

  if (pHeader->length >= SDK_COMMAND_LENGTH_MIN) {
    crc32Index = pHeader->length - SDK_COMMAND_CRCDATA_LEN;
    _SDK_U32_SET(pData + crc32Index, OsdkCrc_sdkCrc32Calc(pData, crc32Index));
  }
}

static E_OsdkStat OsdkProtocol_sdkGetSessionId(void *protocolExtData,
                                               uint8_t *sessionId,
                                               uint16_t seqNum, uint8_t cmdSet,
                                               uint8_t cmdId) {
  int i = 0;
  T_sdkSessionCtx *sessionData = (T_sdkSessionCtx *)protocolExtData;
  for (i = 0; i < SDK_MAX_SESSION_NUM; i++) {
    if (sessionData[i].flag == SDK_SESSION_ID_IS_USE &&
        sessionData[i].seqNum == seqNum &&
        sessionData[i].cmdSet == cmdSet &&
        sessionData[i].cmdId == cmdId) {
      *sessionId = sessionData[i].sessionId;
      return OSDK_STAT_OK;
    }
  }

  for (i = 0; i < SDK_MAX_SESSION_NUM; i++) {
    if (sessionData[i].flag == SDK_SESSION_ID_IS_IDLE) {
      sessionData[i].seqNum = seqNum;
      sessionData[i].cmdSet = cmdSet;
      sessionData[i].cmdId = cmdId;
      sessionData[i].flag = SDK_SESSION_ID_IS_USE;
      *sessionId = sessionData[i].sessionId;
      break;
    }
  }
  if (i == SDK_MAX_SESSION_NUM) {
    return OSDK_STAT_ERR;
  }
  return OSDK_STAT_OK;
}

static E_OsdkStat OsdkProtocol_sdkFindSessionId(void *protocolExtData,
                                                uint8_t sessionId,
                                                uint16_t seqNum,
                                                uint8_t *cmdSet,
                                                uint8_t *cmdId) {
  int i = 0;
  T_sdkSessionCtx *sessionData = (T_sdkSessionCtx *)protocolExtData;

  for (i = 0; i < SDK_MAX_SESSION_NUM; i++) {
    if (sessionData[i].flag == SDK_SESSION_ID_IS_USE &&
        sessionData[i].seqNum == seqNum &&
        sessionData[i].sessionId == sessionId) {
      *cmdSet = sessionData[i].cmdSet;
      *cmdId = sessionData[i].cmdId;
      sessionData[i].flag = SDK_SESSION_ID_IS_IDLE;
      break;
    }
  }
  if (i == SDK_MAX_SESSION_NUM) {
    return OSDK_STAT_ERR;
  }
  return OSDK_STAT_OK;
}
/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
