/**
 ********************************************************************
 * @file    osdk_protocol_v1.c
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
#include "osdk_protocol_v1.h"
#include "dji_aes.h"
#include "osdk_crc.h"

/* Private constants --------------------------------------------------------*/

#define V1_COMMAND_DATA_LENGTH_MAX (1024 - (sizeof(T_V1ProtocolHeader) + 2))
#define V1_COMMAND_DEVICE_ID(type, index) (((index) << 5) | (type))

#define V1_COMMAND_SOF 0x55
#define V1_COMMAND_LENGTH_MIN (sizeof(T_V1ProtocolHeader) + 2)
#define V1_COMMAND_LENGTH_MAX \
  (V1_COMMAND_DATA_LENGTH_MAX + sizeof(T_V1ProtocolHeader) + 2)
#define V1_COMMAND_DATA_OFFSET (sizeof(T_V1ProtocolHeader))
#define V1_COMMAND_MIN_HEADER_LEN 4
#define V1_COMMAND_CRCDATA_LEN (sizeof(uint16_t))
#define V1_COMMAND_CRCHEAD_LEN (sizeof(uint8_t))

#define V1_COMMAND_GET_FRAME_LEN(frame) \
  (((T_V1ProtocolHeader *)frame)->lenAndVer.lv.len)
#define V1_COMMAND_GET_FRAME_CRC16(frame) \
  (((uint8_t *)frame + ((T_V1ProtocolHeader *)frame)->lenAndVer.lv.len - 2))
#define V1_COMMAND_GET_FRAME_DATAPTR(frame) (frame + sizeof(T_V1ProtocolHeader))

#define V1_COMMAND_VERSION 1

/* Private types ------------------------------------------------------------*/
#pragma pack(1)

typedef struct {
  uint8_t sof;
  union {
    struct {
      uint16_t len : 10;
      uint16_t ver : 6;
    } lv;
    uint16_t infoData;
  } lenAndVer;
  uint8_t headCheckCrc8;
  union {
    struct {
      uint8_t type : 5;
      uint8_t index : 3;
    } id;
    uint8_t infoData;
  } sender;
  union {
    struct {
      uint8_t type : 5;
      uint8_t index : 3;
    } id;
    uint8_t infoData;
  } receiver;
  uint16_t seqNum;
  union {
    struct {
      uint8_t encrypt : 4; /*!< @ref v1protocol_encrypt*/
      uint8_t reserve : 1;
      uint8_t needAck : 2;    /*!< @ref v1protocol_needAck*/
      uint8_t packetType : 1; /*!< @ref v1protocol_packetType */
    } cmdType;
    uint8_t infoData;
  } cmdType;
  uint8_t cmdSet; /*!< @ref v1protocol_CmdSet */
  uint8_t cmdId;
} T_V1ProtocolHeader;

#pragma pack()

/* Exported functions definition ---------------------------------------------*/
E_OsdkStat OsdkProtocol_v1Init(void **pProtocolExtData) {
  *pProtocolExtData = NULL;
  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_v1Pack(void *protocolExtData, uint8_t *pFrame,
                               uint16_t *len, const T_CmdInfo *pInfo,
                               const uint8_t *cmdData) {
  T_V1ProtocolHeader *pHeader = (T_V1ProtocolHeader *)pFrame;
  uint16_t frameLen;
  uint16_t frameDataLen;

  if(!pFrame || !len || !pInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_v1Pack param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  frameDataLen = pInfo->dataLen;
  frameLen = frameDataLen + V1_COMMAND_LENGTH_MIN;
  *len = frameLen;

  pHeader->sof = V1_COMMAND_SOF;
  pHeader->lenAndVer.lv.len = frameLen;
  pHeader->lenAndVer.lv.ver = V1_COMMAND_VERSION;
  pHeader->headCheckCrc8 = OsdkCrc_crc8Calc(
      pFrame, (V1_COMMAND_MIN_HEADER_LEN - V1_COMMAND_CRCHEAD_LEN));
  pHeader->cmdType.cmdType.needAck = pInfo->needAck;
  pHeader->cmdType.cmdType.packetType = pInfo->packetType;
  pHeader->cmdType.cmdType.encrypt = pInfo->encType;
  pHeader->cmdSet = pInfo->cmdSet;
  pHeader->cmdId = pInfo->cmdId;
  pHeader->seqNum = pInfo->seqNum;
  pHeader->sender.infoData = pInfo->sender;

  pHeader->receiver.infoData = pInfo->receiver;

  OSDK_LOG_DEBUG(
      MODULE_NAME_PROTOCOL,
      "PACK - sender:0x%02X receiver:0x%02X cmdset:0x%02X cmdid:0x%02X",
      pInfo->sender, pInfo->receiver, pInfo->cmdSet, pInfo->cmdId);

  memcpy(&pFrame[sizeof(T_V1ProtocolHeader)], cmdData, frameDataLen);

  uint16_t crc16 = OsdkCrc_ccittCrc16Calc(
      pFrame, (uint16_t)(pHeader->lenAndVer.lv.len - V1_COMMAND_CRCDATA_LEN));
  memcpy(&pFrame[pHeader->lenAndVer.lv.len - V1_COMMAND_CRCDATA_LEN],
         (uint8_t *)&crc16, V1_COMMAND_CRCDATA_LEN);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_v1Parse(T_CmdParse *protParse, uint8_t byte,
                                uint8_t **pParseFrame, uint16_t *parseLen) {
  uint16_t frameLen;
  uint16_t frameVer;

  if(!protParse || !pParseFrame || !parseLen) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_v1Parse param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  if (protParse->parseIndex == 0) {
    if (byte != V1_COMMAND_SOF) {
      OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol no frame error");
      return OSDK_STAT_ERR;
    }
  }

  protParse->parseBuff[protParse->parseIndex] = byte;
  protParse->parseIndex++;

  if (protParse->parseIndex >= 3) {
    frameLen = ((T_V1ProtocolHeader *)(protParse->parseBuff))->lenAndVer.lv.len;
    frameVer = ((T_V1ProtocolHeader *)(protParse->parseBuff))->lenAndVer.lv.ver;

    if (frameLen < V1_COMMAND_LENGTH_MIN || frameLen > V1_COMMAND_LENGTH_MAX ||
        frameVer != V1_COMMAND_VERSION) {
      protParse->parseIndex = 0;
      OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol frame length error");
      return OSDK_STAT_ERR;
    }

    if (protParse->parseIndex == V1_COMMAND_MIN_HEADER_LEN) {
      uint16_t calCrc8;
      calCrc8 =
          OsdkCrc_crc8Calc(protParse->parseBuff,
                           V1_COMMAND_MIN_HEADER_LEN - V1_COMMAND_CRCHEAD_LEN);
      if (((T_V1ProtocolHeader *)(protParse->parseBuff))->headCheckCrc8 !=
          calCrc8) {
        protParse->parseIndex = 0;
        OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol frame crc8 error");
        return OSDK_STAT_ERR;
      }
    }

    if (protParse->parseIndex == frameLen) {
      uint32_t calCrc16;
      calCrc16 = OsdkCrc_ccittCrc16Calc(protParse->parseBuff,
                                   (size_t)(frameLen - V1_COMMAND_CRCDATA_LEN));
      if (*(uint16_t *)V1_COMMAND_GET_FRAME_CRC16(protParse->parseBuff) !=
          calCrc16) {
        protParse->parseIndex = 0;
        OSDK_LOG_WARN(MODULE_NAME_PROTOCOL, "protocol frame crc16 error");
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

E_OsdkStat OsdkProtocol_v1Unpack(void *protocolExtData, uint8_t *pFrame,
                                 T_CmdInfo *pInfo, uint8_t *cmdData) {
  if(!pFrame || !pInfo || !cmdData) {
    OSDK_LOG_ERROR(MODULE_NAME_PROTOCOL, "OsdkProtocol_v1Unpack param check failed");
    return OSDK_STAT_ERR_PARAM;
  }

  T_V1ProtocolHeader *pHeader = (T_V1ProtocolHeader *)pFrame;
  uint16_t frameDataLen = pHeader->lenAndVer.lv.len -
                          sizeof(T_V1ProtocolHeader) - V1_COMMAND_CRCDATA_LEN;

  pInfo->packetType = pHeader->cmdType.cmdType.packetType;
  pInfo->needAck = pHeader->cmdType.cmdType.needAck;
  pInfo->encType = pHeader->cmdType.cmdType.encrypt;
  pInfo->protoType = PROTOCOL_V1;
  pInfo->seqNum = pHeader->seqNum;

  pInfo->sender = pHeader->sender.infoData;
  pInfo->receiver = pHeader->receiver.infoData;
  pInfo->cmdSet = pHeader->cmdSet;
  pInfo->cmdId = pHeader->cmdId;
  pInfo->dataLen = frameDataLen;
  pInfo->sessionId = 0xff;

  if (frameDataLen != 0) {
    memcpy(cmdData, V1_COMMAND_GET_FRAME_DATAPTR(pFrame), pInfo->dataLen);
  }

  OSDK_LOG_DEBUG(
      MODULE_NAME_PROTOCOL,
      "Unpack - sender:0x%02X receiver:0x%02X cmdset:0x%02X cmdid:0x%02X",
      pInfo->sender, pInfo->receiver, pInfo->cmdSet, pInfo->cmdId);

  return OSDK_STAT_OK;
}

E_OsdkStat OsdkProtocol_v1GetLen(char *buffer, uint32_t *length) {
  T_V1ProtocolHeader *pHeader = (T_V1ProtocolHeader *)buffer;
  *length = pHeader->lenAndVer.lv.len;
  return OSDK_STAT_OK;
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
