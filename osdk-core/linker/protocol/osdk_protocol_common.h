/**
 ********************************************************************
 * @file    osdk_protocol_common.h
 * @version V1.0.0
 * @date    2019/09/25
 * @brief   Defining the common structure and (exported) function prototypes.
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
#ifndef OSDK_PROTOCOL_COMMON_H
#define OSDK_PROTOCOL_COMMON_H

/* Includes ------------------------------------------------------------------*/
#include "dji_aes.h"
#include "osdk_crc.h"
#include "osdk_logger_internal.h"
#include "osdk_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define OSDK_PACKAGE_MAX_LEN 1024
#define OSDK_COMMAND_DEVICE_ID(type, index) (((index) << 5) | (type))

/* Exported types ------------------------------------------------------------*/
typedef enum {
  OSDK_COMMAND_PACKET_TYPE_REQUEST = 0,
  OSDK_COMMAND_PACKET_TYPE_ACK = 1
} E_OsdkCommandPacketType;

typedef enum {
  OSDK_COMMAND_NEED_ACK_NO_NEED = 0,
  OSDK_COMMAND_NEED_ACK_RECEIVE_ACK = 1,
  OSDK_COMMAND_NEED_ACK_FINISH_ACK = 2
} E_OsdkCommandNeedAckType;

/*! @brief The structure of command information.*/
typedef enum {
  OSDK_COMMAND_ENCRYPT_NO_ENC = 0,
  OSDK_COMMAND_ENCRYPT_AES128 = 1,
  OSDK_COMMAND_ENCRYPT_USER_DEFINED = 2,
  OSDK_COMMAND_ENCRYPT_NOR = 3,
  OSDK_COMMAND_ENCRYPT_DES56 = 4,
  OSDK_COMMAND_ENCRYPT_3DES112 = 5,
  OSDK_COMMAND_ENCRYPT_AES192 = 6,
  OSDK_COMMAND_ENCRYPT_AES256 = 7
} E_OsdkCommandEncryptType;

typedef enum {
  PROTOCOL_SDK = 1,
  PROTOCOL_V1 = 2,
  PROTOCOL_RAW = 3
} E_ProtocolType;

/*! @brief The structure of command information.*/
typedef struct _cmdInfo {

  /*! Specifies packet type.
   *  Value 1 means ack packet, value 0 means request packet.
   *  User must assign values before using send func.
   */
  uint8_t packetType;

  /*! Specifies request command need ack or not.
   *  Only used when packetType value is 0.
   *  Value 1 means req need ack, value 0 means not need. 
   */
  uint8_t needAck;

  /*! Specifies packet neek encrypt or not.
   *  Value 1 means need encrypt, value 0 means not need.
   *  User must assign values before using send func.
   */
  uint8_t encType;

  /*! Specifies sender in v1 protocol.
   *  Only used in v1 packet.
   */
  uint8_t sender;

  /*! Specifies receiver in v1 protocol.
   *  Only used in v1 packet.
   */
  uint8_t receiver;

  /*! Specifies command set.
   *  User must assign values before using send func.
   */
  uint8_t cmdSet;

  /*! Specifies command id.
   *  User must assign values before using send func.
   */
  uint8_t cmdId;

  /*! Specifies session id in sdk protocol.
   *  SessionId will be automatically assigned by the protocol function,
   *  user does not need to fill it out.
   */
  uint8_t sessionId;

  /*! Specifies seqence number.
   *  SeqNum will be automatically assigned by the channel function,
   *  user does not need to fill it out.
   */
  uint16_t seqNum;

  /*! Specifies data part length.
   *  User must assign values before using send func.
   */
  uint16_t dataLen;

  /*! Specifies command transfer channel address.
   *  
   *  User must assign values before using send func.
   */
  uint32_t addr;

  /*! Specifies transfer channel id.
   *  ChannelId will be automatically assigned by the channel function,
   *  user does not need to fill it out.
   */
  uint32_t channelId;

  /*! Specifies packet protocol type.
   *  ProtoType will be automatically assigned by the channel function,
   *  user does not need to fill it out.
   */
  E_ProtocolType protoType;
} T_CmdInfo;

/*! @brief The structure of command parse.*/
typedef struct {

  /*! Specifies parse buff.
   *  the received data is temporarily stored in this buff until
   *  parse func receives a complete frame.
   */
  uint8_t parseBuff[OSDK_PACKAGE_MAX_LEN];

  /*! Specifies parse buff index.*/
  uint16_t parseIndex;
} T_CmdParse;

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif  // OSDK_PROTOCOL_COMMON_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
