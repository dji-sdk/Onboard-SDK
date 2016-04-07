/*! @brief
 *  @file DJI_APP.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Core API Link upper level for DJI onboardSDK library
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Dec 16, 2015
 *  -* @author william.wu
 *
 * */

#ifndef DJI_APP_H
#define DJI_APP_H

#include <stdint.h>

#include "DJI_Link.h"
#include "DJI_Type.h"

#define MSG_ENABLE_FLAG_LEN 2

#define HAS_TIME 0x0001
#define HAS_Q 0x0002
#define HAS_A 0x0004
#define HAS_V 0x0008
#define HAS_W 0x0010
#define HAS_POS 0x0020
#define HAS_MAG 0x0040
#define HAS_RC 0x0080
#define HAS_GIMBAL 0x0100
#define HAS_STATUS 0x0200
#define HAS_BATTERY 0x0400
#define HAS_DEVICE 0x0800

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------
typedef struct
{
    unsigned short sequence_number;
    unsigned char session_id : 5;
    unsigned char need_encrypt : 1;
    unsigned char reserve : 2;
} req_id_t;

#define EXC_DATA_SIZE (16u)
#define SET_CMD_SIZE (2u)

//----------------------------------------------------------------------
// for cmd agency
//----------------------------------------------------------------------
#define REQ_TIME_OUT 0x0000
#define REQ_REFUSE 0x0001
#define CMD_RECIEVE 0x0002
#define STATUS_CMD_EXECUTING 0x0003
#define STATUS_CMD_EXE_FAIL 0x0004
#define STATUS_CMD_EXE_SUCCESS 0x0005

#pragma pack(1)

typedef struct ActivateData
{
    unsigned int ID;
    unsigned int reserved;
    unsigned int version;
    unsigned char iosID[32];
    char *encKey;
} ActivateData;

typedef struct VersionData
{
    unsigned short version_ack;
    unsigned int version_crc;
#ifdef SDK_VERSION_3_1
    char version_ID[11];
#endif // SDK_VERSION_3_1
    char version_name[32];
    DJI::onboardSDK::Version version;
} VersionData;

#pragma pack()

#endif // DJI_APP_H
