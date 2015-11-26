/*
 * DJI_Pro_App.cpp
 *
 *  Created on: Sep 8, 2015
 *      Author: wuyuwei
 *  Modified on: Nov 11, 2015
 *  by william.wu
 */
#ifndef __DJI_PRO_APP_H__
#define __DJI_PRO_APP_H__

#include <stdint.h>

#include "DJI_Link.h"
#include "DJI_Type.h"

#define MY_DEV_ID 0x00

#define API_VER_QUERY 0x00
#define API_CTRL_MANAGEMENT 0x00
#define API_OPEN_SERIAL 0x00
#define API_STD_DATA 0x00
#define API_CMD_REQUEST 0x01
#define API_CMD_STATUS_REQUEST 0x02
#define API_CTRL_REQUEST 0x03
#define API_TRANSPARENT_DATA_TO_MOBILE 0xFE
#define API_TRANSPARENT_DATA_TO_OBOARD 0x02
#define API_GIMBAL_CTRL_SPEED_REQUEST 0x1A
#define API_GIMBAL_CTRL_ANGLE_REQUEST 0x1B

#define API_MISSION_WP_INFO 0x10
#define API_MISSION_WP_DATA 0x11
#define API_MISSION_WP_CMD 0x12

#define API_MISSION_HP_START 0x20
#define API_MISSION_HP_CMD 0x21

#define API_VERSION_QUERY 0x00
#define API_USER_ACTIVATION 0x01
#define API_INFO_QUERY 0x02
#define API_SIM_ECHO 0xFF

#define HORIZ_ATT 0x00
#define HORIZ_VEL 0x40
#define HORIZ_POS 0x80

#define VERT_VEL 0x00
#define VERT_POS 0x10
#define VERT_TRU 0x20

#define YAW_ANG 0x00
#define YAW_RATE 0x08

#define HORIZ_GND 0x00
#define HORIZ_BODY 0x02

#define YAW_GND 0x00
#define YAW_BODY 0x01

// data_type

//----------------------------------------------------------------------
// uav std_msgs reciever
//----------------------------------------------------------------------
#define MSG_ENABLE_FLAG_LEN 2

#define ENABLE_MSG_TIME 0x0001
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

#define DATA_MAX_SIZE (1000u)
#define ERR_INDEX (0xff)
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

/*
 *struct of cmd agency data
 */

typedef struct
{
    unsigned char cmd_sequence;
    unsigned char cmd_data;
} TaskData;

//----------------------------------------------------------------------
// for activation
//----------------------------------------------------------------------

/*
 *  code re-construction according onboard api protocol
 */

#define SDK_ERR_SUCCESS 0x0000
#define SDK_ERR_COMMAND_NOT_SUPPORTED 0xFF00
#define SDK_ERR_NO_AUTHORIZED 0xFF01
#define SDK_ERR_NO_RIGHTS 0xFF02
#define SDK_ERR_NO_RESPONSE 0xFFFF

#define SDK_ACTIVATE_SUCCESS 0x0000
#define SDK_ACTIVATE_PARAM_ERROR 0x0001
#define SDK_ACTIVATE_DATA_ENC_ERROR 0x0002
#define SDK_ACTIVATE_NEW_DEVICE 0x0003
#define SDK_ACTIVATE_DJI_APP_NOT_CONNECT 0x0004
#define SDK_ACTIVATE_DIJ_APP_NO_INTERNET 0x0005
#define SDK_ACTIVATE_SERVER_REFUSED 0x0006
#define SDK_ACTIVATE_LEVEL_ERROR 0x0007
#define SDK_ACTIVATE_SDK_VERSION_ERROR 0x0008

#pragma pack(1)

/*
 *struct of activate data
 */

typedef struct
{
    unsigned int app_id;
    unsigned int app_api_level;
    unsigned int app_ver;
    unsigned char app_bundle_id[32];
    char *app_key;
} ActivateData;

/*
 *struct of version query data
 */

typedef struct
{
    unsigned short version_ack;
    unsigned int version_crc;
    char version_name[32];
} VersionData;

/*
 *struct of attitude data
 */

typedef struct
{
    unsigned char ctrl_flag;
    float roll_or_x;
    float pitch_or_y;
    float thr_z;
    float yaw;
} FlightData;

#pragma pack()

typedef void (*CommandResult)(unsigned short result);
typedef void (*Get_API_Version_Notify)(VersionData *);
typedef void (*ReceiveHandler)(DJI::onboardSDK::Header *pHeader);
typedef void (*BroadcastHandler)(void);
typedef void (*TransparentHandler)(unsigned char *buf, unsigned char len);
typedef void (*ResultCallback)(DJI::onboardSDK::CoreAPI *);

using namespace DJI::onboardSDK;
const uint8_t _broadcast0 = sizeof(TimeStampData);
const uint8_t _broadcast1 = sizeof(TimeStampData) + sizeof(QuaternionData);
const uint8_t _broadcast2 =
    sizeof(TimeStampData) + sizeof(QuaternionData) + sizeof(CommonData);
const uint8_t _broadcast3 = sizeof(TimeStampData) + sizeof(QuaternionData) +
                            sizeof(CommonData) + sizeof(SpeedData);
const uint8_t _broadcast4 = sizeof(TimeStampData) + sizeof(QuaternionData) +
                            sizeof(CommonData) + sizeof(SpeedData) +
                            sizeof(CommonData);
const uint8_t _broadcast5 = sizeof(TimeStampData) + sizeof(QuaternionData) +
                            sizeof(CommonData) + sizeof(SpeedData) +
                            sizeof(CommonData) + sizeof(PositionData);
const uint8_t _broadcast6 = sizeof(TimeStampData) + sizeof(QuaternionData) +
                            sizeof(CommonData) + sizeof(SpeedData) +
                            sizeof(CommonData) + sizeof(PositionData) +
                            sizeof(MagnetData);
const uint8_t _broadcast7 = sizeof(TimeStampData) + sizeof(QuaternionData) +
                            sizeof(CommonData) + sizeof(SpeedData) +
                            sizeof(CommonData) + sizeof(PositionData) +
                            sizeof(MagnetData) + sizeof(RadioData);
const uint8_t _broadcast8 =
    sizeof(TimeStampData) + sizeof(QuaternionData) + sizeof(CommonData) +
    sizeof(SpeedData) + sizeof(CommonData) + sizeof(PositionData) +
    sizeof(MagnetData) + sizeof(RadioData) + sizeof(GimbalData);
const uint8_t _broadcast9 = sizeof(TimeStampData) + sizeof(QuaternionData) +
                            sizeof(CommonData) + sizeof(SpeedData) +
                            sizeof(CommonData) + sizeof(PositionData) +
                            sizeof(MagnetData) + sizeof(RadioData) +
                            sizeof(GimbalData) + sizeof(uint8_t);
const uint8_t _broadcast10 =
    sizeof(TimeStampData) + sizeof(QuaternionData) + sizeof(CommonData) +
    sizeof(SpeedData) + sizeof(CommonData) + sizeof(PositionData) +
    sizeof(MagnetData) + sizeof(RadioData) + sizeof(GimbalData) +
    sizeof(uint8_t) + sizeof(BatteryData);
const uint8_t _broadcast11 =
    sizeof(TimeStampData) + sizeof(QuaternionData) + sizeof(CommonData) +
    sizeof(SpeedData) + sizeof(CommonData) + sizeof(PositionData) +
    sizeof(MagnetData) + sizeof(RadioData) + sizeof(GimbalData) +
    sizeof(uint8_t) + sizeof(BatteryData) + sizeof(CtrlInfoData);
const uint8_t _broadcastLen[] = { _broadcast0, _broadcast1,  _broadcast2,
                                  _broadcast3, _broadcast4,  _broadcast5,
                                  _broadcast6, _broadcast7,  _broadcast8,
                                  _broadcast9, _broadcast10, _broadcast11 };

#endif
