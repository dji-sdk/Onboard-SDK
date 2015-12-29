/*! @brief
 *  @file DJI_Type.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Type definition for DJI onboardSDK library
 *  Maintain officially
 *
 *  @attention
 *  Maintain officially, readonly for users
 *  Do not modify any definition in this file,
 *  if you are not sure what are you doing exactlly,
 *  or we will not provide any support.
 *
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Dec 16, 2015
 *  -* @author william.wu
 *
 * */

/*! @attention
 *  Maintain officially, readonly for users
 *  Do not modify any definition in this file,
 *  if you are not sure what are you doing exactlly,
 *  or we will not provide any support.
 * */

#ifndef DJI_TYPE
#define DJI_TYPE

#include "DJI_Config.h"
#include <stdio.h>

#define NAME(x) #x

#ifdef __GNUC__
#define __UNUSED __attribute__((__unused__))
#else
#define __UNUSED
#endif //__GNUC__

#ifdef WIN32
#define __func__ __FUNCTION__
#endif // WIN32

#define APIprintf(...) sprintf(DJI::onboardSDK::buffer, ##__VA_ARGS__)

#define API_LOG(driver, title, fmt, ...)                                       \
    {                                                                          \
        if ((title))                                                           \
        {                                                                      \
            APIprintf("%s %s,line %d: " fmt, title ? title : "NONE", __func__, \
                      __LINE__, ##__VA_ARGS__);                                \
            (driver)->displayLog();                                            \
        }                                                                      \
    }

#ifdef API_DEBUG_DATA
#define DEBUG_LOG "DEBUG"
#else
#define DEBUG_LOG 0
#endif

#ifdef API_ERROR_DATA
#define ERROR_LOG "ERROR"
#else
#define ERROR_LOG 0
#endif

#ifdef API_STATUS_DATA
#define STATUS_LOG "STATUS"
#else
#define STATUS_LOG 0
#endif

namespace DJI
{
namespace onboardSDK
{
const size_t bufsize = 1024;
extern char buffer[];

const size_t SESSION_TABLE_NUM = 32;
const size_t CALLBACK_LIST_NUM = 10;
class CoreAPI;

typedef struct Header
{
    unsigned int sof : 8;
    unsigned int length : 10;
    unsigned int version : 6;
    unsigned int sessionID : 5;
    unsigned int is_ack : 1;

    unsigned int reversed0 : 2; // always 0

    unsigned int padding : 5;
    unsigned int enc_type : 3;
    unsigned int reversed1 : 24;

    unsigned int sequence_number : 16;
    unsigned int head_crc : 16;
} Header;

typedef void (*CallBack)(DJI::onboardSDK::CoreAPI *, Header *, void *);
typedef void* UserData;

typedef struct CallBackHandler
{
    CallBack callback;
    UserData userData;
}CallBackHandler;

typedef struct Command
{
    unsigned short session_mode : 2;
    unsigned short need_encrypt : 1;
    unsigned short retry_time : 13;
    unsigned short timeout; // unit is ms
    size_t length;
    uint8_t *buf;
    CallBack handler;
    UserData userData;
} Command;

typedef struct SDKFilter
{
    unsigned short reuse_index;
    unsigned short reuse_count;
    unsigned short recv_index;
    unsigned char comm_recv_buf[BUFFER_SIZE];
    // for encrypt
    unsigned char comm_key[32];
    unsigned char enc_enabled;
} SDKFilter;

typedef struct MMU_Tab
{
    unsigned int tab_index : 8;
    unsigned int usage_flag : 8;
    unsigned int mem_size : 16;
    unsigned char *pmem;
} MMU_Tab;

typedef struct CMDSession
{
    uint32_t sessionID : 5;
    uint32_t usageFlag : 1;
    uint32_t sent : 5;
    uint32_t retry : 5;
    uint32_t timeout : 16;
    MMU_Tab *mmu;
    CallBack handler;
    UserData userData;
    uint32_t pre_seq_num;
    uint32_t pre_timestamp;
} CMDSession;

typedef struct ACKSession
{
    uint32_t sessionID : 5;
    uint32_t session_status : 2;
    uint32_t res : 25;
    MMU_Tab *mmu;
} ACKSession;

typedef struct Ack
{
    uint16_t session_id : 8;
    uint16_t need_encrypt : 8;
    uint16_t seq_num;
    uint32_t length;
    uint8_t *buf;
} Ack;

#pragma pack(1)

typedef uint8_t BatteryData;
typedef uint8_t MissionACK;

typedef struct GimbalAngleData
{
    int16_t yaw_angle;
    int16_t roll_angle;
    int16_t pitch_angle;
    uint8_t ctrl_byte;
    uint8_t duration;
} GimbalAngleData;

typedef struct GimbalSpeedData
{
    int16_t yaw_angle_rate;
    int16_t roll_angle_rate;
    int16_t pitch_angle_rate;
    uint8_t reserved; // always 0x80;
} GimbalSpeedData;

typedef float float32_t;
typedef double float64_t;

typedef struct QuaternionData
{
    float32_t q0;
    float32_t q1;
    float32_t q2;
    float32_t q3;
} QuaternionData;

typedef struct CommonData
{
    float32_t x;
    float32_t y;
    float32_t z;
} CommonData;

typedef struct VelocityData
{
    float32_t x;
    float32_t y;
    float32_t z;
    uint8_t health_flag : 1;
    uint8_t feedback_sensor_id : 4;
    uint8_t reserve : 3;
} VelocityData;

typedef struct PossitionData
{
    float64_t latitude;
    float64_t longitude;
    float32_t altitude;
    float32_t height;
    uint8_t health;
} PossitionData;

typedef struct RadioData
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t throttle;
    int16_t mode;
    int16_t gear;
} RadioData;

typedef struct MagnetData
{
    int16_t x;
    int16_t y;
    int16_t z;
} MagnetData;

typedef struct GPSData
{
    float64_t latitude;
    float64_t longtitude;
    float64_t altitude;
} GPSData;

typedef struct CtrlInfoData
{
#ifndef SDK_VERSION_2_3
    uint8_t data;
#endif // SDK_VERSION_2_3
    //! @todo mode remote to enums
    uint8_t cur_ctrl_dev_in_navi_mode : 3; /*0->rc  1->app  2->serial*/
    uint8_t serial_req_status : 1;		   /*1->opensd  0->close*/
    uint8_t reserved : 4;
} CtrlInfoData;

#ifdef SDK_VERSION_2_3
typedef uint32_t TimeStampData;
#else
typedef struct TimeStampData
{
    uint32_t time;
    uint32_t asr_ts;
    uint8_t sync_flag;
} TimeStampData;
#endif // SDK_VERSION_2_3

typedef struct GimbalData
{
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
#ifndef SDK_VERSION_2_3
    uint8_t is_pitch_limit : 1;
    uint8_t is_roll_limit : 1;
    uint8_t is_yaw_limit : 1;
    uint8_t reserved : 5;
#endif // SDK_VERSION_2_3
} GimbalData;

typedef uint8_t FlightStatus;

typedef struct
{
    unsigned char cmd_sequence;
    unsigned char cmd_data;
} TaskData;

typedef struct BroadcastData
{
    unsigned short dataFlag;
    TimeStampData timeStamp;
    QuaternionData q;
    CommonData a;
    VelocityData v;
    CommonData w;
    PossitionData pos;
    MagnetData mag;
    RadioData rc;
    GimbalData gimbal;
    FlightStatus status; //! @todo define enum
    BatteryData capacity;
    CtrlInfoData ctrl_info;
    uint8_t activation;
} BroadcastData;

typedef struct VirtualRCSetting
{
    uint8_t enable : 1;
    uint8_t cutoff : 1;
    uint8_t reserved : 6;
} VirtualRCSetting;

typedef struct VirtualRCData
{
    uint32_t roll;
    uint32_t pitch;
    uint32_t throttle;
    uint32_t yaw;
    uint32_t gear;
    uint32_t reserved;
    uint32_t mode;
    uint32_t Channel_07;
    uint32_t Channel_08;
    uint32_t Channel_09;
    uint32_t Channel_10;
    uint32_t Channel_11;
    uint32_t Channel_12;
    uint32_t Channel_13;
    uint32_t Channel_14;
    uint32_t Channel_15;
} VirtualRCData;

#pragma pack()
} // namespace onboardSDK
} // namespace DJI

#define PRO_PURE_DATA_MAX_SIZE 1007 // 2^10 - header size
const size_t MMU_TABLE_NUM = 32;

#endif // DJI_TYPE
