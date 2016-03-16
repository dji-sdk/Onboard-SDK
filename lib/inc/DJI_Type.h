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
#define __DELETE(x) delete (char *) x
#else
#define __UNUSED
#define __DELETE(x) delete x
#pragma warning(disable : 4100)
#pragma warning(disable : 4800)
#endif //__GNUC__

#ifdef WIN32
#define __func__ __FUNCTION__
#endif // WIN32

#define API_LOG(driver, title, fmt, ...)                                                       \
    if (title)                                                                                 \
    {                                                                                          \
        (sprintf(DJI::onboardSDK::buffer, "%s %s,line %d: " fmt, title ? title : "NONE",       \
                 __func__, __LINE__, ##__VA_ARGS__));                                          \
        (driver)->displayLog();                                                                \
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

#ifdef API_BUFFER_DATA
#define BUFFER_LOG "BUFFER"
#else
#define BUFFER_LOG 0
#endif

#ifdef API_STATUS_DATA
#define STATUS_LOG "STATUS"
#else
#define STATUS_LOG 0
#endif

#ifdef API_MISSION_DATA
#define MISSION_LOG "MISSION"
#else
#define MISSION_LOG 0
#endif

namespace DJI
{
typedef uint64_t time_ms;

namespace onboardSDK
{

const size_t bufsize = 1024;
extern char buffer[];
extern uint8_t encrypt;

const size_t SESSION_TABLE_NUM = 32;
const size_t CALLBACK_LIST_NUM = 10;

class CoreAPI;

typedef struct Header
{
    unsigned int sof : 8;
    unsigned int length : 10;
    unsigned int version : 6;
    unsigned int sessionID : 5;
    unsigned int isAck : 1;

    unsigned int reversed0 : 2; // always 0

    unsigned int padding : 5;
    unsigned int enc : 3;
    unsigned int reversed1 : 24;

    unsigned int sequenceNumber : 16;
    unsigned int crc : 16;
} Header;

typedef void (*CallBack)(DJI::onboardSDK::CoreAPI *, Header *, void *);
typedef void *UserData;

typedef struct CallBackHandler
{
    CallBack callback;
    UserData userData;
} CallBackHandler;

typedef struct Command
{
    unsigned short sessionMode : 2;
    unsigned short encrypt : 1;
    unsigned short retry : 13;
    unsigned short timeout; // unit is ms
    size_t length;
    uint8_t *buf;
    CallBack handler;
    UserData userData;
} Command;

typedef struct SDKFilter
{
    unsigned short reuseIndex;
    unsigned short reuseCount;
    unsigned short recvIndex;
    unsigned char recvBuf[BUFFER_SIZE];
    // for encrypt
    unsigned char sdkKey[32];
    unsigned char encode;
} SDKFilter;

typedef struct MMU_Tab
{
    unsigned int tabIndex : 8;
    unsigned int usageFlag : 8;
    unsigned int memSize : 16;
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
    uint32_t preSeqNum;
    time_ms preTimestamp;
} CMDSession;

typedef struct ACKSession
{
    uint32_t sessionID : 5;
    uint32_t sessionStatus : 2;
    uint32_t res : 25;
    MMU_Tab *mmu;
} ACKSession;

typedef struct Ack
{
    uint16_t sessionID : 8;
    uint16_t encrypt : 8;
    uint16_t seqNum;
    uint32_t length;
    uint8_t *buf;
} Ack;

#pragma pack(1)

typedef uint8_t BatteryData;
typedef uint8_t MissionACK;

typedef struct GimbalAngleData
{
    int16_t yaw;
    int16_t roll;
    int16_t pitch;
    uint8_t mode;
    uint8_t duration;
} GimbalAngleData;

typedef struct GimbalSpeedData
{
    int16_t yaw;
    int16_t roll;
    int16_t pitch;
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
    uint8_t health : 1;
    uint8_t sensorID : 4;
    uint8_t reserve : 3;
} VelocityData;

typedef struct PositionData
{
    float64_t latitude;
    float64_t longitude;
    float32_t altitude;
    float32_t height;
    uint8_t health;
} PositionData;

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
    uint8_t device : 3;	/*0->rc  1->app  2->serial*/
    uint8_t signature : 1; /*1->opensd  0->close*/
    uint8_t reserved : 4;
} CtrlInfoData;

#ifdef SDK_VERSION_2_3
typedef uint32_t TimeStampData;
#else
typedef struct TimeStampData
{
    //! @todo type modify
    uint32_t time;
    uint32_t nanoTime;
    uint8_t syncFlag;
} TimeStampData;
#endif // SDK_VERSION_2_3

typedef struct GimbalData
{
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
#ifndef SDK_VERSION_2_3
    uint8_t pitchLimit : 1;
    uint8_t rollLimit : 1;
    uint8_t yawLimit : 1;
    uint8_t reserved : 5;
#endif // SDK_VERSION_2_3
} GimbalData;

typedef uint8_t FlightStatus;

typedef struct
{
    unsigned char cmdSequence;
    unsigned char cmdData;
} TaskData;

typedef struct BroadcastData
{
    unsigned short dataFlag;
    TimeStampData timeStamp;
    QuaternionData q;
    CommonData a;
    VelocityData v;
    CommonData w;
    PositionData pos;
    MagnetData mag;
    RadioData rc;
    GimbalData gimbal;
    FlightStatus status; //! @todo define enum
    BatteryData battery;
    CtrlInfoData ctrlInfo;

    //! @note these variables are not send from FMU,
    //! just a record for user.
    uint8_t controlStatus; //! @todo add IO code
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
    //! @note this is default mapping data structure for
    //! virtual remote controller.
    //! @todo channel map
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
