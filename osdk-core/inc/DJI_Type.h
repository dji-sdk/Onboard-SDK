/*! @file DJI_Type.h
 *  @version 3.1.9
 *  @date November 10, 2016
 *
 *  @brief
 *  Type definition for DJI onboardSDK library.
 *  Officially Maintained
 *  
 *  @copyright
 *  Copyright 2016 DJI. All rights reserved.
 * */

/*! @attention
 *  Do not modify any definition in this file
 *  if you are unsure about what are you doing.
 *  DJI will not provide any support for changes made to this file.
 * */

#ifndef DJI_TYPE
#define DJI_TYPE

#include "DJI_Config.h"
#include "DJICommonType.h"
#include <stdio.h>
#include <exception>
#include <stdexcept>

#define NAME(x) #x

//! Define the UNUSED macro to suppress compiler warnings about unused arguments 
#ifdef __GNUC__
#define __UNUSED __attribute__((__unused__))
#define __DELETE(x) delete (char *) x
#else
#define __UNUSED
#define __DELETE(x) delete x


//! @todo fix warning.
#ifndef STM32
#pragma warning(disable : 4100)
#pragma warning(disable : 4800)
#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#pragma warning(disable : 4267)
#pragma warning(disable : 4700)
#pragma warning(disable : 4101)
#endif // STM32
#endif //__GNUC__

#ifdef WIN32
#define __func__ __FUNCTION__
#endif // WIN32


//! This is the default status printing mechanism
#define API_LOG(driver, title, fmt, ...)                                  \
  if ((title))                                                            \
  {                                                                       \
    int len = (sprintf(DJI::onboardSDK::buffer, "%s %s,line %d: " fmt,    \
        (title) ? (title) : "NONE", __func__, __LINE__, ##__VA_ARGS__));  \
    if ((len != -1) && (len < 1024))                                      \
      (driver)->displayLog();                                             \
    else                                                                  \
      (driver)->displayLog("ERROR: log printer inner fault\n");           \
  }

#ifdef API_TRACE_DATA
#define TRACE_LOG "TRACE"
#else
#define TRACE_LOG 0
#endif

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

#ifdef API_RTK_DEBUG
#define RTK_LOG "MISSION"
#else
#define RTK_LOG 0
#endif

//! @note for ARMCC-5.0 compiler
#ifdef ARMCC
#pragma anon_unions
#endif

namespace DJI
{
namespace onboardSDK
{

const size_t bufsize = 1024;
extern char buffer[];
extern uint8_t encrypt;

const size_t SESSION_TABLE_NUM = 32;
const size_t CALLBACK_LIST_NUM = 10;

/**
 * @note size is in Bytes
 */
const size_t MAX_ACK_SIZE = 107;

//! The CoreAPI class definition is detailed in DJI_API.h 
class CoreAPI;

//! The Header struct is meant to handle the open protocol header.
typedef struct Header
{
  unsigned int sof : 8;
  unsigned int length : 10;
  unsigned int version : 6;
  unsigned int sessionID : 5;
  unsigned int isAck : 1;
  //! @warning this field will change reversed0 --> reserved0 in the next release 
  unsigned int reversed0 : 2; // always 0

  unsigned int padding : 5;
  unsigned int enc : 3;
  //! @warning this field will change reversed1 --> reserved1 in the next release 
  unsigned int reversed1 : 24;

  unsigned int sequenceNumber : 16;
  unsigned int crc : 16;
} Header;

//! The CallBack function pointer is used as an argument in api->send calls
typedef void (*CallBack)(DJI::onboardSDK::CoreAPI *, Header *, UserData);

//! The CallBackHandler struct allows users to encapsulate callbacks and data in one struct 
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

//! @warning this struct will be renamed in a future release.
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

//! @warning this struct will be renamed in a future release.
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

/**
 * Gimbal Data
 */
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

/**
 * HotPoint Data
 */
typedef struct HotPointData
{
  uint8_t version;

  float64_t latitude;
  float64_t longitude;
  float64_t height;

  float64_t radius;
  float32_t yawRate; // degree

  uint8_t clockwise;
  uint8_t startPoint;
  uint8_t yawMode;
  uint8_t reserved[11];
} HotPointData;

/**
 * WayPoint Data
 */
typedef struct WayPointInitData
{
  uint8_t indexNumber;
  float32_t maxVelocity;
  float32_t idleVelocity;

  uint8_t finishAction;
  uint8_t executiveTimes;
  uint8_t yawMode;
  uint8_t traceMode;
  uint8_t RCLostAction;
  uint8_t gimbalPitch;
  float64_t latitude;  //! @note For Camera to recording
  float64_t longitude; //! not supported yet
  float32_t altitude;

  uint8_t reserved[16];
} WayPointInitData;

typedef struct WayPointData
{
  uint8_t index;

  float64_t latitude;
  float64_t longitude;
  float32_t altitude;
  float32_t damping;

  int16_t yaw;
  int16_t gimbalPitch;
  uint8_t turnMode;

  uint8_t reserved[8];
  uint8_t hasAction;
  uint16_t actionTimeLimit;

  uint8_t actionNumber : 4;
  uint8_t actionRepeat : 4;

  uint8_t commandList[16];//! @note issues here list number is 15
  int16_t commandParameter[16];
} WayPointData;

/**
 * ACK Data
 */

typedef uint8_t MissionACK;
typedef uint16_t SimpleACK;

typedef struct HotPointStartACK
{
  uint8_t ack;
  float32_t maxRadius;
} HotpointStartACK;

typedef struct WayPointInitACK
{
  uint8_t ack;
  WayPointInitData data;
} WayPointInitACK;

typedef struct WayPointDataACK
{
  uint8_t ack;
  uint8_t index;
  WayPointData data;
} WayPointDataACK;

typedef struct WayPointVelocityACK
{
  uint8_t ack;
  float32_t idleVelocity;
} WayPointVelocityACK;

// HotPoint data read from flight controller
typedef struct HotPointReadACK
{
  MissionACK ack;
  HotPointData data;
} HotpointReadACK;

typedef struct DroneVersionACK
{
  unsigned char ack[MAX_ACK_SIZE];
} DroneVersionACK;

typedef union MissionACKUnion
{ 
  uint8_t raw_ack_array[MAX_ACK_SIZE];
  DroneVersionACK droneVersion;
  MissionACK missionACK;

  SimpleACK simpleACK;

  HotPointStartACK hotpointStartACK;

  // Contains 1-Byte ACK plus hotpoint mission
  // information read from flight controller
  HotpointReadACK hotpointReadACK;

  // Contains 1-Byte ACK plus waypoint mission
  // information read from flight controller
  WayPointInitACK waypointInitACK;

  // Contains 1-Byte ACK plus waypoint mission
  // information read from flight controller
  WayPointDataACK waypointDataACK;

  WayPointVelocityACK waypointVelocityACK;
} MissionACKUnion; 

typedef struct QuaternionData
{
  float32_t q0;
  float32_t q1;
  float32_t q2;
  float32_t q3;
} QuaternionData;

//! @warning this struct will be deprecated in the next release and renamed to Vector3fData. Use Vector3fData instead.
typedef struct CommonData
{
  float32_t x;
  float32_t y;
  float32_t z;
} CommonData;

//! @note this struct will replace CommonData in the next release.
//! Eigen-like naming convention
typedef struct Vector3fData
{
  float32_t x;
  float32_t y;
  float32_t z;
} Vector3fData;

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
  //! @warning the 'altitude' field will be renamed in a future release.
  //! @note the altitude value can be configured to output GPS-only data
  //!       or a fusion of GPS and Baro in Assistant 2's SDK Tab, 'ALTI' 
  float32_t altitude;

  //! @warning the 'height' field will be renamed in a future release.
  //! @note the height value can be configured to output AGL height
  //!       or height relative to takeoff in Assistant 2's SDK Tab, 'HEIGHT'
  float32_t height;

  uint8_t health;
} PositionData;

 //! @warning the 'RadioData' struct will be deprecated in the next release and renamed to RCData. Use RCData instead.
typedef struct RadioData
{
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t throttle;
  int16_t mode;
  int16_t gear;
} RadioData;

//! @note This struct will replace RadioData in the next release. 
typedef struct RCData
{
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  int16_t throttle;
  int16_t mode;
  int16_t gear;
} RCData;

//! @warning the 'MagnetData' struct will be deprecated in the next release and renamed to MagData. Use MagData instead.
typedef struct MagnetData
{
  int16_t x;
  int16_t y;
  int16_t z;
} MagnetData;

//! @note This struct will replace MagnetData in the next release.
typedef struct MagData
{
  int16_t x;
  int16_t y;
  int16_t z;
} MagData;

//! @note This struct is provided as a means for users to provide sigle GPS points to the SDK.
//!       It does not follow standard SDK GPS datatypes. This may change in a future release. 
typedef struct GPSPositionData
{
  float64_t latitude;
  float64_t longitude;
  //! @warning please provide relative height in the altitude field. The name will change in a future release.
  float64_t altitude;

} GPSPositionData;

typedef struct CtrlInfoData
{
  uint8_t mode;
  //! @todo mode remote to enums
  uint8_t deviceStatus : 3; /*0->rc  1->app  2->serial*/
  uint8_t flightStatus : 1; /*1->opensd  0->close*/
  uint8_t vrcStatus : 1;
  uint8_t reserved : 3;
} CtrlInfoData;

typedef struct TimeStampData
{
  //! @todo type modify
  uint32_t time;
  uint32_t nanoTime;
  uint8_t syncFlag;
} TimeStampData;

typedef struct GimbalData
{
  float32_t roll;
  float32_t pitch;
  float32_t yaw;
  uint8_t pitchLimit : 1;
  uint8_t rollLimit : 1;
  uint8_t yawLimit : 1;
  uint8_t reserved : 5;
} GimbalData;

typedef uint8_t FlightStatus;

typedef struct TaskData
{
  unsigned char cmdSequence;
  unsigned char cmdData;
} TaskData;

//! @todo rename to a final version
//! RTKData from the A3. This is not available on the M100.
typedef struct RTKData
{
  uint32_t date;
  uint32_t time;
  float64_t longitude;
  float64_t latitude;
  //! @warning the 'Hmsl' field will be renamed in a future release.
  float32_t Hmsl;

  float32_t velocityNorth;
  float32_t velocityEast;
  //! @warning the 'velocityGround' field will be renamed to velocityDown in the next release.
  float32_t velocityGround;
  
  int16_t yaw;
  uint8_t posFlag;
  uint8_t yawFlag;

} RTKData;

//! @todo rename to a final version
//! Detailed GPSData from the A3. This is not available on the M100.
typedef struct GPSData
{
  uint32_t date;
  uint32_t time;
  int32_t longitude;
  int32_t latitude;
  //! @warning the 'Hmsl' field will be renamed in a future release.    
  int32_t Hmsl;

  float32_t velocityNorth;
  float32_t velocityEast;
  //! @warning the 'velocityGround' field will be renamed to velocityDown in the next release.
  float32_t velocityGround;

} GPSData;

#ifndef SDK_DEV
//! @todo
typedef struct BroadcastData
{
  unsigned short dataFlag;
  TimeStampData timeStamp;
  QuaternionData q;
  //! @warning the CommonData type will change to Vector3fData in a future release
  CommonData a;
  VelocityData v;
  //! @warning the CommonData type will change to Vector3fData in a future release
  CommonData w;
  PositionData pos;
  //! @warning the MagnetData type will change to MagData in a future release
  MagnetData mag;
  GPSData gps;
  RTKData rtk;
  //! @warning the RadioData type will change to RCData in a future release
  RadioData rc;
  GimbalData gimbal;
  FlightStatus status; //! @todo define enum
  BatteryData battery;
  CtrlInfoData ctrlInfo;

  //! @note this variable is not set by the FC but populated by the API
  uint8_t activation;
} BroadcastData;
#endif // SDK_DEV

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
  //! @todo channel mapping
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
  char version_ID[11];
  char version_name[32];
  DJI::onboardSDK::Version version;
} VersionData;

#pragma pack()
#ifdef SDK_DEV
#include "devtype.h"
#endif // SDK_DEV
} // namespace onboardSDK
} // namespace DJI

#define PRO_PURE_DATA_MAX_SIZE 1007 // 2^10 - header size
const size_t MMU_TABLE_NUM = 32;


#endif // DJI_TYPE
