#ifndef DJI_TYPE
#define DJI_TYPE

#include "DJI_Config.h"

#ifdef WIN32
#define __func__ __FUNCTION__
#endif
#ifdef QT
#define APIprintf qDebug
#include <QDebug>
#else
#define APIprintf printf
#endif

#ifdef API_DEBUG_DATA
#define API_DEBUG(format, ...)                                                 \
    APIprintf("DEBUG %s,line %d: " format, __func__, __LINE__, ##__VA_ARGS__)
#else
#define API_DEBUG(format, ...) 0
#endif

#ifdef API_ERROR_DATA
#define API_ERROR(format, ...)                                                 \
    APIprintf("Error %s,line %d: " format, __func__, __LINE__, ##__VA_ARGS__)
#else
#define API_ERROR(format, ...) 0
#endif

#ifdef API_STATUS_DATA
#define API_STATUS(format, ...)                                                \
    APIprintf("Status %s,line %d: " format, __func__, __LINE__, ##__VA_ARGS__)
#else
#define API_STATUS(format, ...) 0
#endif

namespace DJI
{
namespace onboardSDK
{
const size_t SESSION_TABLE_NUM = 32;
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

typedef void (*CallBack)(DJI::onboardSDK::CoreAPI *, Header *);

typedef struct Command
{
    unsigned short session_mode : 2;
    unsigned short need_encrypt : 1;
    unsigned short retry_time : 13;
    unsigned short timeout; // unit is ms
    unsigned int length;
    unsigned char *buf;
    CallBack callback;
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
    CallBack callback;
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
    uint8_t  *buf;
} Ack;

#pragma pack(1)
typedef uint8_t BatteryData;

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

typedef struct
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

typedef struct
{
    float64_t lati;
    float64_t longti;
    float32_t alti;
    float32_t height;
    uint8_t health_flag;
} PossitionData;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t throttle;
    int16_t mode;
    int16_t gear;
} RadioData;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} MagnetData;

typedef struct CtrlInfoData
{
    //! @todo mode remote to enums
#ifdef SDK_VERSION_3_0
    uint8_t data;
#endif
    uint8_t cur_ctrl_dev_in_navi_mode : 3; /*0->rc  1->app  2->serial*/
    uint8_t serial_req_status : 1;		 /*1->opensd  0->close*/
    uint8_t reserved : 4;
} CtrlInfoData;

#ifdef SDK_VERSION_2_3
typedef uint32_t TimeStampData;
#endif //SDK_VERSION_2_3

#ifdef SDK_VERSION_3_0
typedef struct TimeStampData
{
    uint32_t time;
    uint32_t asr_ts;
    uint8_t sync_flag;
} TimeStampData;
#endif //SDK_VERSION_3_0

typedef struct GimbalData
{
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
#ifdef SDK_VERSION_3_0
    uint8_t is_pitch_limit : 1;
    uint8_t is_roll_limit : 1;
    uint8_t is_yaw_limit : 1;
    uint8_t reserved : 5;
#endif //SDK_VERSION_3_0
} GimbalData;

typedef uint8_t FlightStatus;

typedef struct BroadcastData
{
    TimeStampData timeStamp;
    QuaternionData q;
    CommonData a;
    VelocityData v;
    CommonData w;
    PossitionData pos;
    MagnetData mag;
    RadioData rc;
    GimbalData gimbal;
    FlightStatus status;//! @todo define enum
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

/* memory management unit */

const size_t MMU_TABLE_NUM = 32;

/* session management unit */

#define ACK_SESSION_IDLE 0
#define ACK_SESSION_PROCESS 1
#define ACK_SESSION_USING 2
#define CMD_SESSION_0 0
#define CMD_SESSION_1 1
#define CMD_SESSION_AUTO 32

#endif // DJI_TYPE
