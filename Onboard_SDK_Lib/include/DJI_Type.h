#ifndef DJI_PRO_TYPE
#define DJI_PRO_TYPE

#include "DJI_Config.h"

#ifdef API_DEBUG_DATA
#define API_DEBUG(...) printf(__VA_ARGS__)
#else
#define API_DEBUG(format, ...) 0
#endif

#ifdef API_ERROR_DATA
#define API_ERROR(...) printf(__VA_ARGS__)
#else
#define API_ERROR(format, ...) 0
#endif

#ifdef API_STATUS_DATA
#define API_STATUS(...) printf(__VA_ARGS__)
#else
#define API_STATUS(format, ...) 0
#endif

namespace DJI
{
namespace onboardSDK
{
const size_t SESSION_TABLE_NUM = 32;
class API;

typedef struct
{
    unsigned int sof : 8;

    unsigned int length : 10;
    unsigned int version : 6;
    unsigned int session_id : 5;
    unsigned int is_ack : 1;
    unsigned int reversed0 : 2; // always 0

    unsigned int padding : 5;
    unsigned int enc_type : 3;
    unsigned int reversed1 : 24;

    unsigned int sequence_number : 16;
    unsigned int head_crc : 16;
} Header;

typedef void (*CallBack)(DJI::onboardSDK::API *, Header *);

typedef struct
{
    unsigned short session_mode : 2;
    unsigned short need_encrypt : 1;
    unsigned short retry_time : 13;
    unsigned short ack_timeout; // unit is ms
    unsigned int length;
    unsigned char *buf;
    CallBack ack_callback;
} Command;

typedef struct
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

typedef struct
{
    unsigned int session_id : 5;
    unsigned int usage_flag : 1;
    unsigned int sent_time : 5;
    unsigned int retry_send_time : 5;
    unsigned int ack_timeout : 16;
    MMU_Tab *mmu;
    CallBack ack_callback;
    unsigned int pre_seq_num;
    unsigned int pre_timestamp;
} CMDSession;

typedef struct
{
    unsigned int session_id : 5;
    unsigned int session_status : 2;
    unsigned int res : 25;
    MMU_Tab *mmu;
} ACKSession;

typedef struct Ack
{
    unsigned short session_id : 8;
    unsigned short need_encrypt : 8;
    unsigned short seq_num;
    unsigned int length;
    unsigned char *buf;
} Ack;

typedef uint8_t BatteryData;
#pragma  pack(1)

typedef struct
{
    signed short yaw_angle;
    signed short roll_angle;
    signed short pitch_angle;
    struct
    {
        unsigned char base              : 1;
        unsigned char yaw_cmd_ignore    : 1;
        unsigned char roll_cmd_ignore   : 1;
        unsigned char pitch_cmd_ignore  : 1;
        unsigned char reserve           : 4;
    }ctrl_byte;
    unsigned char duration;
}GimbalAngleData_t;

typedef struct
{
    signed short yaw_angle_rate;
    signed short roll_angle_rate;
    signed short pitch_angle_rate;
    struct
    {
        unsigned char reserve : 7;
        unsigned char ctrl_switch : 1;//decide increment mode or absolute mode
    }ctrl_byte;
}GimbalSpeedData_t;

typedef float 	float32_t;
typedef double	float64_t;

typedef struct
{
    float32_t q0;
    float32_t q1;
    float32_t q2;
    float32_t q3;
}QuaternionData;

typedef struct
{
    float32_t x;
    float32_t y;
    float32_t z;
}CommonData;

typedef struct
{
    float32_t x;
    float32_t y;
    float32_t z;
    unsigned char health_flag         :1;
    unsigned char feedback_sensor_id  :4;
    unsigned char reserve             :3;
}SpeedData;

typedef struct
{
    float64_t lati;
    float64_t longti;
    float32_t alti;
    float32_t height;
    unsigned char health_flag;
}PositionData_t;

typedef struct
{
    signed short roll;
    signed short pitch;
    signed short yaw;
    signed short throttle;
    signed short mode;
    signed short gear;
}RadioData_t;

typedef struct
{
    signed short x;
    signed short y;
    signed short z;
}api_mag_data_t;

typedef struct
{
    unsigned char cur_ctrl_dev_in_navi_mode   :3;/*0->rc  1->app  2->serial*/
    unsigned char serial_req_status           :1;/*1->opensd  0->close*/
    unsigned char reserved                    :4;
}CtrlInfoData;

typedef struct
{
    unsigned int timeStamp;
    QuaternionData q;
    CommonData a;
    SpeedData v;
    CommonData w;
    PositionData_t pos;
    api_mag_data_t mag;
    RadioData_t rc;
    CommonData gimbal;
    unsigned char status;
    BatteryData capacity;
    CtrlInfoData ctrl_info;
    uint8_t activation;
}BroadcastData;

#pragma  pack()
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

#endif // DJI_PRO_TYPE
