#ifndef DJI_PRO_TYPE
#define DJI_PRO_TYPE



#include "DJI_Config.h"

#ifdef API_DEBUG_DATA
#define API_DEBUG(...) printf(__VA_ARGS__)
#else
#define API_DEBUG(format,...) 0
#endif

#ifdef API_ERROR_DATA
#define API_ERROR(...) printf(__VA_ARGS__)
#else
#define API_ERROR(format,...) 0
#endif

#ifdef API_STATUS_DATA
#define API_STATUS(...) printf(__VA_ARGS__)
#else
#define API_STATUS(format,...) 0
#endif


namespace DJI
{
namespace onboardSDK
{
const size_t SESSION_TABLE_NUM = 32;
class API;


typedef struct
{
    unsigned int sof        : 8;

    unsigned int length     : 10;
    unsigned int version    : 6;
    unsigned int session_id : 5;
    unsigned int is_ack     : 1;
    unsigned int reversed0  : 2; // always 0

    unsigned int padding    : 5;
    unsigned int enc_type   : 3;
    unsigned int reversed1  : 24;

    unsigned int sequence_number : 16;
    unsigned int head_crc   : 16;
} Header;

typedef void(*CallBack)(DJI::onboardSDK::API*,Header*);

typedef struct
{
    unsigned short session_mode : 2;
    unsigned short need_encrypt : 1;
    unsigned short retry_time : 13;
    unsigned short ack_timeout;	//unit is ms
    unsigned int length;
    unsigned char *buf;
    CallBack ack_callback;
}Command;


typedef struct
{
    unsigned short reuse_index;
    unsigned short reuse_count;
    unsigned short recv_index;
    unsigned char comm_recv_buf[BUFFER_SIZE];
    // for encrypt
    unsigned char         comm_key[32];
    unsigned char         enc_enabled;
} SDKFilter;

typedef struct MMU_Tab
{
    unsigned int tab_index  : 8;
    unsigned int usage_flag : 8;
    unsigned int mem_size   : 16;
    unsigned char *pmem;
}MMU_Tab;

typedef struct
{
    unsigned int session_id         : 5;
    unsigned int usage_flag         : 1;
    unsigned int sent_time          : 5;
    unsigned int retry_send_time    : 5;
    unsigned int ack_timeout        : 16;
    MMU_Tab *mmu;
    CallBack ack_callback;
    unsigned int pre_seq_num;
    unsigned int pre_timestamp;
}CMDSession;


typedef struct
{
    unsigned int session_id     :5;
    unsigned int session_status :2;
    unsigned int res            :25;
    MMU_Tab *mmu;
}ACKSession;

typedef struct Ack
{
    unsigned short session_id   : 8;
    unsigned short need_encrypt : 8;
    unsigned short seq_num;
    unsigned int length;
    unsigned char *buf;
}Ack;
}//namespace onboardSDK
}//namespace DJI









#define PRO_PURE_DATA_MAX_SIZE			1007   // 2^10 - header size

/* memory management unit */

#define MMU_TABLE_NUM					32


/* session management unit */

#define ACK_SESSION_IDLE				0
#define ACK_SESSION_PROCESS             1
#define ACK_SESSION_USING				2
#define CMD_SESSION_0					0
#define CMD_SESSION_1					1
#define CMD_SESSION_AUTO				32


#endif // DJI_PRO_TYPE

