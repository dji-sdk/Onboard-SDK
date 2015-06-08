/*
 * DJI_Pro_App.h
 *
 *  Created on: Mar 17, 2015
 *      Author: Ying Jiahang
 */

#ifndef __DJI_PRO_APP_H__
#define __DJI_PRO_APP_H__
/*
 * cmd_set    cmd_id     data
 * (1byte)   (1byte)    (* byte)
 */
#include <stdint.h>
// cmd_set
#define MY_DEV_ID           0x00

#define MY_ACTIVATION_SET   0x00
#define MY_CTRL_CMD_SET     0x01

// cmd_id
#define API_OPEN_SERIAL         0x00
#define API_CMD_REQUEST         0x01
#define API_CMD_STATUS_REQUEST  0x02
#define API_CTRL_REQUEST        0x03

#define API_MISSION_WP_INFO     0x10
#define API_MISSION_WP_DATA     0x11
#define API_MISSION_WP_CMD      0x12

#define API_MISSION_HP_START    0x20
#define API_MISSION_HP_CMD      0x21

#define API_CTRL_GIMBAL_SPEED   0x1A


#define API_VERSION_QUERY   0x00
#define API_USER_ACTIVATION 0x01
#define API_INFO_QUERY      0x02
#define API_SIM_ECHO        0xFF

// data_type
typedef float   fp32;
typedef double  fp64;

//----------------------------------------------------------------------
// uav std_msgs reciever
//----------------------------------------------------------------------
#define MSG_ENABLE_FLAG_LEN     2

#define ENABLE_MSG_TIME         0x0001
#define ENABLE_MSG_Q            0x0002
#define ENABLE_MSG_A            0x0004
#define ENABLE_MSG_V            0x0008
#define ENABLE_MSG_W            0x0010
#define ENABLE_MSG_POS          0x0020
#define ENABLE_MSG_MAG          0x0040
#define ENABLE_MSG_RC           0x0080
#define ENABLE_MSG_GIMBAL       0x0100
#define ENABLE_MSG_STATUS       0x0200
#define ENABLE_MSG_BATTERY      0x0400
#define ENABLE_MSG_DEVICE       0x0800

#pragma  pack(1)

typedef struct
{
    uint8_t ctrl_flag;
    fp32    roll_or_x;
    fp32    pitch_or_y;
    fp32    thr_z;
    fp32    yaw;
}api_ctrl_without_sensor_data_t;

typedef struct
{
    fp32    q0;
    fp32    q1;
    fp32    q2;
    fp32    q3;
}sdk_4_16B_data_t;

typedef struct
{
    fp32    x;
    fp32    y;
    fp32    z;
}sdk_3_12B_data_t;

typedef struct
{
    fp64    lati;
    fp64    longti;
    fp32    alti;
    fp32    height;
    uint8_t health_flag;
}sdk_4_24B_data_t;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t throttle;
    int16_t mode;
    int16_t gear;
}sdk_5_10B_data_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}sdk_3_6B_data_t;

typedef struct
{
    uint32_t        time_stamp;
    sdk_4_16B_data_t    q;
    sdk_3_12B_data_t    a;
    sdk_3_12B_data_t    v;
    sdk_3_12B_data_t    w;
    sdk_4_24B_data_t    pos;
    sdk_3_6B_data_t     mag;
    sdk_5_10B_data_t    rc;
    sdk_3_12B_data_t    gimbal;
    uint8_t         status;
    uint8_t         battery_remaining_capacity;
    uint8_t         ctrl_device;
}sdk_std_msg_t;

#pragma  pack()

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------
typedef struct
{
    uint16_t    sequence_number;
    uint8_t session_id  : 5;
    uint8_t need_encrypt    : 1;
    uint8_t reserve     : 2;
}req_id_t;

typedef int16_t (*func_cmd_handler)(uint8_t cmd_id,uint8_t* pbuf,uint16_t len, req_id_t req_id);
typedef struct _cmd_tab
{
    uint8_t             cmd_id;
    func_cmd_handler    pf_cmd_handler;
}cmd_handler_table_t;

typedef struct _set_tab
{
    uint8_t                cmd_set;
    cmd_handler_table_t*   p_cmd_handler_table;
}set_handler_table_t;

#define DATA_MAX_SIZE   (1000u)
#define ERR_INDEX       (0xff)
#define EXC_DATA_SIZE   (16u)
#define SET_CMD_SIZE    (2u)
typedef struct
{
    uint8_t     len;
    uint8_t     cmd_set;
    uint8_t     cmd_id;
    uint8_t     data_buf[DATA_MAX_SIZE];
}dji_sdk_data_msg_t;

// app_send_func.
void App_Send_Data(unsigned char flag, uint8_t is_enc, unsigned char  dev_id, unsigned char cmd_id,unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout, int n);
void App_Recv_Req_Data(ProHeader *header);
void App_Send_Ack(req_id_t req_id, uint8_t *ack, int len);
void App_Set_Table(set_handler_table_t* set_tab,cmd_handler_table_t* cmd_tab);

//----------------------------------------------------------------------
// for cmd agency
//----------------------------------------------------------------------

#define REQ_TIME_OUT            0x0000
#define REQ_REFUSE          0x0001
#define CMD_RECIEVE         0x0002
#define STATUS_CMD_EXECUTING        0x0003
#define STATUS_CMD_EXE_FAIL     0x0004
#define STATUS_CMD_EXE_SUCCESS      0x0005

typedef void (*cmd_ack_callback)(uint16_t *ack);

typedef struct
{
    uint8_t         cmd_sequence;
    uint8_t         cmd_data;
}cmd_agency_data_t;

typedef struct
{
    uint8_t         is_send_cmd;
    cmd_agency_data_t   cmd;
    uint16_t        ack_result;
    cmd_ack_callback    ack_callback;
}dji_sdk_cmd_unit;

int CmdStartThread(void);
void App_Complex_Send_Cmd(uint8_t cmd, cmd_ack_callback ack_callback);

//----------------------------------------------------------------------
// for activation
//----------------------------------------------------------------------

#define SDK_ERR_SUCCESS         0x0000
#define SDK_ERR_COMMAND_NOT_SUPPORTED   0xFF00
#define SDK_ERR_NO_AUTHORIZED       0xFF01
#define SDK_ERR_NO_RIGHTS       0xFF02

typedef struct
{
    uint32_t    app_id;
    uint32_t    app_api_level;
    uint32_t    app_ver;
    uint8_t     app_bundle_id[32];
}activation_data_t;

typedef struct
{
    uint16_t    version_ack;
    uint32_t    version_crc;
    char        version_name[32];
}version_query_data_t;

bool is_sys_error(uint16_t ack);

#endif
