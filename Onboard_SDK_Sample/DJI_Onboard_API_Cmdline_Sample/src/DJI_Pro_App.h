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
#include "DJI_Pro_Link.h"
// cmd_set
#define MY_DEV_ID         	0x00

#define MY_ACTIVATION_SET	0x00
#define MY_CTRL_CMD_SET		0x01

// cmd_id
#define API_OPEN_SERIAL	      	0x00
#define API_CMD_REQUEST       	0x01
#define API_CMD_STATUS_REQUEST 	0x02
#define API_CTRL_REQUEST      	0x03

#define API_GIMBAL_CTRL_SPEED_REQUEST   0x1A
#define API_GIMBAL_CTRL_ANGLE_REQUEST   0x1B

#define API_MISSION_WP_INFO   	0x10
#define API_MISSION_WP_DATA   	0x11
#define API_MISSION_WP_CMD    	0x12

#define API_MISSION_HP_START  	0x20
#define API_MISSION_HP_CMD    	0x21

#define API_CTRL_GIMBAL_SPEED 	0x1A


#define API_VERSION_QUERY	0x00
#define API_USER_ACTIVATION	0x01
#define API_INFO_QUERY		0x02
#define	API_SIM_ECHO		0xFF

#define MAKE_VERSION(a,b,c,d) (((a << 24)&0xff000000) | ((b << 16)&0x00ff0000) | ((c << 8)&0x0000ff00) | (d&0x000000ff))
#define SDK_VERSION           (MAKE_VERSION(2,3,2,0))

// data_type
typedef float 	fp32;
typedef double	fp64;

//----------------------------------------------------------------------
// uav std_msgs reciever
//----------------------------------------------------------------------
#define MSG_ENABLE_FLAG_LEN		2

#define ENABLE_MSG_TIME			0x0001
#define ENABLE_MSG_Q			0x0002
#define ENABLE_MSG_A			0x0004
#define ENABLE_MSG_V			0x0008
#define ENABLE_MSG_W			0x0010
#define ENABLE_MSG_POS			0x0020
#define ENABLE_MSG_MAG			0x0040
#define ENABLE_MSG_RC			0x0080
#define ENABLE_MSG_GIMBAL		0x0100
#define ENABLE_MSG_STATUS		0x0200
#define ENABLE_MSG_BATTERY		0x0400
#define ENABLE_MSG_DEVICE		0x0800

#pragma  pack(1)

/* ��̨�Ƕȿ��ƽṹ�� */
typedef struct
{
    int16_t yaw_angle;
    int16_t roll_angle;
    int16_t pitch_angle;
    struct
    {
        uint8_t base : 1;
        uint8_t yaw_cmd_ignore : 1;
        uint8_t roll_cmd_ignore : 1;
        uint8_t pitch_cmd_ignore : 1;
        uint8_t reserve : 4;
    }ctrl_byte;
    uint8_t duration;
}gimbal_custom_control_angle_t;

/* ��̨���ٶȿ��ƽṹ�� */
typedef struct
{
    int16_t yaw_angle_rate;
    int16_t roll_angle_rate;
    int16_t pitch_angle_rate;
    struct
    {
        uint8_t reserve : 7;
        uint8_t ctrl_switch : 1;
    }ctrl_byte;
}gimbal_custom_speed_t;

typedef struct
{
	unsigned char ctrl_flag;
	fp32 	roll_or_x;
	fp32	pitch_or_y;
	fp32	thr_z;
	fp32	yaw;
}api_ctrl_without_sensor_data_t;

typedef struct
{
    fp32            q0;
    fp32            q1;
    fp32            q2;
    fp32            q3;
}sdk_4_16B_data_t;

typedef struct
{
    fp32            x;
    fp32            y;
    fp32            z;
}sdk_3_12B_data_t;

typedef struct
{
    fp32            x;
    fp32            y;
    fp32            z;
    uint8_t         health_flag         :1;
    uint8_t         feedback_sensor_id  :4;
    uint8_t         reserve             :3;
}sdk_4_13B_data_t;

typedef struct
{
    fp64            lati;
    fp64            longti;
    fp32            alti;
    fp32            height;
    uint8_t         health_flag;
}sdk_5_25B_data_t;

typedef struct
{
    int16_t         roll;
    int16_t         pitch;
    int16_t         yaw;
    int16_t         throttle;
    int16_t         mode;
    int16_t         gear;
}sdk_5_10B_data_t;

typedef struct
{
    int16_t         x;
    int16_t         y;
    int16_t         z;
}sdk_3_6B_data_t;

typedef struct
{
    uint8_t         cur_ctrl_dev_in_navi_mode   :3;/*0->rc  1->app  2->serial*/
    uint8_t         serial_req_status           :1;/*1->opensd  0->close*/
    uint8_t         reserved                    :4;
}_ctrl_device;



typedef struct
{
    unsigned int		time_stamp;
    sdk_4_16B_data_t	q;
    sdk_3_12B_data_t	a;
    sdk_4_13B_data_t	v;
    sdk_3_12B_data_t	w;
    sdk_5_25B_data_t	pos;
    sdk_3_6B_data_t		mag;
    sdk_5_10B_data_t	rc;
    sdk_3_12B_data_t	gimbal;
    uint8_t             status;
    uint8_t             battery_remaining_capacity;
    _ctrl_device         ctrl_device;
}uplink_data_t,sdk_std_msg_t;

#pragma  pack()

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------
typedef struct 
{
    unsigned short	sequence_number;
    unsigned char	session_id 	: 5;
    unsigned char	need_encrypt	: 1;
    unsigned char	reserve	   	: 2;
}req_id_t;

typedef int16_t (*func_cmd_handler)(unsigned char cmd_id,unsigned char* pbuf,unsigned short len, req_id_t req_id);
typedef struct _cmd_tab
{
    unsigned char             cmd_id;
    func_cmd_handler    pf_cmd_handler;
}cmd_handler_table_t;

typedef struct _set_tab
{
    unsigned char                cmd_set;
    cmd_handler_table_t*   p_cmd_handler_table;
}set_handler_table_t;

#define DATA_MAX_SIZE 	(1000u)
#define ERR_INDEX       (0xff)
#define EXC_DATA_SIZE	(16u)
#define SET_CMD_SIZE	(2u)
typedef struct
{
	unsigned char     len;
    unsigned char     cmd_set;
    unsigned char     cmd_id;
    unsigned char     data_buf[DATA_MAX_SIZE];
}dji_sdk_data_msg_t;

// app_send_func.
void App_Send_Data(unsigned char flag, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout, int n);
void App_Recv_Req_Data(ProHeader *header);
void App_Send_Ack(req_id_t req_id, unsigned char *ack, int len);
void App_Set_Table(set_handler_table_t* set_tab,cmd_handler_table_t* cmd_tab);

//----------------------------------------------------------------------
// for cmd agency
//----------------------------------------------------------------------

#define	REQ_TIME_OUT			0x0000
#define REQ_REFUSE			0x0001
#define CMD_RECIEVE			0x0002
#define STATUS_CMD_EXECUTING		0x0003
#define STATUS_CMD_EXE_FAIL		0x0004
#define STATUS_CMD_EXE_SUCCESS		0x0005

typedef void (*cmd_ack_callback)(unsigned short *ack);

typedef struct
{
	unsigned char			cmd_sequence;
	unsigned char			cmd_data;
}cmd_agency_data_t;

typedef struct
{
	unsigned char 		is_send_cmd;
	cmd_agency_data_t	cmd;
	unsigned short 		ack_result;
	cmd_ack_callback 	ack_callback;
}dji_sdk_cmd_unit;

int CmdStartThread(void);
void App_Complex_Send_Cmd(unsigned char cmd, cmd_ack_callback ack_callback);

//----------------------------------------------------------------------
// for activation 
//----------------------------------------------------------------------

#define SDK_ERR_SUCCESS			0x0000
#define SDK_ERR_COMMAND_NOT_SUPPORTED	0xFF00
#define SDK_ERR_NO_AUTHORIZED		0xFF01
#define SDK_ERR_NO_RIGHTS		0xFF02

typedef struct
{
	unsigned int	app_id;
	unsigned int	app_api_level;
	unsigned int	app_ver;
    unsigned char	app_bundle_id[32];
}activation_data_t;

typedef struct
{
	unsigned short	version_ack;
	unsigned int	version_crc;
	char     	version_name[32];
}version_query_data_t;

bool is_sys_error(unsigned short ack);

#endif
