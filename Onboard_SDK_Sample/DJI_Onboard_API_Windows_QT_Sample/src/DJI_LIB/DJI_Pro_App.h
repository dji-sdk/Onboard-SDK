/*
 * DJI_Pro_App.cpp
 *
 *  Created on: Sep 8, 2015
 *      Author: wuyuwei
 */
#ifndef __DJI_PRO_APP_H__
#define __DJI_PRO_APP_H__

#include <stdint.h>
#include "DJI_Pro_Link.h"

#define MY_DEV_ID               0x00
#define MY_ACTIVATION_SET       0x00
#define MY_CTRL_CMD_SET         0x01
#define MY_BROADCAST_CMD_SET    0x02
// cmd_id
#define API_VER_QUERY	      	0x00
#define API_CTRL_MANAGEMENT     0x00
#define API_OPEN_SERIAL         0x00
#define API_STD_DATA            0x00
#define API_CMD_REQUEST       	0x01
#define API_CMD_STATUS_REQUEST 	0x02
#define API_CTRL_REQUEST      	0x03
#define API_TRANSPARENT_DATA_TO_MOBILE  0xFE
#define API_TRANSPARENT_DATA_TO_OBOARD  0x02
#define API_GIMBAL_CTRL_SPEED_REQUEST   0x1A
#define API_GIMBAL_CTRL_ANGLE_REQUEST   0x1B

#define API_MISSION_WP_INFO   	0x10
#define API_MISSION_WP_DATA   	0x11
#define API_MISSION_WP_CMD    	0x12

#define API_MISSION_HP_START  	0x20
#define API_MISSION_HP_CMD    	0x21

#define API_CTRL_GIMBAL_SPEED 	0x1A

#define API_VERSION_QUERY       0x00
#define API_USER_ACTIVATION     0x01
#define API_INFO_QUERY          0x02
#define	API_SIM_ECHO            0xFF

#define API_CAMERA_SHOT         0x20
#define API_CAMERA_VIDEO_START  0x21
#define API_CAMERA_VIDEO_STOP   0X22

#define HORIZ_ATT               0x00
#define HORIZ_VEL               0x40
#define HORIZ_POS               0x80

#define VERT_VEL                0x00
#define VERT_POS                0x10
#define VERT_TRU                0x20

#define YAW_ANG                 0x00
#define YAW_RATE                0x08

#define HORIZ_GND               0x00
#define HORIZ_BODY              0x02

#define YAW_GND                 0x00
#define YAW_BODY                0x01

#define MAKE_VERSION(a,b,c,d) (((a << 24)&0xff000000) | ((b << 16)&0x00ff0000) | ((c << 8)&0x0000ff00) | (d&0x000000ff))
#define SDK_VERSION           (MAKE_VERSION(2,3,10,0))

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

/*
 *struct of gimbal contorl
 */

typedef struct
{
    signed short yaw_angle;
    signed short roll_angle;
    signed short pitch_angle;
    struct
    {
        unsigned char base : 1;
        unsigned char yaw_cmd_ignore : 1;
        unsigned char roll_cmd_ignore : 1;
        unsigned char pitch_cmd_ignore : 1;
        unsigned char reserve : 4;
    }ctrl_byte;
    unsigned char duration;
}gimbal_custom_control_angle_t;

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
}gimbal_custom_speed_t;

/*
 *struct of api contrl
 */

typedef struct
{
    fp32 q0;
    fp32 q1;
    fp32 q2;
    fp32 q3;
}api_quaternion_data_t;

typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
}api_common_data_t;

/*
 *struct of vellocity data
 */

typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
    unsigned char health_flag         :1;
    unsigned char feedback_sensor_id  :4;
    unsigned char reserve             :3;
}api_vel_data_t;

typedef struct
{
    fp64 lati;
    fp64 longti;
    fp32 alti;
    fp32 height;
    unsigned char health_flag;
}api_pos_data_t;

typedef struct
{
    signed short roll;
    signed short pitch;
    signed short yaw;
    signed short throttle;
    signed short mode;
    signed short gear;
}api_rc_data_t;

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
}api_ctrl_info_data_t;

typedef struct
{
    unsigned int time_stamp;
    api_quaternion_data_t q;
    api_common_data_t a;
    api_vel_data_t v;
    api_common_data_t w;
    api_pos_data_t pos;
    api_mag_data_t mag;
    api_rc_data_t rc;
    api_common_data_t gimbal;
    unsigned char status;
    unsigned char battery_remaining_capacity;
    api_ctrl_info_data_t ctrl_info;
}sdk_std_msg_t;

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

#define DATA_MAX_SIZE 	(1000u)
#define ERR_INDEX       (0xff)
#define EXC_DATA_SIZE	(16u)
#define SET_CMD_SIZE	(2u)

//----------------------------------------------------------------------
// for cmd agency
//----------------------------------------------------------------------
#define	REQ_TIME_OUT                0x0000
#define REQ_REFUSE                  0x0001
#define CMD_RECIEVE                 0x0002
#define STATUS_CMD_EXECUTING		0x0003
#define STATUS_CMD_EXE_FAIL         0x0004
#define STATUS_CMD_EXE_SUCCESS		0x0005

/*
 *struct of cmd agency data
 */

typedef struct
{
    unsigned char cmd_sequence;
    unsigned char cmd_data;
}cmd_agency_data_t;

//----------------------------------------------------------------------
// for activation 
//----------------------------------------------------------------------

/*
 *  code re-construction according onboard api protocol
 */

#define SDK_ERR_SUCCESS                     0x0000
#define SDK_ERR_COMMAND_NOT_SUPPORTED       0xFF00
#define SDK_ERR_NO_AUTHORIZED               0xFF01
#define SDK_ERR_NO_RIGHTS                   0xFF02
#define SDK_ERR_NO_RESPONSE                 0xFFFF

#define SDK_ACTIVATE_SUCCESS                0x0000
#define SDK_ACTIVATE_PARAM_ERROR            0x0001
#define SDK_ACTIVATE_DATA_ENC_ERROR         0x0002
#define SDK_ACTIVATE_NEW_DEVICE             0x0003
#define SDK_ACTIVATE_DJI_APP_NOT_CONNECT    0x0004
#define SDK_ACTIVATE_DIJ_APP_NO_INTERNET    0x0005
#define SDK_ACTIVATE_SERVER_REFUSED         0x0006
#define SDK_ACTIVATE_LEVEL_ERROR            0x0007
#define SDK_ACTIVATE_SDK_VERSION_ERROR      0x0008

#define PARSE_STD_MSG(_flag, _enable, _data, _buf, _datalen)\
    if((_flag & _enable))\
    {\
        memcpy((unsigned char *)&(_data),(unsigned char *)(_buf)+(_datalen), sizeof(_data));\
        _datalen += sizeof(_data);\
    }

#pragma  pack(1)

/*
 *struct of activate data
 */

typedef struct
{
    unsigned int	app_id;
    unsigned int	app_api_level;
    unsigned int	app_ver;
    unsigned char	app_bundle_id[32];
    char *app_key;
}activate_data_t;

/*
 *struct of version query data
 */

typedef struct
{
    unsigned short	version_ack;
    unsigned int	version_crc;
    char     	version_name[32];
}version_query_data_t;

/*
 *struct of attitude data
 */

typedef struct
{
    unsigned char ctrl_flag;
    float 	roll_or_x;
    float	pitch_or_y;
    float	thr_z;
    float	yaw;
}attitude_data_t;

#pragma  pack()

typedef void (*Command_Result_Notify)(unsigned short result);
typedef void (*Get_API_Version_Notify)(version_query_data_t *);
typedef void (*User_Handler_Func)(ProHeader *pHeader);
typedef void (*Transparent_Transmission_Func)(unsigned char *buf,unsigned char len);


void DJI_Pro_App_Send_Data(unsigned char session_mode, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,
                   unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout, int retry_time);
void DJI_Pro_App_Send_Ack(req_id_t req_id, unsigned char *ack, int len);

int DJI_Pro_Status_Ctrl(unsigned char cmd,Command_Result_Notify user_notice_entrance);
int DJI_Pro_Get_API_Version(Get_API_Version_Notify user_notice_entrance);
int DJI_Pro_Activate_API(activate_data_t *p_user_data,
                         Command_Result_Notify user_notice_entrance);
int DJI_Pro_Send_To_Mobile_Device(unsigned char *data,unsigned char len,
                                  Command_Result_Notify user_notice_entrance);
int DJI_Pro_Control_Management(unsigned char cmd,
                               Command_Result_Notify user_notice_entrance);
int DJI_Pro_Attitude_Control(attitude_data_t *p_user_data);
int DJI_Pro_Gimbal_Angle_Control(gimbal_custom_control_angle_t *p_user_data);
int DJI_Pro_Gimbal_Speed_Control(gimbal_custom_speed_t *p_user_data);
int DJI_Pro_Camera_Control(unsigned char camera_cmd);
int DJI_Pro_Get_Broadcast_Data(sdk_std_msg_t *p_user_buf);
unsigned char DJI_Pro_Get_CmdSet_Id(ProHeader *header);
unsigned char DJI_Pro_Get_CmdCode_Id(ProHeader *header);
int DJI_Pro_Get_Bat_Capacity(unsigned char *data);
int DJI_Pro_Get_Quaternion(api_quaternion_data_t *p_user_buf);
int DJI_Pro_Get_GroundAcc(api_common_data_t *p_user_buf);
int DJI_Pro_Get_GroundVo(api_vel_data_t *p_user_buf);
int DJI_Pro_Get_CtrlInfo(api_ctrl_info_data_t *p_user_buf);
//TODO...
int DJI_Pro_Register_Transparent_Transmission_Callback(Transparent_Transmission_Func user_rec_handler_entrance);
int DJI_Pro_Setup(User_Handler_Func user_cmd_handler_entrance);

#endif
