/*
 ============================================================================
 Name        : dji_sdk_node.c
 Author      : Ying Jiahang, Wu Yuwei
 Version     :
 Copyright   : Your copyright notice
 Description : 
 ============================================================================
 */
/* ROS */
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Quaternion.h"
/* SDK */ 
#include <stdio.h>
#include <stdlib.h>
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"

/* MATH for_example */
#include <math.h>

/* parameter */
#define C_EARTH (double) 6378137.0
#define C_PI	(double) 3.141592653589793

#define NO_AUTHORITY 8

using namespace ros;

/* cmd agency ack func. */
void cmd_callback_fun(uint16_t *ack);

/* ros sub from serial */
ros::Subscriber cmd_data_sub,nav_open_close_sub, ctrl_mode_sub, ctrl_data_sub, simple_task_sub, activation_sub;
/* ros pub for webserver */
ros::Publisher battery_pub, nav_ctrl_status_pub, flight_status_pub, activation_status_pub, test_fre_pub;
/* ros timer */
ros::Timer simple_task_timer;

/* enc_key */
static char *key;
/* switch mode */
static float ctrl_mode = 1;
/* switch simple task */
static int simple_task_num = -1;
/* req_id for nav closed by app msg */
static req_id_t nav_force_close_req_id = {0};
/* std msg from uav */
static sdk_std_msg_t recv_sdk_std_msgs = {0};

/* ros launch param */
std::string	serial_name;
int		baud_rate;

int		app_id;
int		app_api_level;
int		app_version;
std::string	app_bundle_id;

std::string     enc_key;
/* activation */
static activation_data_t activation_msg = {14,2,1,""};

/*
  * table of sdk req data handler
  */
int16_t sdk_std_msgs_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id);
int16_t	nav_force_close_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id);
/* cmd id table */
cmd_handler_table_t cmd_handler_tab[] = 
{
	{0x00,sdk_std_msgs_handler				},
	{0x01,nav_force_close_handler			},
	{ERR_INDEX,NULL							}
};
/* cmd set table */
set_handler_table_t set_handler_tab[] =
{
	{0x02,cmd_handler_tab					},
	{ERR_INDEX,NULL							}
};

/*
  * sdk_req_data_callback
  */
int16_t nav_force_close_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id)
{
	if(len != sizeof(uint8_t))
		return -1;
	uint8_t msg;
	memcpy(&msg, pbuf, sizeof(msg));
	/* test session ack */
	nav_force_close_req_id.sequence_number = req_id.sequence_number;
	nav_force_close_req_id.session_id      = req_id.session_id;
	nav_force_close_req_id.reserve	       = 1;

	printf("WARNING nav close by app %d !!!!!!!!!!!!!! \n", msg);
	return 0;

}

#define _recv_std_msgs(_flag, _enable, _data, _buf, _datalen) \
	if( (_flag & _enable))\
	{\
		memcpy((uint8_t *)&(_data),(uint8_t *)(_buf)+(_datalen), sizeof(_data));\
		_datalen += sizeof(_data);\
	}

int16_t sdk_std_msgs_handler(uint8_t cmd_id,uint8_t* pbuf,uint16_t len,req_id_t req_id)
{
	uint16_t *msg_enable_flag = (uint16_t *)pbuf;
	uint16_t data_len = MSG_ENABLE_FLAG_LEN;
	
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_TIME	, recv_sdk_std_msgs.time_stamp			, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_Q		, recv_sdk_std_msgs.q				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_A		, recv_sdk_std_msgs.a				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_V		, recv_sdk_std_msgs.v				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_W		, recv_sdk_std_msgs.w				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_POS	, recv_sdk_std_msgs.pos				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_MAG	, recv_sdk_std_msgs.mag				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_RC		, recv_sdk_std_msgs.rc				, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_GIMBAL	, recv_sdk_std_msgs.gimbal			, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_STATUS	, recv_sdk_std_msgs.status			, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_BATTERY	, recv_sdk_std_msgs.battery_remaining_capacity	, pbuf, data_len);
	_recv_std_msgs( *msg_enable_flag, ENABLE_MSG_DEVICE	, recv_sdk_std_msgs.ctrl_device			, pbuf, data_len);

	/* testing reciever frequence */
	if( (*msg_enable_flag & ENABLE_MSG_DEVICE))
	{
		std_msgs::Float32 msg;
		msg.data = (float)recv_sdk_std_msgs.ctrl_device;
		test_fre_pub.publish(msg);
		
	}

	return 0;
}

/*
  * app_example 
  */
/* mode_test */
/* test mode 2: vert velocity, hori angle, yaw angular rate */
void basic_test_mode2(bool &is_init)
{
	static int cnt;
	if(!is_init)
	{
		cnt = 0;
		is_init= true;
	}
	else
	{
		api_ctrl_without_sensor_data_t send_data = {0};
		cnt++;
		if(cnt < 5*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x = 2;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 10*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x = -2;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 15*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 2;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 20*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = -2;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 25*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0.5; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 30*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = -0.5; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 35*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 90;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 40*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = -90;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else
		{
			cnt = 0;
		}
		App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
	}
}
// test mode 4: vert velocity, hori velocity, yaw angular rate
void basic_test_mode4(bool &is_init)
{
	static int cnt;
	if(!is_init)
	{
		cnt = 0;
		is_init= true;
	}
	else
	{
		api_ctrl_without_sensor_data_t send_data = {0};
		cnt++;
		if(cnt < 5*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x = 0.5;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 10*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x = -0.5;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 15*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0.5;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 20*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = -0.5;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 25*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0.5; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 30*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = -0.5; //m/s
			send_data.yaw = 0;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 35*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 90;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else if(cnt < 40*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = -90;
			printf("f %#x r_x %4.1f p_y %4.1f t_z %4.1f yaw %4.1f \n",
				send_data.ctrl_flag,
				send_data.roll_or_x,
				send_data.pitch_or_y,
				send_data.thr_z,
				send_data.yaw);
		}
		else
		{
			cnt = 0;
		}
		App_Send_Data(0, 0,MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
	}

}

/* test cmd agency */
static uint8_t test_cmd_send_flag = 1;
static uint8_t test_cmd_is_resend = 0;
void cmd_callback_test_fun(uint16_t *ack)
{
	char result[6][50]={{"REQ_TIME_OUT"},{"REQ_REFUSE"},{"CMD_RECIEVE"},{"STATUS_CMD_EXECUTING"},{"STATUS_CMD_EXE_FAIL"},{"STATUS_CMD_EXE_SUCCESS"}};
	uint16_t recv_ack = *ack;
	printf("[DEBUG] recv_ack %#x \n", recv_ack);
	printf("[TEST_CMD] Cmd result: %s \n", *(result+recv_ack));
	test_cmd_send_flag = 1;
	if(recv_ack != STATUS_CMD_EXE_SUCCESS)
	{
		test_cmd_is_resend = 1;
	}

	/* for debug */
	if(recv_ack != STATUS_CMD_EXE_SUCCESS)
	{
		test_cmd_send_flag  = 0;
		printf("[ERROR] APP LAYER NOT STATUS_CMD_EXE_SUCCESS !!!!!!!!!!!!!!!!!!\n");
	}	

} 

/* take off -> landing -> take off -> go home */
void basic_test_cmd(bool &is_init)
{
	static int cnt;
	uint8_t send_data = 0;
	if(!is_init)
	{
		cnt = 0;
		is_init= true;
	}
	else
	{
		if(test_cmd_send_flag)
		{	
			if(test_cmd_is_resend)
			{
				cnt--;
				test_cmd_is_resend = 0;
			}
			if(cnt % 4 == 0)
			{
				test_cmd_send_flag = 0;
				send_data = 4;
			}
			else if(cnt % 4 == 1)
			{
				test_cmd_send_flag = 0;
				send_data = 6;
			}
			else if(cnt % 4 == 2)
			{
				test_cmd_send_flag = 0;
				send_data = 4;
			}
			else if(cnt % 4 == 3)
			{
				test_cmd_send_flag = 0;
				send_data = 1;
			}
			App_Complex_Send_Cmd(send_data, cmd_callback_test_fun);
			printf("[TEST_CMD] send %d \n",send_data);
			cnt++;
		}
	}
}

/* random test cmd */
void random_test_ack_cmd_callback(ProHeader *header)
{
	/*
		#define	REQ_TIME_OUT					0x0000
		#define REQ_REFUSE					0x0001
		#define CMD_RECIEVE					0x0002
		#define STATUS_CMD_EXECUTING		0x0003
		#define STATUS_CMD_EXE_FAIL			0x0004
		#define STATUS_CMD_EXE_SUCCESS		0x0005
	*/
	uint16_t ack_data;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((uint8_t *)&ack_data,(uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
		std_msgs::Float32 msg;
		msg.data = NO_AUTHORITY;
		activation_status_pub.publish(msg);
	}
	else
	{
		char result[6][50]={{"REQ_TIME_OUT"},{"REQ_REFUSE"},{"CMD_RECIEVE"},{"STATUS_CMD_EXECUTING"},{"STATUS_CMD_EXE_FAIL"},{"STATUS_CMD_EXE_SUCCESS"}};
		printf("random_test Cmd result: %s \n", *(result+ack_data));
	}
}

void random_test_cmd(bool &is_init)
{
	cmd_agency_data_t cmd;
	cmd.cmd_data = uint8_t(random()%22);
	cmd.cmd_sequence = uint8_t(random()%5 +1);
	if(int(random()%2) == 0)
	{
		App_Send_Data(2, 1, MY_CTRL_CMD_SET, API_CMD_REQUEST, (uint8_t*)&cmd, sizeof(cmd), random_test_ack_cmd_callback, 500, 10);
	}
	else
	{
		uint8_t req_status = cmd.cmd_sequence;
		App_Send_Data(2, 1, MY_CTRL_CMD_SET, API_CMD_STATUS_REQUEST, (uint8_t*)&req_status, sizeof(req_status),random_test_ack_cmd_callback, 500, 10);
	}
}

void test_all(bool &is_init)
{
	static int cnt;
	if(!is_init)
	{
		cnt = 0;
		is_init= true;
	}
	else
	{
		static bool init_flag = false;
		if(cnt < 1)
		{
			uint8_t send_data = 4; 
			App_Complex_Send_Cmd(send_data, cmd_callback_fun);
			cnt ++;
		}
		else if(cnt < 50*12)
		{
			/* wait for takeoff finish */
			cnt ++;
		}
		else if(cnt < 50*40)
		{
			basic_test_mode2(init_flag);
			cnt ++;
		}
		else if(cnt < 50*40+1)
		{
			init_flag = false;
			cnt ++;
		}
		else if(cnt < 50*40*2)
		{
			basic_test_mode4(init_flag);
			cnt ++;
		}
		else if(cnt < 50*40*2+1)
		{
			init_flag = false;
			uint8_t send_data = 1; 
			App_Complex_Send_Cmd(send_data, cmd_callback_fun);
			cnt ++;
		}
		else if(cnt < 50*40*2+50*15)
		{
			/*wait for landing */
			cnt ++;
		}
		else
		{
			printf("Test_all finished !\n");
			cnt = 0;
			simple_task_num = -1;
		}
				
	}
}

/* test activation */
void test_activation_ack_cmd_callback(ProHeader *header)
{
	/*
		#define	ACTIVATION_SUCCESS		0x0000
		#define PARAM_ERROR				0x0001
		#define DATA_ENC_ERROR			0x0002
		#define NEW_DEVICE_TRY_AGAIN	0x0003
		#define DJI_APP_TIMEOUT			0x0004
		#define DJI_APP_NO_INTERNET		0x0005
		#define SERVER_REFUSED			0x0006
		#define LEVEL_ERROR				0x0007
	*/ 
	uint16_t ack_data;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((uint8_t *)&ack_data,(uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
		std_msgs::Float32 msg;
		msg.data = NO_AUTHORITY;
		activation_status_pub.publish(msg);
	}
	else
	{
		char result[][50]={{"ACTIVATION_SUCCESS"},{"PARAM_ERROR"},{"DATA_ENC_ERROR"},{"NEW_DEVICE_TRY_AGAIN"},{"DJI_APP_TIMEOUT"},{" DJI_APP_NO_INTERNET"},{"SERVER_REFUSED"},{"LEVEL_ERROR"}};
		printf("[ACTIVATION] Activation result: %s \n", *(result+ack_data));
		std_msgs::Float32 msg;
		msg.data = (float)ack_data;
		activation_status_pub.publish(msg);

		if(ack_data == 0)
		{
			Pro_Config_Comm_Encrypt_Key(key);
			printf("[ACTIVATION] set key %s\n",key);
		}
	}
}

void test_activation(void)
{
	/*	msg.app_id 	= 0;
		msg.app_api_level 	= 1;
		msg.app_ver = 2;
		msg.app_bundle_id[0] = 4;
	*/
	App_Send_Data( 2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION,(uint8_t*)&activation_msg,sizeof(activation_msg), test_activation_ack_cmd_callback, 1000, 1);
	printf("[ACTIVATION] send acticition msg: %d %d %d %d \n", activation_msg.app_id, activation_msg.app_api_level, activation_msg.app_ver ,activation_msg.app_bundle_id[0]);
}

void test_version_query_ack_cmd_callback(ProHeader *header)
{
	version_query_data_t ack;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((uint8_t *)&ack,(uint8_t *)&header->magic, sizeof(uint16_t));
	/* pay attention to memory holes */
	memcpy((uint8_t *)&ack.version_crc,(uint8_t *)&header->magic+sizeof(uint16_t), (header->length - EXC_DATA_SIZE)-sizeof(uint16_t));
	if(ack.version_ack == SDK_ERR_SUCCESS)
	{	
		printf("[ACTIVATION] Activation result:\n 	ack SDK_ACT_SUCCESS\n");
	}
	else
	{
		printf("[ACTIVATION] Activation result:\n 	ack SDK_NO_AUTHORIZED\n");
	}
	printf(" 	version_crc %X\n 	version_name %s \n", ack.version_crc, ack.version_name);
}

void test_version_query(void)
{
	uint8_t msg = 1;
	App_Send_Data(2, 0, MY_ACTIVATION_SET, API_VERSION_QUERY, (uint8_t*)&msg,sizeof(msg), test_version_query_ack_cmd_callback, 1000, 0);
	printf("[ACTIVATION] send version_query msg \n");
}

/*
  * ros_callback function
  */
static uint8_t cmd_send_flag = 1;
void cmd_callback_fun(uint16_t *ack)
{
	/*
		#define	REQ_TIME_OUT				0x0000
		#define REQ_REFUSE				0x0001
		#define CMD_RECIEVE				0x0002
		#define STATUS_CMD_EXECUTING	0x0003
		#define STATUS_CMD_EXE_FAIL		0x0004
		#define STATUS_CMD_EXE_SUCCESS	0x0005
	*/
	uint16_t ack_data = *ack;

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
		std_msgs::Float32 msg;
		msg.data = NO_AUTHORITY;
		activation_status_pub.publish(msg);
	}
	else
	{
		printf("[DEBUG] recv_ack %#x \n", ack_data);
		char result[6][50]={{"REQ_TIME_OUT"},{"REQ_REFUSE"},{"CMD_RECIEVE"},{"STATUS_CMD_EXECUTING"},{"STATUS_CMD_EXE_FAIL"},{"STATUS_CMD_EXE_SUCCESS"}};
		printf("random_test Cmd result: %s \n", *(result+ack_data));
	}
	cmd_send_flag = 1;
} 

void ros_cmd_data_callback(const std_msgs::Float32::ConstPtr& msg)
{
	uint8_t send_data = (uint8_t)msg->data;
	printf("cmd %d\n", send_data);
	if( send_data > 21)
		return;

	if(cmd_send_flag)
	{
		App_Complex_Send_Cmd(send_data, cmd_callback_fun);
		cmd_send_flag = 0;
	}
	else
	{
		printf("[CMD] wating! \n");
	}
}
void sdk_ack_nav_open_close_callback(ProHeader *header)
{
	uint16_t ack_data;
	printf("call %s\n",__func__);
	printf("Recv ACK,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((uint8_t *)&ack_data,(uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

	std_msgs::Float32 msg;
	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
		msg.data = NO_AUTHORITY;
		activation_status_pub.publish(msg);
	}
	else
	{
		msg.data = (float)ack_data;
		nav_ctrl_status_pub.publish(msg);
	}
}

void ros_nav_open_close_callback(const std_msgs::Float32::ConstPtr& msg)
{
	uint8_t send_data = (uint8_t)msg->data;
	printf("send open nav %d\n",send_data);
	App_Send_Data(1, 1, MY_CTRL_CMD_SET, API_OPEN_SERIAL, (uint8_t*)&send_data, sizeof(send_data), sdk_ack_nav_open_close_callback,  1000, 0);
}

void ros_ctrl_data_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
{	
	api_ctrl_without_sensor_data_t send_data = {0};
	/*
		send_data.send_yaw = (float)msg->z;
		send_data.send_pitch = (float)msg->x;
		send_data.send_roll = (float)msg->y;
		send_data.send_thr = (float)(msg->w);
	*/
	printf("mode %f yaw %f pitch %f roll %f vel %f\n", ctrl_mode, (float)msg->z, (float)msg->x, (float)msg->y, (float)(msg->w));

	if(ctrl_mode == 1)
	{
	    send_data.ctrl_flag 	= 0x0a; 	// mode 2
		send_data.roll_or_x 	= msg->y;
		send_data.pitch_or_y 	= msg->x;
		send_data.thr_z 	= msg->w; 		//m/s
		send_data.yaw 		= msg->z;
	}
	else if (ctrl_mode == 2)
	{
		send_data.ctrl_flag 	= 0x48; 	// mode 4
		send_data.roll_or_x 	= msg->y;
		send_data.pitch_or_y 	= msg->x;
		send_data.thr_z 	= msg->w; 		//m/s
		send_data.yaw 		= msg->z;
	}
	App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&send_data, sizeof(send_data), NULL, 0, 0);
} 

void ros_ctrl_mode_callback(const std_msgs::Float32::ConstPtr& msg)
{
	ctrl_mode = (float)msg->data;
	printf("mode %f\n",ctrl_mode);
}

void ros_simple_task_callback(const std_msgs::Float32::ConstPtr& msg)
{
	simple_task_num = (int)msg->data;
}
void ros_activation_callback(const std_msgs::Float32::ConstPtr& msg)
{
	printf("ros_activation_callback %f \n",msg->data);
	test_activation();
}

/*
  * timer spin_function 50Hz
  */
void spin_callback(const ros::TimerEvent& e)
{
	static unsigned int count = 0;

	count++;
	if(count % 50 == 0)
	{
		std_msgs::Float32 msg;

		msg.data = (float)recv_sdk_std_msgs.status;
		flight_status_pub.publish(msg);

		msg.data = (float)recv_sdk_std_msgs.battery_remaining_capacity;
		battery_pub.publish(msg);
/*		
		ROS_INFO("STD_MSGS:");
		printf("[STD_MSGS] time_stamp %d \n",recv_sdk_std_msgs.time_stamp);
		printf("[STD_MSGS] q %f %f %f %f \n",recv_sdk_std_msgs.q.q0,recv_sdk_std_msgs.q.q1,recv_sdk_std_msgs.q.q2,recv_sdk_std_msgs.q.q3);
		printf("[STD_MSGS] a %f %f %f\n",recv_sdk_std_msgs.a.x,recv_sdk_std_msgs.a.y,recv_sdk_std_msgs.a.z);
		printf("[STD_MSGS] v %f %f %f\n",recv_sdk_std_msgs.v.x,recv_sdk_std_msgs.v.y,recv_sdk_std_msgs.v.z);
		printf("[STD_MSGS] w %f %f %f\n",recv_sdk_std_msgs.w.x,recv_sdk_std_msgs.w.y,recv_sdk_std_msgs.w.z);
		printf("[STD_MSGS] pos %f %f %f %f \n",recv_sdk_std_msgs.pos.lati, recv_sdk_std_msgs.pos.longti, recv_sdk_std_msgs.pos.alti, recv_sdk_std_msgs.pos.height);
		printf("[STD_MSGS] mag %d %d %d \n",recv_sdk_std_msgs.mag.x,recv_sdk_std_msgs.mag.y,recv_sdk_std_msgs.mag.z);
		printf("[STD_MSGS] rc %d %d %d %d %d\n",recv_sdk_std_msgs.rc.roll, recv_sdk_std_msgs.rc.pitch, recv_sdk_std_msgs.rc.yaw, recv_sdk_std_msgs.rc.throttle,recv_sdk_std_msgs.rc.mode);
		printf("[STD_MSGS] gimbal %f %f %f\n",recv_sdk_std_msgs.gimbal.x, recv_sdk_std_msgs.gimbal.y,recv_sdk_std_msgs.gimbal.z);
		printf("[STD_MSGS] status %d\n",recv_sdk_std_msgs.status);
		printf("[STD_MSGS] battery %d\n",recv_sdk_std_msgs.battery_remaining_capacity);
		printf("[STD_MSGS] ctrl_device %d\n",recv_sdk_std_msgs.ctrl_device);
*/
	}

	/* test session ack for force close */
	if(nav_force_close_req_id.reserve == 1)
	{
		std_msgs::Float32 msg2;
		msg2.data = 4;
		nav_ctrl_status_pub.publish(msg2);
		nav_force_close_req_id.reserve = 0;

		uint16_t ack = 0x0001;
		printf("Ack close send %d !!!!!!!!!!! \n", ack);
		App_Send_Ack(nav_force_close_req_id, (uint8_t *)&ack, sizeof(ack));
	}

	if(simple_task_num > -1)
	{
		static bool init_flag;
		static ros::Time time = ros::Time::now();
		if(ros::Time::now().toSec()- time.toSec() < 1)
		{
			time = ros::Time::now();
		}
		else
		{
			init_flag = false;
			time = ros::Time::now();
		}
		switch(simple_task_num)
		{
			case 0:
			basic_test_mode2(init_flag);
			break;	
			case 1:
			basic_test_mode4(init_flag);
			break;	
			case 2:
			basic_test_cmd(init_flag);
			break;
			case 3:
			random_test_cmd(init_flag);
			break;
			case 4:
			test_all(init_flag);
			break;
			case 5:
			test_activation();
			simple_task_num = -1;
			break;
			case 6:
			test_version_query();
			simple_task_num = -1;
			break;
		}
	}
}

/*
  * main_function
  */
int main (int argc, char** argv)
{

	printf("Test SDK Protocol demo\n");
	/* initialize ros */
	ros::init(argc, argv, "SDK_serial");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("serial_name", serial_name, std::string("/dev/ttySAC0"));
	nh_private.param("baud_rate", baud_rate, 230400);
	nh_private.param("app_id", app_id, 14);
	nh_private.param("app_api_level", app_api_level, 2);
	nh_private.param("app_version", app_version, 1);
	nh_private.param("app_bundle_id", app_bundle_id, std::string("12345678901234567890123456789012"));
	nh_private.param("enc_key", enc_key, std::string("9b7c15ee8dc3849976a779b37cdec9fe4c6308af5a03b3a570b8dc0e3c7337b8"));

	activation_msg.app_id 		= (uint32_t)app_id;
	activation_msg.app_api_level 	= (uint32_t)app_api_level;
	activation_msg.app_ver		= (uint32_t)app_version;
	memcpy(activation_msg.app_bundle_id, app_bundle_id.c_str(), 32); 
	
	key = (char*)enc_key.c_str();
	
	printf("[INIT] SET serial_port	: %s \n", serial_name.c_str());
	printf("[INIT] SET baud_rate	: %d \n", baud_rate);
	printf("[INIT] ACTIVATION INFO	: \n");
	printf("[INIT] 	  app_id     	  %d \n", activation_msg.app_id);
	printf("[INIT]    app_api_level	  %d \n", activation_msg.app_api_level);
	printf("[INIT]    app_version     %d \n", activation_msg.app_ver);
	printf("[INIT]    app_bundle_id	  %s \n", activation_msg.app_bundle_id);
	printf("[INIT]    enc_key	  %s \n", key);

	/* start ros subscriber */
	cmd_data_sub 		= nh.subscribe("/sdk_request_cmd", 10, ros_cmd_data_callback);
	nav_open_close_sub      = nh.subscribe("/nav_open_close_request", 10, ros_nav_open_close_callback);
	ctrl_data_sub		= nh.subscribe("/sdk_request_ctrl", 10, ros_ctrl_data_callback);
	ctrl_mode_sub		= nh.subscribe("/sdk_request_ctrl_mode", 10, ros_ctrl_mode_callback);
	simple_task_sub		= nh.subscribe("/sdk_request_simple_task", 10, ros_simple_task_callback);
	activation_sub		= nh.subscribe("/sdk_request_activation", 10, ros_activation_callback);
	/* start ros publisher */
    battery_pub 		= nh.advertise<std_msgs::Float32>("/battery_status", 10);
	nav_ctrl_status_pub 	= nh.advertise<std_msgs::Float32>("/nav_open_close_status", 10);
	flight_status_pub 	= nh.advertise<std_msgs::Float32>("/flight_status", 10);
	activation_status_pub   = nh.advertise<std_msgs::Float32>("/activation_status", 10);
	test_fre_pub		= nh.advertise<std_msgs::Float32>("/test_fre", 10);
	/* ros timer 50Hz */
	simple_task_timer 	= nh.createTimer(ros::Duration(1.0/50.0), spin_callback);
	/* open serial port */
	Pro_Hw_Setup((char *)serial_name.c_str(),baud_rate);
	Pro_Link_Setup();
	App_Recv_Set_Hook(App_Recv_Req_Data);
	App_Set_Table(set_handler_tab, cmd_handler_tab);

	CmdStartThread();

	Pro_Config_Comm_Encrypt_Key(key);
	/* ros spin for timer */
	ros::spin();

	return 0;
}
