/*
 ============================================================================
 Name        : DJI_Pro_Test.cpp
 Author      : Wu Yuwei
 Version     :
 Copyright   : Your copyright notice
 Description :
 ============================================================================
 */

/* SDK */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"
#include "DJI_Pro_Test.h"
/* MATH for_example */
#include <math.h>

/* parameter */
#define C_EARTH (double) 6378137.0
#define C_PI	(double) 3.141592653589793

#define NO_AUTHORITY 8

/* cmd agency ack func */
void cmd_callback_fun(unsigned short *ack);

/* enc_key */
static const char *key;
/* switch mode */
static float ctrl_mode = 1;
/* switch simple task */
static int simple_task_num = -1;
/* req_id for nav closed by app msg */
static req_id_t nav_force_close_req_id = {0};
/* std msg from uav */
static sdk_std_msg_t recv_sdk_std_msgs = {0};

std::string	serial_name;
int		baud_rate;

int		app_id;
int		app_api_level;
int		app_version;
std::string	app_bundle_id;

std::string     enc_key;
// activation
static activation_data_t activation_msg = {14,2,1,""};

static unsigned char battery_remaining_s = 0xFF;
static unsigned char ctrl_device_s = 0xFF;
static unsigned char activation_status_s = 0xFF;
static pthread_mutex_t status_lock_s = PTHREAD_MUTEX_INITIALIZER;

/* table of sdk req data handler */
int16_t sdk_std_msgs_handler(unsigned char cmd_id,unsigned char* pbuf,unsigned short len,req_id_t req_id);
int16_t	nav_force_close_handler(unsigned char cmd_id,unsigned char* pbuf,unsigned short len,req_id_t req_id);

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
 *  sdk_req_data_callback
 */
int16_t nav_force_close_handler(unsigned char cmd_id,unsigned char* pbuf,unsigned short len,req_id_t req_id)
{
	if(len != sizeof(unsigned char))
		return -1;
	unsigned char msg;
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
		memcpy((unsigned char *)&(_data),(unsigned char *)(_buf)+(_datalen), sizeof(_data));\
		_datalen += sizeof(_data);\
	}

int16_t sdk_std_msgs_handler(unsigned char cmd_id,unsigned char* pbuf,unsigned short len,req_id_t req_id)
{
	unsigned short *msg_enable_flag = (unsigned short *)pbuf;
	unsigned short data_len = MSG_ENABLE_FLAG_LEN;

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
		;
	}

	pthread_mutex_lock(&status_lock_s);
	battery_remaining_s = recv_sdk_std_msgs.battery_remaining_capacity;
	ctrl_device_s = recv_sdk_std_msgs.ctrl_device;
	pthread_mutex_unlock(&status_lock_s);

	return 0;
}

/*
 * app_example
 * mode_test
 * test mode 2: vert velocity, hori angle, yaw angular rate
 */
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
		}
		else if(cnt < 10*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x = -2;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 15*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 2;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 20*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = -2;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 25*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0.5; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 30*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = -0.5; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 35*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 90;
		}
		else if(cnt < 40*50)
		{
			send_data.ctrl_flag = 0x0a;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = -90;
		}
		else
		{
			cnt = 0;
		}
		App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (unsigned char*)&send_data, sizeof(send_data), NULL, 0, 0);
	}
}

/*
 * test mode 4: vert velocity, hori velocity, yaw angular rate
 */
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
		}
		else if(cnt < 10*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x = -0.5;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 15*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0.5;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 20*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = -0.5;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 25*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0.5; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 30*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = -0.5; //m/s
			send_data.yaw = 0;
		}
		else if(cnt < 35*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = 90;
		}
		else if(cnt < 40*50)
		{
			send_data.ctrl_flag = 0x48;
			send_data.roll_or_x =  0;
			send_data.pitch_or_y = 0;
			send_data.thr_z = 0; //m/s
			send_data.yaw = -90;
		}
		else
		{
			cnt = 0;
		}
		App_Send_Data(0, 0,MY_CTRL_CMD_SET, API_CTRL_REQUEST, (unsigned char*)&send_data, sizeof(send_data), NULL, 0, 0);
	}

}

/* test cmd agency */
static unsigned char test_cmd_send_flag = 1;
static unsigned char test_cmd_is_resend = 0;

void cmd_callback_test_fun(unsigned short *ack)
{
	char result[6][50]={{"REQ_TIME_OUT"},{"REQ_REFUSE"},{"CMD_RECIEVE"},{"STATUS_CMD_EXECUTING"},{"STATUS_CMD_EXE_FAIL"},{"STATUS_CMD_EXE_SUCCESS"}};
	unsigned short recv_ack = *ack;
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

/*
 * take off -> landing -> take off -> go home
 */
void basic_test_cmd(bool &is_init)
{
	static int cnt;
	unsigned char send_data = 0;
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
/*
 * random test cmd
 */
void random_test_ack_cmd_callback(ProHeader *header)
{
	/*
	*	#define	REQ_TIME_OUT			0x0000
		#define REQ_REFUSE			0x0001
		#define CMD_RECIEVE			0x0002
		#define STATUS_CMD_EXECUTING		0x0003
		#define STATUS_CMD_EXE_FAIL		0x0004
		#define STATUS_CMD_EXE_SUCCESS		0x0005
	*/
	unsigned short ack_data;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
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
	cmd.cmd_data = (unsigned char)(random()%22);
	cmd.cmd_sequence = (unsigned char)(random()%5 +1);
	if(int(random()%2) == 0)
	{
		App_Send_Data(2, 1, MY_CTRL_CMD_SET, API_CMD_REQUEST, (unsigned char*)&cmd, sizeof(cmd), random_test_ack_cmd_callback, 500, 10);
	}
	else
	{
		unsigned char req_status = cmd.cmd_sequence;
		App_Send_Data(2, 1, MY_CTRL_CMD_SET, API_CMD_STATUS_REQUEST, (unsigned char*)&req_status, sizeof(req_status),random_test_ack_cmd_callback, 500, 10);
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
			unsigned char send_data = 4;
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
			unsigned char send_data = 1;
			App_Complex_Send_Cmd(send_data, cmd_callback_fun);
			cnt ++;
		}
		else if(cnt < 50*40*2+50*15)
		{
			//wait landing
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
/*
 *  test activation
 */
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
	unsigned short ack_data;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
	}
	else
	{
		char result[][50]={{"ACTIVATION_SUCCESS"},{"PARAM_ERROR"},{"DATA_ENC_ERROR"},{"NEW_DEVICE_TRY_AGAIN"},{"DJI_APP_TIMEOUT"},{" DJI_APP_NO_INTERNET"},{"SERVER_REFUSED"},{"LEVEL_ERROR"}};
		printf("[ACTIVATION] Activation result: %s \n", *(result+ack_data));

		activation_status_s = (unsigned char)ack_data;

		if(ack_data == 0)
		{
			Pro_Config_Comm_Encrypt_Key(key);
			printf("[ACTIVATION] set key %s\n",key);
		}
	}
}

void Pro_API_Activation(void)
{
	App_Send_Data( 2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION,(unsigned char*)&activation_msg,sizeof(activation_msg), test_activation_ack_cmd_callback, 1000, 1);
	printf("[ACTIVATION] send acticition msg: %d %d %d %d \n", activation_msg.app_id, activation_msg.app_api_level, activation_msg.app_ver ,activation_msg.app_bundle_id[0]);
}

void test_version_query_ack_cmd_callback(ProHeader *header)
{
	version_query_data_t ack;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((unsigned char *)&ack,(unsigned char *)&header->magic, sizeof(unsigned short));
	/* pay attention to memory holes */
	memcpy((unsigned char *)&ack.version_crc,(unsigned char *)&header->magic+sizeof(unsigned short), (header->length - EXC_DATA_SIZE)-sizeof(unsigned short));
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
	unsigned char msg = 1;
	App_Send_Data(2, 0, MY_ACTIVATION_SET, API_VERSION_QUERY, (unsigned char*)&msg,sizeof(msg), test_version_query_ack_cmd_callback, 1000, 0);
	printf("[ACTIVATION] send version_query msg \n");
}

/*
 * ros_callback function
 */
static unsigned char cmd_send_flag = 1;

void cmd_callback_fun(unsigned short *ack)
{
	/*
	*	#define	REQ_TIME_OUT			0x0000
		#define REQ_REFUSE				0x0001
		#define CMD_RECIEVE				0x0002
		#define STATUS_CMD_EXECUTING	0x0003
		#define STATUS_CMD_EXE_FAIL		0x0004
		#define STATUS_CMD_EXE_SUCCESS	0x0005
	*/
	unsigned short ack_data = *ack;

	if( is_sys_error(ack_data))
	{
		printf("[DEBUG] SDK_SYS_ERROR!!! \n");
	}
	else
	{
		printf("[DEBUG] recv_ack %#x \n", ack_data);
		char result[6][50]={{"REQ_TIME_OUT"},{"REQ_REFUSE"},{"CMD_RECIEVE"},{"STATUS_CMD_EXECUTING"},{"STATUS_CMD_EXE_FAIL"},{"STATUS_CMD_EXE_SUCCESS"}};
		printf("random_test Cmd result: %s \n", *(result+ack_data));
	}
	cmd_send_flag = 1;
}

void sdk_ack_nav_open_close_callback(ProHeader *header)
{
	unsigned short ack_data;
	printf("call %s\n",__func__);
	printf("Recv ACK,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
}

void ros_ctrl_data_callback(float x,float y,float z,float w)
{
	api_ctrl_without_sensor_data_t send_data = {0};
	printf("mode %f yaw %f pitch %f roll %f vel %f\n", ctrl_mode, z,x,y,w);

	if(ctrl_mode == 1)
	{
	    send_data.ctrl_flag 	= 0x0a;  	//mode 2
		send_data.roll_or_x 	= y;
		send_data.pitch_or_y 	= x;
		send_data.thr_z 		= w;		//m/s
		send_data.yaw 			= z;
	}
	else if (ctrl_mode == 2)
	{
		send_data.ctrl_flag 	= 0x48; 	//mode 4
		send_data.roll_or_x 	= y;
		send_data.pitch_or_y 	= x;
		send_data.thr_z 		= w; 		//m/s
		send_data.yaw 			= z;
	}
	App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (unsigned char*)&send_data, sizeof(send_data), NULL, 0, 0);
}

void ros_ctrl_mode_callback(int data)
{
	ctrl_mode = (float)data;
	printf("mode %f\n",ctrl_mode);
}

/*
 * timer spin_function 50Hz
 */
void spin_callback(void)
{
	static unsigned int count = 0;

	count++;
	if(count % 50 == 0)
	{
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
		nav_force_close_req_id.reserve = 0;
		unsigned short ack = 0x0001;
		printf("Ack close send %d !!!!!!!!!!! \n", ack);
		App_Send_Ack(nav_force_close_req_id, (unsigned char *)&ack, sizeof(ack));
	}

	if(simple_task_num > -1)
	{
		static bool init_flag;
		static unsigned int time = Get_Time();
		if(Get_Time() - time < 1000)
		{
		}
		else
		{
			init_flag = false;
		}
		time = Get_Time();

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
			Pro_API_Activation();
			simple_task_num = -1;
			break;
			case 6:
			test_version_query();
			simple_task_num = -1;
			break;
		}
	}
}

unsigned int Get_Time(void)
{
	struct timeval cur_time;
	gettimeofday(&cur_time,NULL);
	return (cur_time.tv_sec * 1000) + (cur_time.tv_usec / 1000); //unit is ms
}

static void * Simple_Task_Threadfun(void * arg)
{

	while(1)
	{
		spin_callback();
		usleep(20000);
	}
	return (void *)NULL;
}

int Start_Simple_Task_Thread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0,Simple_Task_Threadfun,NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

void DJI_Onboard_API_Simple_Task(int data)
{
	simple_task_num = data;
}

void DJI_Onboard_API_Status_Query(void)
{
	pthread_mutex_lock(&status_lock_s);
	printf("--  Current status info: --\n");

	if(activation_status_s == 0x0)
		printf("Activation status:[Activation pass]\n");
	else
		printf("Activation status:[unknown]\n");

	if(battery_remaining_s == 0xFF)
		printf("Battery capacity:[invalid]\n");
	else
		printf("Battery capacity:[%d%%]\n",battery_remaining_s);

	if(ctrl_device_s == 0x0)
		printf("Control device:[RC]\n");
	else if(ctrl_device_s == 0x1)
		printf("Control device:[APP]\n");
	else if(ctrl_device_s == 0x2)
		printf("Control device:[third party onboard device]\n");
	else
		printf("Control device:[unknown]\n");

	pthread_mutex_unlock(&status_lock_s);
}

void DJI_Onboard_API_Control(unsigned char arg)
{
	unsigned char send_data = arg;
	if(arg == 1 || arg == 0)
	{
	}
	else
	{
		printf("Parameter [ERROR]\n");
	}
	App_Send_Data(1,1,MY_CTRL_CMD_SET,API_OPEN_SERIAL,(unsigned char*)&send_data,
					sizeof(send_data),sdk_ack_nav_open_close_callback,1000,0);
}

void DJI_Onboard_API_UAV_Control(unsigned char arg)
{
	unsigned char send_data = arg;

	if( !(arg == 1 || arg == 4 || arg == 6) )
	{
		printf("Parameter [ERROR]\n");
	}

	if(cmd_send_flag)
	{
		App_Complex_Send_Cmd(send_data, cmd_callback_fun);
		cmd_send_flag = 0;
	}
	else
	{
		printf("Previous Command not finish [ERROR]\n");
	}
}

void DJI_Onboard_API_Activation(void)
{
	Pro_API_Activation();
}

int DJI_Pro_Test_Setup(void)
{
	int ret;

	activation_msg.app_id = 10086;
	activation_msg.app_api_level = 2;
	activation_msg.app_ver = 1;
	memcpy(activation_msg.app_bundle_id,"1234567890123456789012", 32);
	key = "DJI-DEMO AES256 KEY-lala-haha-MA";

	ret = Pro_Hw_Setup("/dev/ttyUSB0",230400);
	if(ret < 0)
		return ret;
	Pro_Link_Setup();
	App_Recv_Set_Hook(App_Recv_Req_Data);
	App_Set_Table(set_handler_tab, cmd_handler_tab);
	CmdStartThread();
	Start_Simple_Task_Thread();

	return 0;
}

