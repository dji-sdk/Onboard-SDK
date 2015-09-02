/*
 * DJI_Pro_Link.c
 *
 *  Created on: Mar 12, 2015
 *      Author: wuyuwei
 */

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "DJI_Pro_Link.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Codec.h"

#include "DJI_Pro_Rmu.h"

static pthread_mutex_t sync_lock = PTHREAD_MUTEX_INITIALIZER;
static ACK_Callback_Func Call_APP_Func = 0;

static void Send_Pro_Data(unsigned char *buf)
{
	ProHeader *pHeader = (ProHeader *)buf;
    Pro_Hw_Send(buf,pHeader->length);
}

static inline void Get_Sync_Lock(void)
{
	pthread_mutex_lock(&sync_lock);
}

static inline void Free_Sync_Lock(void)
{
	pthread_mutex_unlock(&sync_lock);
}

void Pro_Link_Recv_Hook(ProHeader *header)
{
	ProHeader *p2header;
	static ACK_Session_Tab * ack_session = Get_ACK_Session_Tab();
	static CMD_Session_Tab * cmd_session = Get_CMD_Session_Tab();
	//TODO: parse the protocol data stream here
	if(header->is_ack == 1)
	{
		if(header->session_id == 1)
		{
			printf("%s:Recv Session 1 ACK\n",__func__);
			if(cmd_session[1].usage_flag == 1 && cmd_session[1].ack_callback)
			{
				cmd_session[1].ack_callback(header);
				Get_Memory_Lock();
				Free_CMD_Session(&cmd_session[1]);
				Free_Memory_Lock();
			}
		}
		else if(header->session_id > 1 && header->session_id < 32)
		{
			Get_Sync_Lock();
			if(cmd_session[header->session_id].usage_flag == 1)
			{
				Get_Memory_Lock();
				p2header = (ProHeader*)cmd_session[header->session_id].mmu->pmem;
				if(p2header->session_id == header->session_id &&
						p2header->sequence_number == header->sequence_number)
				{
					printf("%s:Recv Session %d ACK\n",__func__,p2header->session_id);
					Call_APP_Func = cmd_session[header->session_id].ack_callback;
					Free_CMD_Session(&cmd_session[header->session_id]);
					Free_Memory_Lock();
					if(Call_APP_Func)
					{
						Call_APP_Func(header);
					}
				}
				else
				{
					Free_Memory_Lock();
				}
			}
			Free_Sync_Lock();
		}
	}
	else
	{
		//TODO,is a request package
		switch(header->session_id)
		{
		case 0:
			Pro_Request_Interface(header);
			break;
		case 1:
		default:
			if(ack_session[header->session_id - 1].session_status == ACK_SESSION_PROCESS)
			{
				printf("%s,This session is waiting for App ack:"
						"session id=%d,seq_num=%d\n",__func__,
						header->session_id,header->sequence_number);
			}
			else if(ack_session[header->session_id - 1].session_status == ACK_SESSION_IDLE)
			{
				if(header->session_id > 1)
				{
					ack_session[header->session_id - 1].session_status = ACK_SESSION_PROCESS;
				}
				Pro_Request_Interface(header);
			}
			else if(ack_session[header->session_id - 1].session_status == ACK_SESSION_USING)
			{
				Get_Memory_Lock();
				p2header = (ProHeader *)ack_session[header->session_id - 1].mmu->pmem;
				if(p2header->sequence_number == header->sequence_number)
				{
					printf("%s:repeat ACK to remote,session id=%d,seq_num=%d\n",
								__func__,header->session_id,header->sequence_number);
					Get_Sync_Lock();
					Send_Pro_Data(ack_session[header->session_id - 1].mmu->pmem);
					Free_Sync_Lock();
					Free_Memory_Lock();
				}
				else
				{
					printf("%s:same session,but new seq_num pkg,session id=%d,"
							"pre seq_num=%d,""cur seq_num=%d\n",__func__,
							header->session_id,p2header->sequence_number,
							header->sequence_number);
					ack_session[header->session_id - 1].session_status = ACK_SESSION_PROCESS;
					Free_Memory_Lock();
					Pro_Request_Interface(header);
				}
			}
			break;
		}
	}
}

static void Send_Poll(void)
{
	unsigned char i;
	unsigned int cur_timestamp;
	static CMD_Session_Tab * cmd_session = Get_CMD_Session_Tab();
	Get_Sync_Lock();
	for(i = 1 ; i < SESSION_TABLE_NUM ; i ++)
	{
		if(cmd_session[i].usage_flag == 1)
		{
			cur_timestamp = Get_TimeStamp();
			if((cur_timestamp - cmd_session[i].pre_timestamp)
					> cmd_session[i].ack_timeout)
			{
				Get_Memory_Lock();
				if(cmd_session[i].retry_send_time > 0 )
				{
					if(cmd_session[i].sent_time >= cmd_session[i].retry_send_time )
					{
						Free_CMD_Session(&cmd_session[i]);
					}
					else
					{
						Send_Pro_Data(cmd_session[i].mmu->pmem);
						cmd_session[i].pre_timestamp = cur_timestamp;
						cmd_session[i].sent_time ++;
					}
				}
				else
				{
					Send_Pro_Data(cmd_session[i].mmu->pmem);
					cmd_session[i].pre_timestamp = cur_timestamp;
				}
				Free_Memory_Lock();
			}

		}

	}
	Free_Sync_Lock();
}

static void * PollThread(void * arg)
{
	while(1)
	{
		Send_Poll();
		usleep(POLL_TICK * 1000);
	}
	return NULL;
}

static int Start_PollThread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0,PollThread,NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

unsigned int Get_TimeStamp(void)
{
	struct timeval cur_time;
	gettimeofday(&cur_time,NULL);
	return (cur_time.tv_sec * 1000) + (cur_time.tv_usec / 1000);
}

void Pro_Link_Setup(void)
{
	DJI_Pro_Rmu_Setup();
	Start_PollThread();
}

void Pro_Config_Comm_Encrypt_Key(const char *key)
{
	sdk_set_encrypt_key_interface(key);
}

static unsigned short Pro_Calc_Length(unsigned short size, unsigned short encrypt_flag)
{
	unsigned short len;
	if(encrypt_flag)
	{
		len = size + sizeof(ProHeader) + 4 +  (16 - size % 16);
	}
	else
	{
		len = size + sizeof(ProHeader) + 4;
	}
	return len;
}

int Pro_Ack_Interface(ProAckParameter *parameter)
{
	unsigned short ret = 0;
	ACK_Session_Tab * ack_session = (ACK_Session_Tab *)NULL;;

	if(parameter->length > PRO_PURE_DATA_MAX_SIZE)
	{
		printf("%s:%d:ERROR,length=%d is oversize\n",__func__,__LINE__,parameter->length);
		return -1;
	}

	if(parameter->session_id == 0)
	{
		;
	}
	else if(parameter->session_id > 0 && parameter->session_id < 32)
	{
		Get_Memory_Lock();
		ack_session = Request_ACK_Session(parameter->session_id,
				Pro_Calc_Length(parameter->length,parameter->need_encrypt));
		if(ack_session == (ACK_Session_Tab*)NULL)
		{
			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);
			Free_Memory_Lock();
			return -1;
		}

		ret = sdk_encrypt_interface(ack_session->mmu->pmem,parameter->buf,
					parameter->length,1,parameter->need_encrypt,
					parameter->session_id,parameter->seq_num);
		if(ret == 0)
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);
			Free_Memory_Lock();
			return -1;
		}

		Get_Sync_Lock();
		Send_Pro_Data(ack_session->mmu->pmem);
		Free_Sync_Lock();
		Free_Memory_Lock();
		ack_session->session_status = ACK_SESSION_USING;
		return 0;
	}

	return -1;
}

int Pro_Send_Interface(ProSendParameter *parameter)
{
	unsigned short ret = 0;
	CMD_Session_Tab * cmd_session = (CMD_Session_Tab *) NULL;
	static unsigned short global_seq_num = 0;

	if(parameter->length > PRO_PURE_DATA_MAX_SIZE)
	{
		printf("%s:%d:ERROR,length=%d is oversize\n",__func__,__LINE__,parameter->length);
		return -1;
	}

	Get_Sync_Lock();
	switch(parameter->pkg_type)
	{
	case 0:
		Get_Memory_Lock();
		cmd_session = Request_CMD_Session(CMD_SESSION_0,Pro_Calc_Length(parameter->length,parameter->need_encrypt));
		if(cmd_session == (CMD_Session_Tab *)NULL)
		{
			Free_Memory_Lock();
			Free_Sync_Lock();
			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);
			return -1;
		}
		ret = sdk_encrypt_interface(cmd_session->mmu->pmem,parameter->buf,parameter->length,
				0,parameter->need_encrypt,cmd_session->session_id,global_seq_num);
		if(ret == 0)
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);
			Free_Memory_Lock();
			Free_Sync_Lock();
			return -1;
		}
		Send_Pro_Data(cmd_session->mmu->pmem);
		global_seq_num ++;
		Free_CMD_Session(cmd_session);
		Free_Memory_Lock();
		break;
	case 1:
		Get_Memory_Lock();
		cmd_session = Request_CMD_Session(CMD_SESSION_1,Pro_Calc_Length(parameter->length,parameter->need_encrypt));
		if(cmd_session == (CMD_Session_Tab *)NULL)
		{
			Free_Memory_Lock();
			Free_Sync_Lock();
			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);
			return -1;
		}
		if(global_seq_num == cmd_session->pre_seq_num)
		{
			global_seq_num ++;
		}
		ret = sdk_encrypt_interface(cmd_session->mmu->pmem,parameter->buf,parameter->length,
				0,parameter->need_encrypt,cmd_session->session_id,global_seq_num);
		if(ret == 0)
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);
			Free_Memory_Lock();
			Free_Sync_Lock();
			return -1;
		}
		cmd_session->pre_seq_num = global_seq_num ++;
		cmd_session->ack_callback = parameter->ack_callback;
		cmd_session->ack_timeout = (parameter->ack_timeout > POLL_TICK) ?
									parameter->ack_timeout : POLL_TICK;

		cmd_session->pre_timestamp = Get_TimeStamp();
		cmd_session->sent_time = 1;
		cmd_session->retry_send_time = 1;

		Send_Pro_Data(cmd_session->mmu->pmem);
		Free_Memory_Lock();
		break;
	case 2:
		Get_Memory_Lock();
		cmd_session = Request_CMD_Session(CMD_SESSION_AUTO,Pro_Calc_Length(parameter->length,parameter->need_encrypt));
		if(cmd_session == (CMD_Session_Tab *)NULL)
		{
			Free_Memory_Lock();
			Free_Sync_Lock();
			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);
			return -1;
		}
		if(global_seq_num == cmd_session->pre_seq_num)
		{
			global_seq_num ++;
		}
		ret = sdk_encrypt_interface(cmd_session->mmu->pmem,parameter->buf,parameter->length,
				0,parameter->need_encrypt,cmd_session->session_id,global_seq_num);
		if(ret == 0)
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);
			Free_Memory_Lock();
			Free_Sync_Lock();
			return -1;
		}
		cmd_session->pre_seq_num = global_seq_num ++;
		cmd_session->ack_callback = parameter->ack_callback;
		cmd_session->ack_timeout = (parameter->ack_timeout > POLL_TICK) ?
									parameter->ack_timeout : POLL_TICK;
		cmd_session->pre_timestamp = Get_TimeStamp();
		cmd_session->sent_time = 1;
		cmd_session->retry_send_time = parameter->retry_time;
		Send_Pro_Data(cmd_session->mmu->pmem);
		Free_Memory_Lock();
		break;
	}
	Free_Sync_Lock();
	return 0;
}

Req_Callback_Func app_recv_hook;
void App_Recv_Set_Hook(Req_Callback_Func p_hook)
{
	app_recv_hook = p_hook;
}

void Pro_Request_Interface(ProHeader *header)
{
	//TODO call app data handler interface here
#if 0
	unsigned char buf[2] = {0x88,0xEE};
	ProAckParameter param;

	printf("%s:Recv request,session id=%d,seq_num=%d\n",
			__func__,header->session_id,header->sequence_number);
	if(header->session_id > 0)
	{
		param.session_id = header->session_id;
		param.seq_num = header->sequence_number;
		param.need_encrypt = header->enc_type;
		param.buf = buf;
		param.length = sizeof(buf);

		Pro_Ack_Interface(&param);
	}

#else
	if (app_recv_hook)
	{
		app_recv_hook(header);
	}
#endif
}

void Test_ACK_Callback(ProHeader *header)
{
	printf("%s:session id=%d,sq_num=%d\n",__func__,
			header->session_id,header->sequence_number);
}

void Test_Pro_Link(void)
{
	unsigned char buf[16];
	ProSendParameter param;
const char key[] = {"0000000000000000000000000000000000000000000000000000000000000000"};

	Pro_Config_Comm_Encrypt_Key(key);
#if 0
	//session 0
	buf[0] = 0x11;
	buf[1] = 0x22;
	param.pkg_type = 0;
	param.length = 2;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);

	//session 1
	buf[0] = 0x33;
	buf[1] = 0x44;
	param.pkg_type = 1;
	param.length = 2;
	param.ack_callback = Test_ACK_Callback;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);
#endif
	//session 2~31
	buf[0] = 0x55;
	buf[1] = 0x66;
	param.pkg_type = 2;
	param.length = 2;
	param.ack_timeout = 1000;  //unit is ms
	param.ack_callback = Test_ACK_Callback;
	param.retry_time = 1;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);

	//session 2~31
	buf[0] = 0x77;
	buf[1] = 0x88;
	param.pkg_type = 2;
	param.length = 2;
	param.ack_timeout = 1000;  //unit is ms
	param.ack_callback = Test_ACK_Callback;
	param.retry_time = 5;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);
}
