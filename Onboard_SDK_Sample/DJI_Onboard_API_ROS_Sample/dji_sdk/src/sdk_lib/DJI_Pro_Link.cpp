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

static pthread_mutex_t send_lock = PTHREAD_MUTEX_INITIALIZER;
static Session_Queue Send_Session_Common_Tab;
static unsigned char Send_Global_Common_Memory[PKG_MAX_SIZE];
static Session_Queue Send_Session_Tab[SESSION_AND_MEM_COUNT];
static Memory_Manage_Unit Send_MMU_Tab[SESSION_AND_MEM_COUNT];
static unsigned char Send_Global_Memory[SESSION_AND_MEM_COUNT * PKG_MAX_SIZE];

static Ack_Session_Queue Ack_Session_Tab[SESSION_AND_MEM_COUNT + 1];
static Memory_Manage_Unit Ack_MMU_Tab[SESSION_AND_MEM_COUNT + 1];
static unsigned char Ack_Global_Memory[(SESSION_AND_MEM_COUNT + 1) * PKG_MAX_SIZE];

static void Send_Pro_Data(unsigned char *buf)
{
	ProHeader *pHeader = (ProHeader *)buf;
	Pro_Hw_Send(buf,pHeader->length);
}

void Pro_Link_Recv_Hook(ProHeader *header)
{
	unsigned char i;
	ProHeader *p2header;
	Ack_Session_Queue *p2acksession;

	//TODO: parse the protocol data stream here
	if(header->is_ack == 1)
	{
		if(header->session_id == 1)
		{
			printf("%s:Recv Session 1 ACK\n",__func__);
			if(Send_Session_Common_Tab.usage_flag == 1 &&
					Send_Session_Common_Tab.ack_callback)
			{
				Send_Session_Common_Tab.ack_callback(header);
			}
		}
		else if(header->session_id > 1 && header->session_id < 32)
		{
			pthread_mutex_lock(&send_lock);
			for(i = 0 ; i < SESSION_AND_MEM_COUNT ; i ++)
			{
				if(Send_Session_Tab[i].usage_flag == 1)
				{
					p2header = (ProHeader*)Send_Session_Tab[i].mmu->start_addr;
					if(p2header->session_id == header->session_id &&
							p2header->sequence_number == header->sequence_number)
					{
						printf("%s:Recv Session %d ACK\n",__func__,p2header->session_id);
						Send_Session_Tab[i].ack_callback(header);
						Free_Send_Session(&Send_Session_Tab[i]);
						break;
					}
				}
			}
			pthread_mutex_unlock(&send_lock);
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
			p2acksession = Search_Ack_Session(header->session_id);
			if(p2acksession)
			{
				if(p2acksession->session_status == ACK_SESSION_PROCESS)
				{
					printf("%s,This session is waiting for App ack:"
							"session id=%d,seq_num=%d\n",__func__,
							header->session_id,header->sequence_number);
				}
				else if(p2acksession->session_status == ACK_SESSION_IDLE)
				{
					if(header->session_id > 1)
					{
						p2acksession->session_status = ACK_SESSION_PROCESS;
					}
					Pro_Request_Interface(header);
				}
				else if(p2acksession->session_status == ACK_SESSION_USING)
				{
					p2header = (ProHeader *)p2acksession->mmu->start_addr;
					if(p2header->sequence_number == header->sequence_number)
					{
						printf("%s:repeat ACK to remote,session id=%d,seq_num=%d\n",
								__func__,header->session_id,header->sequence_number);
						pthread_mutex_lock(&send_lock);
						Send_Pro_Data((unsigned char*)p2acksession->mmu->start_addr);
						pthread_mutex_unlock(&send_lock);
					}
					else
					{
						printf("%s:same session,but new seq_num pkg,session id=%d,"
								"pre seq_num=%d,""cur seq_num=%d\n",__func__,
								header->session_id,p2header->sequence_number,
								header->sequence_number);
						p2acksession->session_status = ACK_SESSION_PROCESS;
						Pro_Request_Interface(header);
					}
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

	pthread_mutex_lock(&send_lock);

	for(i = 0 ; i < SESSION_AND_MEM_COUNT ; i ++)
	{
		if(Send_Session_Tab[i].usage_flag == 1)
		{
			cur_timestamp = Get_TimeStamp();
			if((cur_timestamp - Send_Session_Tab[i].pre_timestamp)
					> Send_Session_Tab[i].ack_timeout)
			{
				if(Send_Session_Tab[i].retry_send_time > 0 )
				{
					if(Send_Session_Tab[i].sent_time >= Send_Session_Tab[i].retry_send_time )
					{
						Free_Send_Session(&Send_Session_Tab[i]);
					}
					else
					{
						Send_Pro_Data((unsigned char*)Send_Session_Tab[i].mmu->start_addr);
						Send_Session_Tab[i].pre_timestamp = cur_timestamp;
						Send_Session_Tab[i].sent_time ++;
					}
				}
				else
				{
					Send_Pro_Data((unsigned char*)Send_Session_Tab[i].mmu->start_addr);
					Send_Session_Tab[i].pre_timestamp = cur_timestamp;
				}
			}
		}
	}

	pthread_mutex_unlock(&send_lock);
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

Memory_Manage_Unit *Request_Send_MMU(unsigned short size)
{
	unsigned char i;
	Memory_Manage_Unit *p2mmu = NULL;

	for(i = 0 ; i < SESSION_AND_MEM_COUNT; i ++)
	{
		if(Send_MMU_Tab[i].usage_flag == 0)
		{
			p2mmu = &Send_MMU_Tab[i];
			p2mmu->usage_flag = 1;
			p2mmu->end_addr = p2mmu->start_addr + size;
			break;
		}
	}

	return p2mmu;
}

Memory_Manage_Unit *Request_Ack_MMU(unsigned short size)
{
	unsigned char i;
	Memory_Manage_Unit *p2mmu = NULL;

	for(i = 0 ; i < (SESSION_AND_MEM_COUNT + 1); i ++)
	{
		if(Ack_MMU_Tab[i].usage_flag == 0)
		{
			p2mmu = &Ack_MMU_Tab[i];
			p2mmu->usage_flag = 1;
			p2mmu->end_addr = p2mmu->start_addr + size;
			break;
		}
	}

	return p2mmu;
}

void Free_Send_MMU(Memory_Manage_Unit *mmu)
{

	if(mmu->usage_flag == 1)
	{
		mmu->usage_flag = 0;
	}
}

Session_Queue * Request_Send_Session(unsigned short size)
{
	int i;
	Session_Queue *p2session = NULL;
	Memory_Manage_Unit *p2mmu = NULL;

	for(i = 0 ; i < SESSION_AND_MEM_COUNT ; i ++)
	{
		if(Send_Session_Tab[i].usage_flag == 0)
		{
			Send_Session_Tab[i].usage_flag = 1;
			p2session = &Send_Session_Tab[i];
			break;
		}
	}

	p2mmu = Request_Send_MMU(size);
	if(p2mmu == NULL)
	{
		p2session = NULL;
	}
	else
	{
		p2session->mmu = p2mmu;
	}

	return p2session;
}

Ack_Session_Queue * Search_Ack_Session(unsigned char session_id)
{
	unsigned char i;
	for(i = 0 ; i < (SESSION_AND_MEM_COUNT + 1) ; i ++)
	{
		if(Ack_Session_Tab[i].session_id == session_id)
		{
			return &Ack_Session_Tab[i];
		}
	}

	return (Ack_Session_Queue *)NULL;
}

void Free_Send_Session(Session_Queue * session)
{
	if(session->usage_flag == 1)
	{
		Free_Send_MMU(session->mmu);
		session->usage_flag = 0;
	}
}

unsigned int Get_TimeStamp(void)
{
	struct timeval cur_time;
	gettimeofday(&cur_time,NULL);
	return (cur_time.tv_sec * 1000) + (cur_time.tv_usec / 1000);
}

void Pro_Link_Setup(void)
{
	unsigned char i;

	for(i = 0; i < SESSION_AND_MEM_COUNT; i ++)
	{
		Send_Session_Tab[i].session_id = i + 2;
		Send_Session_Tab[i].usage_flag = 0;
		Send_Session_Tab[i].pre_seq_num = 0x10000;

		Send_MMU_Tab[i].mmu_index = i;
		Send_MMU_Tab[i].usage_flag = 0;
		Send_MMU_Tab[i].start_addr = (unsigned long)&Send_Global_Memory[i * PKG_MAX_SIZE];
	}

	Send_Session_Common_Tab.usage_flag = 0;
	Send_Session_Common_Tab.session_id = 1;
	Send_Session_Common_Tab.ack_callback = 0;

	for(i = 0; i < (SESSION_AND_MEM_COUNT + 1); i ++)
	{
		Ack_Session_Tab[i].session_id = i + 1;
		Ack_Session_Tab[i].session_status = ACK_SESSION_IDLE;
		Ack_MMU_Tab[i].mmu_index = i;
		Ack_MMU_Tab[i].usage_flag = 0;
		Ack_MMU_Tab[i].start_addr =  (unsigned long)&Ack_Global_Memory[i * PKG_MAX_SIZE];
	}

	Start_PollThread();
}

void Pro_Config_Comm_Encrypt_Key(const char *key)
{
	sdk_set_encrypt_key_interface(key);
}

int Pro_Ack_Interface(ProAckParameter *parameter)
{
	unsigned short ret = 0;
	Ack_Session_Queue *p2acksession;
	Memory_Manage_Unit *p2mmu;

	if(parameter->session_id == 0)
	{
		;
	}
	else if(parameter->session_id > 0 && parameter->session_id < 32)
	{
		p2acksession = Search_Ack_Session(parameter->session_id);
		if(p2acksession)
		{
			if(parameter->length > PRO_ACK_MAX_SIZE)
			{
				printf("%s:ERROR,ACK buffer is not enough\n",__func__);
				p2acksession->session_status = ACK_SESSION_IDLE;
				return -1;
			}
			p2mmu = Request_Ack_MMU(parameter->length + sizeof(ProHeader) + 4);
			if(p2mmu)
			{
				p2acksession->mmu = p2mmu;

				ret = sdk_encrypt_interface((unsigned char*)p2mmu->start_addr,parameter->buf,
						parameter->length,1,parameter->need_encrypt,
						parameter->session_id,parameter->seq_num);

				if(ret == 0)
				{
					printf("%s:%d:ERROR\n",__func__,__LINE__);
					return -1;
				}

				pthread_mutex_lock(&send_lock);
				Send_Pro_Data((unsigned char *)p2mmu->start_addr);
				pthread_mutex_unlock(&send_lock);

				p2acksession->session_status = ACK_SESSION_USING;
			}
		}
	}

	return 0;
}

int Pro_Send_Interface(ProSendParameter *parameter)
{
	unsigned short ret = 0;
	Session_Queue *p2session = NULL;
	static unsigned short global_seq_num = 0;

	if(parameter->length > PRO_DATA_MAX_SIZE)
	{
		return -1;
	}

	pthread_mutex_lock(&send_lock);

	switch(parameter->pkg_type)
	{
	case 0:
		ret = sdk_encrypt_interface(Send_Global_Common_Memory,parameter->buf,parameter->length,
				0,parameter->need_encrypt,0,global_seq_num ++);
		if(ret == 0)
		{
			printf("%s:%d:ERROR\n",__func__,__LINE__);
			pthread_mutex_unlock(&send_lock);
			return -1;
		}
		Send_Pro_Data(Send_Global_Common_Memory);
		break;
	case 1:
		if(global_seq_num == Send_Session_Common_Tab.pre_seq_num)
		{
			global_seq_num ++;
		}
		Send_Session_Common_Tab.pre_seq_num = global_seq_num;

		ret = sdk_encrypt_interface(Send_Global_Common_Memory,parameter->buf,parameter->length,
				0,parameter->need_encrypt,1,global_seq_num ++);

		if(ret == 0)
		{
			printf("%s:%d:ERROR\n",__func__,__LINE__);
			pthread_mutex_unlock(&send_lock);
			return -1;
		}

		Send_Session_Common_Tab.ack_callback = parameter->ack_callback;
		Send_Session_Common_Tab.ack_timeout = (parameter->ack_timeout > POLL_TICK) ?
												parameter->ack_timeout : POLL_TICK;
		Send_Session_Common_Tab.pre_timestamp = Get_TimeStamp();
		Send_Session_Common_Tab.usage_flag = 1;

		Send_Pro_Data(Send_Global_Common_Memory);
		break;
	case 2:
		p2session = Request_Send_Session(parameter->length + sizeof(ProHeader) + 4);
		if(p2session)
		{
			if(global_seq_num == p2session->pre_seq_num)
			{
				global_seq_num ++;
			}
			p2session->pre_seq_num = global_seq_num;

			ret = sdk_encrypt_interface((unsigned char*)p2session->mmu->start_addr,
					parameter->buf,parameter->length,0,parameter->need_encrypt,
					p2session->session_id,global_seq_num ++);
			if(ret == 0)
			{
				printf("%s:%d:ERROR\n",__func__,__LINE__);
				Free_Send_Session(p2session);
				pthread_mutex_unlock(&send_lock);
				return -1;
			}

			p2session->ack_callback = parameter->ack_callback;
			p2session->ack_timeout = (parameter->ack_timeout > POLL_TICK) ?
										parameter->ack_timeout : POLL_TICK;
			p2session->pre_timestamp = Get_TimeStamp();

			Send_Pro_Data((unsigned char*)p2session->mmu->start_addr);
			p2session->sent_time = 1;
			p2session->retry_send_time = parameter->retry_time;
			ret = 0;
		}
		break;
	}

	pthread_mutex_unlock(&send_lock);

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
		param.buf = buf;
		param.length = sizeof(buf);
		Pro_Ack_Interface(&param);
	}
#endif

	if (app_recv_hook)
		app_recv_hook(header);
}

void Test_ACK_Callback(ProHeader *header)
{
	printf("%s:session id=%d,sq_num=%d\n",__func__,
			header->sequence_number,header->sequence_number);
}

void Test_Pro_Link(void)
{
	unsigned char buf[16];
	ProSendParameter param;

	const char key[32] = {"this is key"};
	//set the communication
	Pro_Config_Comm_Encrypt_Key(key);
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
	param.retry_time = 1;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);
}
