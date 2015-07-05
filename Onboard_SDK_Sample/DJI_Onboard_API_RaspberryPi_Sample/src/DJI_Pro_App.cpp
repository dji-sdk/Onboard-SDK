/*
 * DJI_Pro_App.cpp
 *
 *  Created on: Mar 17, 2015
 *      Author: Ying Jiahang
 */

#include "DJI_Pro_Link.h"
#include "DJI_Pro_App.h"
#include <string.h>
#include <stdio.h>

#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>

static dji_sdk_data_msg_t recv_data = {0};
static unsigned char Pro_Encode_Data[1024];
static unsigned char Pro_Encode_ACK[10];

set_handler_table_t* app_set_tab_ptr;
cmd_handler_table_t* app_cmd_tab_ptr;

unsigned char find_set_index(unsigned char cmd_set,set_handler_table_t* p_set_handler_tab)
{
    unsigned char item_data=0;
    unsigned char item_index=0;
    if(cmd_set == ERR_INDEX)return ERR_INDEX;
    if(p_set_handler_tab==NULL)return ERR_INDEX;

    while(item_data!=ERR_INDEX)
    {
        item_data = p_set_handler_tab[item_index].cmd_set;
        if(item_data == cmd_set)
        {
            return item_index;
        }
        item_index++;
    }
    return ERR_INDEX;
}

unsigned char find_cmd_index(unsigned char cmd_id,cmd_handler_table_t* p_cmd_handler_tab)
{
    unsigned short item_data=0;
    unsigned short item_index=0;
    if(cmd_id == ERR_INDEX) return ERR_INDEX;
    if(p_cmd_handler_tab==NULL) return ERR_INDEX;
    while(item_data!=ERR_INDEX)
    {
        item_data = p_cmd_handler_tab[item_index].cmd_id;
        if(item_data == cmd_id)
        {
            return item_index;
        }
        item_index++;
    }
    return ERR_INDEX;
}

void App_Set_Table(set_handler_table_t* set_tab,cmd_handler_table_t* cmd_tab)
{
	app_set_tab_ptr = set_tab;
	app_cmd_tab_ptr = cmd_tab;
}

void App_Send_Data(unsigned char flag, unsigned char is_enc, unsigned char  cmd_set, unsigned char cmd_id,unsigned char *pdata,int len,ACK_Callback_Func ack_callback, int timeout ,int n)
{
	ProSendParameter param;
	unsigned char *ptemp = (unsigned char *)Pro_Encode_Data;
	*ptemp++ = cmd_set;
	*ptemp++ = cmd_id;

	memcpy(Pro_Encode_Data + SET_CMD_SIZE,pdata,len);

	param.ack_callback = ack_callback;
	param.pkg_type = flag;
	param.length = len + SET_CMD_SIZE;
	param.buf = Pro_Encode_Data;
	param.retry_time = n;

	param.ack_timeout = timeout; 
	param.need_encrypt = is_enc;
	
	Pro_Send_Interface(&param);
}

void App_Recv_Req_Data(ProHeader *header)
{
	req_id_t recv_req_id;
	recv_req_id.sequence_number = header->sequence_number;
	recv_req_id.session_id 	    = header->session_id;
	recv_req_id.need_encrypt    = header->enc_type & 0xff;
	recv_data.len 		    = header->length - EXC_DATA_SIZE;

	memcpy((unsigned char *)&recv_data.cmd_set,(unsigned char *)&header->magic, recv_data.len);

	unsigned char set_index = 0;
	unsigned char cmd_index = 0;

	set_index = find_set_index(recv_data.cmd_set,app_set_tab_ptr);
	cmd_index = find_cmd_index(recv_data.cmd_id,app_set_tab_ptr[set_index].p_cmd_handler_table);

	if(set_index == ERR_INDEX || cmd_index == ERR_INDEX)
	{
		printf("ERROR, NO SUCH CMD_SET(%d) OR CMD_ID(%d) !\n", set_index, cmd_index);
		return;
	}
	app_set_tab_ptr[set_index].p_cmd_handler_table[cmd_index].pf_cmd_handler(recv_data.cmd_id,
	                                                                         recv_data.data_buf,
	                                                                         (recv_data.len - SET_CMD_SIZE),
	                                                                         recv_req_id);
}

void App_Send_Ack(req_id_t req_id, unsigned char *ack, int len)
{
	ProAckParameter param;

	memcpy(Pro_Encode_ACK,ack,len);
	
	param.session_id = req_id.session_id;
	param.seq_num = req_id.sequence_number;
	param.need_encrypt = req_id.need_encrypt;
	param.buf = Pro_Encode_ACK;
	param.length = len;
	Pro_Ack_Interface(&param);
}

//----------------------------------------------------------------------
// cmd agency
//----------------------------------------------------------------------

static dji_sdk_cmd_unit cmd_unit = {0};

void sdk_ack_cmd_callback(ProHeader *header)
{
	/*
	 *	ack_data  0 -> null  1 -> recieve request but cmd can't exec  2 -> cmd exec
	 */
	unsigned short ack_data;
	printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number, header->session_id, header->length - EXC_DATA_SIZE);
	memcpy((unsigned char *)&ack_data,(unsigned char *)&header->magic, (header->length - EXC_DATA_SIZE));
	cmd_unit.ack_result = ack_data;
}

static void * CmdRecvThread(void * arg)
{
	while(1)
	{
		if(cmd_unit.is_send_cmd)
		{
			printf("[DEBUG] in send\n");
			cmd_unit.is_send_cmd = 0;
			App_Send_Data(2,1,MY_CTRL_CMD_SET, API_CMD_REQUEST,(unsigned char*)&cmd_unit.cmd,sizeof(cmd_unit.cmd),sdk_ack_cmd_callback, 10, 0);
			printf("[DEBUG] send req cmd ok\n");
			sleep(2);
			if( (cmd_unit.ack_result&0xFF00) == 0xFF00 )
			{
				cmd_unit.ack_callback(&cmd_unit.ack_result);
				continue;
			}
			else if(cmd_unit.ack_result == REQ_TIME_OUT)
			{
				printf("[DEBUG] recv ack cmd time out \n");
				cmd_unit.ack_callback(&cmd_unit.ack_result);
				continue;
			}
			else if(cmd_unit.ack_result == REQ_REFUSE)
			{
				cmd_unit.ack_callback(&cmd_unit.ack_result);
				continue;
			}
			else if (cmd_unit.ack_result == CMD_RECIEVE)
			{
				printf("[DEBUG] CMD_RECIEVE \n");

				unsigned char req_status = cmd_unit.cmd.cmd_sequence; // can be anything
				sleep(7);
				App_Send_Data(2, 1,MY_CTRL_CMD_SET, API_CMD_STATUS_REQUEST,(unsigned char*)&req_status,sizeof(unsigned char),sdk_ack_cmd_callback, 10, 0);
				printf("[DEBUG] send req status ok\n");
				sleep(1);
				printf("[DEBUG] recv ack1 status ok\n");
				if( is_sys_error(cmd_unit.ack_result))
				{
					printf("SDK_SYS_ERROR!!! \n");
					continue;
				}
				else
				{
					cmd_unit.ack_callback(&cmd_unit.ack_result);
				}
				
				// for debug
				if(cmd_unit.ack_result != STATUS_CMD_EXE_SUCCESS)
				{
					printf("[DEBUG] WARNING CMD UN-SECCUSS\n");
				}
				
			}
		}
		else
		{
			usleep(100000);
		}
	}
	return NULL;
}

int CmdStartThread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0,CmdRecvThread,NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

void App_Complex_Send_Cmd(unsigned char cmd, cmd_ack_callback ack_callback)
{
	cmd_unit.cmd.cmd_data = cmd;
	cmd_unit.is_send_cmd = 1;
	cmd_unit.ack_callback = ack_callback;
	cmd_unit.ack_result = 0x0000;
	cmd_unit.cmd.cmd_sequence ++;
}

//----------------------------------------------------------------------
// for activation
//----------------------------------------------------------------------
bool is_sys_error(unsigned short ack_data)
{
	if( ack_data == SDK_ERR_COMMAND_NOT_SUPPORTED)
	{
		printf("[SDK_SYS] SDK_ERR_COMMAND_NOT_SUPPORTED \n");
	}
	else if( ack_data == SDK_ERR_NO_AUTHORIZED)
	{
		printf("[SDK_SYS] SDK_ERR_NO_AUTHORIZED \n");
	}
	else if( ack_data == SDK_ERR_NO_RIGHTS)
	{
		printf("[SDK_SYS] SDK_ERR_NO_RIGHTS \n");
	}

	if( (ack_data&0xFF00) == 0xFF00)
		return true;
	else
		return false;
}
