/*
 * DJI_Pro_Link.h
 *
 *  Created on: Mar 12, 2015
 *      Author: wuyuwei
 */

#ifndef DJI_PRO_LINK_H_
#define DJI_PRO_LINK_H_

#define DJI_SDK_PRO_VER					0
#define PRO_DATA_MAX_SIZE				512 //TODO: for user define
#define PKG_MAX_SIZE					(PRO_DATA_MAX_SIZE + 16) //data,CRC32,header size
#define PRO_ACK_MAX_SIZE				8	//TODO: for user define
#define ACK_MAX_SIZE					(PRO_DATA_MAX_SIZE + 16)

#define SESSION_AND_MEM_COUNT			30

#define ACK_SESSION_IDLE				0
#define ACK_SESSION_PROCESS				1
#define ACK_SESSION_USING				2

#define POLL_TICK						20  //unit is ms

typedef struct ProHeader
{
	unsigned int sof : 8; 			// 1byte
	unsigned int length : 10;
	unsigned int version : 6; 		// 2byte
	unsigned int session_id : 5;
	unsigned int is_ack : 1;
	unsigned int reversed0 : 2; 	// always 0

	unsigned int padding : 5;
	unsigned int enc_type : 3;
	unsigned int reversed1 : 24;

	unsigned int sequence_number : 16;
	unsigned int head_crc : 16;
	unsigned int magic[0];
}ProHeader;

typedef void (*ACK_Callback_Func)(ProHeader *pHeader);

typedef struct Memory_Manage_Unit
{
	unsigned char mmu_index;
	unsigned char usage_flag;
	unsigned short res;
	unsigned long start_addr;
	unsigned long end_addr;
}Memory_Manage_Unit;

typedef struct Session_Queue
{
	unsigned int session_id : 5;
	unsigned int usage_flag : 1;
	unsigned int sent_time : 5;
	unsigned int retry_send_time : 5;
	unsigned int ack_timeout : 16;
	Memory_Manage_Unit *mmu;
	ACK_Callback_Func ack_callback;
	unsigned int pre_seq_num;
	unsigned int pre_timestamp;
}Session_Queue;

typedef struct Ack_Session_Queue
{
	unsigned int session_id : 5;
	unsigned int session_status:2;
	unsigned int res:25;
	Memory_Manage_Unit *mmu;
}Ack_Session_Queue;

typedef struct ProSendParameter
{
	unsigned short pkg_type : 2;
	unsigned short need_encrypt : 1;
	unsigned short retry_time : 13;
	unsigned short ack_timeout;	//unit is ms
	unsigned int length;
	unsigned char *buf;
	ACK_Callback_Func ack_callback;
}ProSendParameter;

typedef struct ProAckParameter
{
	unsigned short session_id : 8;
	unsigned short need_encrypt : 8;
	unsigned short seq_num;
	unsigned int length;
	unsigned char *buf;
}ProAckParameter;


unsigned int Get_TimeStamp(void);
void Pro_Link_Setup(void);

Session_Queue * Request_Send_Session(unsigned short size);
void Free_Send_Session(Session_Queue * session);
Ack_Session_Queue * Search_Ack_Session(unsigned char session_id);

void Pro_Config_Comm_Encrypt_Key(const char *key);
int Pro_Ack_Interface(ProAckParameter *parameter);
int Pro_Send_Interface(ProSendParameter *parameter);
void Pro_Request_Interface(ProHeader *header);

void Test_Pro_Link(void);
void Pro_Link_Recv_Hook(ProHeader *header);

int  Pro_Send_Interface(ProSendParameter *parameter);
typedef void (*Req_Callback_Func)(ProHeader *pHeader);
void App_Recv_Set_Hook(Req_Callback_Func p_hook);

#endif /* DJI_PRO_LINK_H_ */
