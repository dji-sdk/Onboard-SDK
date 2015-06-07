#ifndef __DJI_PRO_CODEC_H__
#define __DJI_PRO_CODEC_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <memory.h>



#define _SDK_MAX_RECV_SIZE          (300)
#define _SDK_SOF                    ((uint8_t)(0xAA))
#define _SDK_CRC_HEAD_SIZE          (2)                 // CRC16
#define _SDK_CRC_DATA_SIZE          (4)                 // CRC32
#define _SDK_HEAD_DATA_LEN          (sizeof(SDKHeader) - 2)
#define _SDK_FULL_DATA_SIZE_MIN     (sizeof(SDKHeader) + _SDK_CRC_DATA_SIZE)

#define _SDK_U32_SET(_addr, _val)   (*((uint32_t*)(_addr)) = (_val))
#define _SDK_U16_SET(_addr, _val)   (*((uint16_t*)(_addr)) = (_val))

#define _SDK_CALC_CRC_HEAD(_msg, _len)   sdk_stream_crc16_calc((const uint8_t*)(_msg), _len)
#define _SDK_CALC_CRC_TAIL(_msg, _len)   sdk_stream_crc32_calc((const uint8_t*)(_msg), _len)


typedef struct
{
	uint32_t sof : 8; // 1byte

	uint32_t length : 10;
	uint32_t version : 6; // 2byte
	uint32_t session_id : 5;
	uint32_t is_ack : 1;
	uint32_t reversed0 : 2; // always 0

	uint32_t padding : 5;
	uint32_t enc_type : 3;
	uint32_t reversed1 : 24;

	uint32_t sequence_number : 16;
	uint32_t head_crc : 16;
	uint32_t magic[0];
} SDKHeader;

typedef struct
{
	uint16_t reuse_index;
	uint16_t reuse_count;
	uint16_t recv_index;
	uint8_t comm_recv_buf[_SDK_MAX_RECV_SIZE];
	// for encrypt
    uint8_t         comm_key[32];
    uint8_t         enc_enabled;
} SDKFilter;


typedef void(*ptr_filter_hook)(SDKHeader* p_head);

void sdk_serial_byte_handle(uint8_t in_data);
void sdk_serial_set_hook(ptr_filter_hook p_hook);

void sdk_set_encrypt_key_interface(const char* sz_key);
unsigned short sdk_encrypt_interface(uint8_t *pdest, const uint8_t *psrc,
		uint16_t w_len,uint8_t is_ack,uint8_t is_enc,uint8_t session_id,uint16_t seq_num);

/*
* Internal functions
*
**/

void sdk_stream_recalc_crc(void* p_data);
uint16_t sdk_stream_crc16_calc(const uint8_t* pMsg, uint32_t nLen);
uint32_t sdk_stream_crc32_calc(const uint8_t* pMsg, uint32_t nLen);
#endif



