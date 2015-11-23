#ifndef __DJI_PRO_CODEC_H__
#define __DJI_PRO_CODEC_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <memory.h>

#define _SDK_MAX_RECV_SIZE          (1024)
#define _SDK_SOF                    ((unsigned char)(0xAA))
#define _SDK_CRC_HEAD_SIZE          (2)                 // CRC16
#define _SDK_CRC_DATA_SIZE          (4)                 // CRC32
#define _SDK_HEAD_DATA_LEN          (sizeof(Header) - 2)
#define _SDK_FULL_DATA_SIZE_MIN     (sizeof(Header) + _SDK_CRC_DATA_SIZE)

#define _SDK_U32_SET(_addr, _val)   (*((unsigned int*)(_addr)) = (_val))
#define _SDK_U16_SET(_addr, _val)   (*((unsigned short*)(_addr)) = (_val))

#define _SDK_CALC_CRC_HEAD(_msg, _len)   sdk_stream_crc16_calc((const unsigned char*)(_msg), _len)
#define _SDK_CALC_CRC_TAIL(_msg, _len)   sdk_stream_crc32_calc((const unsigned char*)(_msg), _len)

#include "DJI_Type.h"

void transformTwoByte(const char *pstr,unsigned char *pdata);
/*
* Internal functions
*
**/
#endif
