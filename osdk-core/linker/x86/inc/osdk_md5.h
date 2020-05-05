/**
 ********************************************************************
 * @file    osdk_md5.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   This is the header file for "osdk_md5.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2018-2019 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OSDK_MD5_H
#define OSDK_MD5_H

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define MD5_BLOCK_SIZE 16  // MD5 outputs a 16 byte digest

/* Exported types ------------------------------------------------------------*/
typedef unsigned char BYTE;  // 8-bit byte
typedef unsigned int WORD;  // 32-bit word, change to "long" for 16-bit machines

typedef struct {
  BYTE data[64];
  WORD datalen;
  unsigned long long bitlen;
  WORD state[4];
} MD5_CTX;

/* Exported functions --------------------------------------------------------*/
void OsdkMd5_Init(MD5_CTX *ctx);
void OsdkMd5_Update(MD5_CTX *ctx, const BYTE *data, size_t len);
void OsdkMd5_Final(MD5_CTX *ctx, BYTE *hash);

#ifdef __cplusplus
}
#endif

#endif  // OSDK_MD5_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
