/**
 ********************************************************************
 * @file    psdk_md5.h
 * @version V2.0.0
 * @date    2019/07/01
 * @brief   This is the header file for "psdk_md5.c", defining the structure and
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
 * crypto-algorithms
 * =================
 *
 * About
 * ---
 * These are basic implementations of standard cryptography algorithms, written by Brad Conte (brad@bradconte.com) from
 * scratch and without any cross-licensing. They exist to provide publically accessible, restriction-free implementations
 * of popular cryptographic algorithms, like AES and SHA-1. These are primarily intended for educational and pragmatic
 * purposes (such as comparing a specification to actual implementation code, or for building an internal application
 * that computes test vectors for a product). The algorithms have been tested against standard test vectors.
 * This code is released into the public domain free of any restrictions. The author requests acknowledgement if the code
 * is used, but does not require it. This code is provided free of any liability and without any quality claims by the
 * author.
 * Note that these are *not* cryptographically secure implementations. They have no resistence to side-channel attacks
 * and should not be used in contexts that need cryptographically secure implementations.
 * These algorithms are not optimized for speed or space. They are primarily designed to be easy to read, although some
 * basic optimization techniques have been employed.
 * Building
 * ---
 * The source code for each algorithm will come in a pair of a source code file and a header file. There should be no
 * inter-header file dependencies, no additional libraries, no platform-specific header files, or any other complicating
 * matters. Compiling them should be as easy as adding the relevent source code to the project.
 *
 * @statement DJI has modified some symbols' name.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PSDK_MD5_H
#define PSDK_MD5_H

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define MD5_BLOCK_SIZE 16               // MD5 outputs a 16 byte digest

/* Exported types ------------------------------------------------------------*/
typedef unsigned char BYTE;             // 8-bit byte
typedef unsigned int WORD;              // 32-bit word, change to "long" for 16-bit machines

typedef struct {
    BYTE data[64];
    WORD datalen;
    unsigned long long bitlen;
    WORD state[4];
} MD5_CTX;

/* Exported functions --------------------------------------------------------*/
void PsdkMd5_Init(MD5_CTX *ctx);
void PsdkMd5_Update(MD5_CTX *ctx, const BYTE *data, size_t len);
void PsdkMd5_Final(MD5_CTX *ctx, BYTE *hash);

#ifdef __cplusplus
}
#endif

#endif // PSDK_MD5_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
