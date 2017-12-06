/** @file dji_aes.hpp
 *  @version 3.3
 *  @date April 12th, 2017
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef ONBOARDSDK_AES256_H
#define ONBOARDSDK_AES256_H

#include <stdint.h>

typedef struct tagAES256Context
{
  uint8_t key[32];
  uint8_t enckey[32];
  uint8_t deckey[32];
} aes256_context;

typedef void (*ptr_aes256_codec)(aes256_context* ctx, uint8_t* buf);

uint8_t rj_xtime(uint8_t x);
void aes_subBytes(uint8_t* buf);
void aes_subBytes_inv(uint8_t* buf);
void aes_addRoundKey(uint8_t* buf, uint8_t* key);
void aes_addRoundKey_cpy(uint8_t* buf, uint8_t* key, uint8_t* cpk);
void aes_shiftRows(uint8_t* buf);
void aes_shiftRows_inv(uint8_t* buf);
void aes_mixColumns(uint8_t* buf);
void aes_mixColumns_inv(uint8_t* buf);
void aes_expandEncKey(uint8_t* k, uint8_t* rc);
void aes_expandDecKey(uint8_t* k, uint8_t* rc);
void aes256_init(aes256_context* ctx, uint8_t* k);
void aes256_done(aes256_context* ctx);
void aes256_encrypt_ecb(aes256_context* ctx, uint8_t* buf);
void aes256_decrypt_ecb(aes256_context* ctx, uint8_t* buf);

#endif // ONBOARDSDK_AES256_H
