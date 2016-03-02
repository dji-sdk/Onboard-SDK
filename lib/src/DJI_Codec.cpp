#include "DJI_Codec.h"
#include "DJI_Link.h"
#include "DJI_API.h"

//////////////////////////////////////////////////////////////////////////
// BEGIN OF AES-256
//
/*
*   Byte-oriented AES-256 implementation.
*   All lookup tables replaced with 'on the fly' calculations.
*
*   Copyright (c) 2007-2009 Ilya O. Levin, http://www.literatecode.com
*   Other contributors: Hal Finney
*
*   Permission to use, copy, modify, and distribute this software for any
*   purpose with or without fee is hereby granted, provided that the above
*   copyright notice and this permission notice appear in all copies.
*
*   THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
*   WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
*   ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
*   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
*   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
*   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
typedef struct tagAES256Context
{
    unsigned char key[32];
    unsigned char enckey[32];
    unsigned char deckey[32];
} aes256_context;

#define F(x) (((x) << 1) ^ ((((x) >> 7) & 1) * 0x1b))
#define FD(x) (((x) >> 1) ^ (((x)&1) ? 0x8d : 0))

#define BACK_TO_TABLES
#ifdef BACK_TO_TABLES

const unsigned char sbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab,
    0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4,
    0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71,
    0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
    0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6,
    0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb,
    0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45,
    0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44,
    0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a,
    0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49,
    0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
    0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25,
    0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e,
    0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1,
    0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb,
    0x16
};
const unsigned char sboxinv[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7,
    0xfb, 0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde,
    0xe9, 0xcb, 0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42,
    0xfa, 0xc3, 0x4e, 0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49,
    0x6d, 0x8b, 0xd1, 0x25, 0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c,
    0xcc, 0x5d, 0x65, 0xb6, 0x92, 0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15,
    0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84, 0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7,
    0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06, 0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02,
    0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b, 0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc,
    0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73, 0x96, 0xac, 0x74, 0x22, 0xe7, 0xad,
    0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e, 0x47, 0xf1, 0x1a, 0x71, 0x1d,
    0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b, 0xfc, 0x56, 0x3e, 0x4b,
    0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4, 0x1f, 0xdd, 0xa8,
    0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f, 0x60, 0x51,
    0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef, 0xa0,
    0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c,
    0x7d
};

#define rj_sbox(x) sbox[(x)]
#define rj_sbox_inv(x) sboxinv[(x)]

#else /* tableless subroutines */

/* -------------------------------------------------------------------------- */
unsigned char gf_alog(unsigned char x) // calculate anti-logarithm gen 3
{
    unsigned char atb = 1, z;

    while (x--)
    {
        z = atb;
        atb <<= 1;
        if (z & 0x80)
            atb ^= 0x1b;
        atb ^= z;
    }

    return atb;
} /* gf_alog */

/* -------------------------------------------------------------------------- */
unsigned char gf_log(unsigned char x) // calculate logarithm gen 3
{
    unsigned char atb = 1, i = 0, z;

    do
    {
        if (atb == x)
            break;
        z = atb;
        atb <<= 1;
        if (z & 0x80)
            atb ^= 0x1b;
        atb ^= z;
    } while (++i > 0);

    return i;
} /* gf_log */

/* -------------------------------------------------------------------------- */
unsigned char gf_mulinv(unsigned char x) // calculate multiplicative inverse
{
    return (x) ? gf_alog(255 - gf_log(x)) : 0;
} /* gf_mulinv */

/* -------------------------------------------------------------------------- */
unsigned char rj_sbox(unsigned char x)
{
    unsigned char y, sb;

    sb = y = gf_mulinv(x);
    y = (y << 1) | (y >> 7);
    sb ^= y;
    y = (y << 1) | (y >> 7);
    sb ^= y;
    y = (y << 1) | (y >> 7);
    sb ^= y;
    y = (y << 1) | (y >> 7);
    sb ^= y;

    return (sb ^ 0x63);
} /* rj_sbox */

/* -------------------------------------------------------------------------- */
unsigned char rj_sbox_inv(unsigned char x)
{
    unsigned char y, sb;

    y = x ^ 0x63;
    sb = y = (y << 1) | (y >> 7);
    y = (y << 2) | (y >> 6);
    sb ^= y;
    y = (y << 3) | (y >> 5);
    sb ^= y;

    return gf_mulinv(sb);
} /* rj_sbox_inv */

#endif

/* -------------------------------------------------------------------------- */
unsigned char rj_xtime(unsigned char x)
{
    return (x & 0x80) ? ((x << 1) ^ 0x1b) : (x << 1);
} /* rj_xtime */

/* -------------------------------------------------------------------------- */
void aes_subBytes(unsigned char *buf)
{
    register unsigned char i = 16;

    while (i--) buf[i] = rj_sbox(buf[i]);
} /* aes_subBytes */

/* -------------------------------------------------------------------------- */
void aes_subBytes_inv(unsigned char *buf)
{
    register unsigned char i = 16;

    while (i--) buf[i] = rj_sbox_inv(buf[i]);
} /* aes_subBytes_inv */

/* -------------------------------------------------------------------------- */
void aes_addRoundKey(unsigned char *buf, unsigned char *key)
{
    register unsigned char i = 16;

    while (i--) buf[i] ^= key[i];
} /* aes_addRoundKey */

/* -------------------------------------------------------------------------- */
void aes_addRoundKey_cpy(unsigned char *buf, unsigned char *key, unsigned char *cpk)
{
    register unsigned char i = 16;

    while (i--) buf[i] ^= (cpk[i] = key[i]), cpk[16 + i] = key[16 + i];
} /* aes_addRoundKey_cpy */

/* -------------------------------------------------------------------------- */
void aes_shiftRows(unsigned char *buf)
{
    register unsigned char i, j; /* to make it potentially parallelable :) */

    i = buf[1];
    buf[1] = buf[5];
    buf[5] = buf[9];
    buf[9] = buf[13];
    buf[13] = i;
    i = buf[10];
    buf[10] = buf[2];
    buf[2] = i;
    j = buf[3];
    buf[3] = buf[15];
    buf[15] = buf[11];
    buf[11] = buf[7];
    buf[7] = j;
    j = buf[14];
    buf[14] = buf[6];
    buf[6] = j;

} /* aes_shiftRows */

/* -------------------------------------------------------------------------- */
void aes_shiftRows_inv(unsigned char *buf)
{
    register unsigned char i, j; /* same as above :) */

    i = buf[1];
    buf[1] = buf[13];
    buf[13] = buf[9];
    buf[9] = buf[5];
    buf[5] = i;
    i = buf[2];
    buf[2] = buf[10];
    buf[10] = i;
    j = buf[3];
    buf[3] = buf[7];
    buf[7] = buf[11];
    buf[11] = buf[15];
    buf[15] = j;
    j = buf[6];
    buf[6] = buf[14];
    buf[14] = j;

} /* aes_shiftRows_inv */

/* -------------------------------------------------------------------------- */
void aes_mixColumns(unsigned char *buf)
{
    register unsigned char i, a, b, c, d, e;

    for (i = 0; i < 16; i += 4)
    {
        a = buf[i];
        b = buf[i + 1];
        c = buf[i + 2];
        d = buf[i + 3];
        e = a ^ b ^ c ^ d;
        buf[i] ^= e ^ rj_xtime(a ^ b);
        buf[i + 1] ^= e ^ rj_xtime(b ^ c);
        buf[i + 2] ^= e ^ rj_xtime(c ^ d);
        buf[i + 3] ^= e ^ rj_xtime(d ^ a);
    }
} /* aes_mixColumns */

/* -------------------------------------------------------------------------- */
void aes_mixColumns_inv(unsigned char *buf)
{
    register unsigned char i, a, b, c, d, e, x, y, z;

    for (i = 0; i < 16; i += 4)
    {
        a = buf[i];
        b = buf[i + 1];
        c = buf[i + 2];
        d = buf[i + 3];
        e = a ^ b ^ c ^ d;
        z = rj_xtime(e);
        x = e ^ rj_xtime(rj_xtime(z ^ a ^ c));
        y = e ^ rj_xtime(rj_xtime(z ^ b ^ d));
        buf[i] ^= x ^ rj_xtime(a ^ b);
        buf[i + 1] ^= y ^ rj_xtime(b ^ c);
        buf[i + 2] ^= x ^ rj_xtime(c ^ d);
        buf[i + 3] ^= y ^ rj_xtime(d ^ a);
    }
} /* aes_mixColumns_inv */

/* -------------------------------------------------------------------------- */
void aes_expandEncKey(unsigned char *k, unsigned char *rc)
{
    register unsigned char i;

    k[0] ^= rj_sbox(k[29]) ^ (*rc);
    k[1] ^= rj_sbox(k[30]);
    k[2] ^= rj_sbox(k[31]);
    k[3] ^= rj_sbox(k[28]);
    *rc = F(*rc);

    for (i = 4; i < 16; i += 4)
        k[i] ^= k[i - 4], k[i + 1] ^= k[i - 3], k[i + 2] ^= k[i - 2], k[i + 3] ^= k[i - 1];
    k[16] ^= rj_sbox(k[12]);
    k[17] ^= rj_sbox(k[13]);
    k[18] ^= rj_sbox(k[14]);
    k[19] ^= rj_sbox(k[15]);

    for (i = 20; i < 32; i += 4)
        k[i] ^= k[i - 4], k[i + 1] ^= k[i - 3], k[i + 2] ^= k[i - 2], k[i + 3] ^= k[i - 1];

} /* aes_expandEncKey */

/* -------------------------------------------------------------------------- */
void aes_expandDecKey(unsigned char *k, unsigned char *rc)
{
    unsigned char i;

    for (i = 28; i > 16; i -= 4)
        k[i + 0] ^= k[i - 4], k[i + 1] ^= k[i - 3], k[i + 2] ^= k[i - 2], k[i + 3] ^= k[i - 1];

    k[16] ^= rj_sbox(k[12]);
    k[17] ^= rj_sbox(k[13]);
    k[18] ^= rj_sbox(k[14]);
    k[19] ^= rj_sbox(k[15]);

    for (i = 12; i > 0; i -= 4)
        k[i + 0] ^= k[i - 4], k[i + 1] ^= k[i - 3], k[i + 2] ^= k[i - 2], k[i + 3] ^= k[i - 1];

    *rc = FD(*rc);
    k[0] ^= rj_sbox(k[29]) ^ (*rc);
    k[1] ^= rj_sbox(k[30]);
    k[2] ^= rj_sbox(k[31]);
    k[3] ^= rj_sbox(k[28]);
} /* aes_expandDecKey */

/* -------------------------------------------------------------------------- */
void aes256_init(aes256_context *ctx, unsigned char *k)
{
    unsigned char rcon = 1;
    register unsigned char i;

    for (i = 0; i < sizeof(ctx->key); i++) ctx->enckey[i] = ctx->deckey[i] = k[i];
    for (i = 8; --i;) aes_expandEncKey(ctx->deckey, &rcon);
} /* aes256_init */

/* -------------------------------------------------------------------------- */
void aes256_done(aes256_context *ctx)
{
    register unsigned char i;

    for (i = 0; i < sizeof(ctx->key); i++) ctx->key[i] = ctx->enckey[i] = ctx->deckey[i] = 0;
} /* aes256_done */

/* -------------------------------------------------------------------------- */
void aes256_encrypt_ecb(aes256_context *ctx, unsigned char *buf)
{
    unsigned char i, rcon;

    aes_addRoundKey_cpy(buf, ctx->enckey, ctx->key);
    for (i = 1, rcon = 1; i < 14; ++i)
    {
        aes_subBytes(buf);
        aes_shiftRows(buf);
        aes_mixColumns(buf);
        if (i & 1)
            aes_addRoundKey(buf, &ctx->key[16]);
        else
            aes_expandEncKey(ctx->key, &rcon), aes_addRoundKey(buf, ctx->key);
    }
    aes_subBytes(buf);
    aes_shiftRows(buf);
    aes_expandEncKey(ctx->key, &rcon);
    aes_addRoundKey(buf, ctx->key);
} /* aes256_encrypt */

/* -------------------------------------------------------------------------- */
void aes256_decrypt_ecb(aes256_context *ctx, unsigned char *buf)
{
    unsigned char i, rcon;

    aes_addRoundKey_cpy(buf, ctx->deckey, ctx->key);
    aes_shiftRows_inv(buf);
    aes_subBytes_inv(buf);

    for (i = 14, rcon = 0x80; --i;)
    {
        if ((i & 1))
        {
            aes_expandDecKey(ctx->key, &rcon);
            aes_addRoundKey(buf, &ctx->key[16]);
        }
        else
            aes_addRoundKey(buf, ctx->key);
        aes_mixColumns_inv(buf);
        aes_shiftRows_inv(buf);
        aes_subBytes_inv(buf);
    }
    aes_addRoundKey(buf, ctx->key);
} /* aes256_decrypt */

// END OF AES-256

//////////////////////////////////////////////////////////////////////////
uint16_t crc_tab16[] = {
    0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601, 0x06c0, 0x0780,
    0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1,
    0xce81, 0x0e40, 0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 0xd801,
    0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40,
    0xdd01, 0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680,
    0xd641, 0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040, 0xf001, 0x30c0,
    0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240, 0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501,
    0x35c0, 0x3480, 0xf441, 0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
    0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1, 0xe981,
    0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1,
    0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640, 0x2200,
    0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141,
    0x6300, 0xa3c1, 0xa281, 0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480,
    0xa441, 0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 0xaa01, 0x6ac0,
    0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01,
    0x7bc0, 0x7a80, 0xba41, 0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
    0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381,
    0x7340, 0xb101, 0x71c0, 0x7080, 0xb041, 0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0,
    0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440, 0x9c01,
    0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40, 0x5a00, 0x9ac1, 0x9b81, 0x5b40,
    0x9901, 0x59c0, 0x5880, 0x9841, 0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81,
    0x4a40, 0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41, 0x4400, 0x84c1,
    0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341, 0x4100,
    0x81c1, 0x8081, 0x4040,
};

uint32_t crc_tab32[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535,
    0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd,
    0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d,
    0x6ddde4eb, 0xf4d4b551, 0x83d385c7, 0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,
    0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4,
    0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
    0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59, 0x26d930ac,
    0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab,
    0xb6662d3d, 0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f,
    0x9fbfe4a5, 0xe8b8d433, 0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb,
    0x086d3d2d, 0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
    0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea,
    0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65, 0x4db26158, 0x3ab551ce,
    0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a,
    0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409,
    0xce61e49f, 0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
    0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739,
    0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8,
    0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1, 0xf00f9344, 0x8708a3d2, 0x1e01f268,
    0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0,
    0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8,
    0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef,
    0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703,
    0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7,
    0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d, 0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a,
    0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae,
    0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
    0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777, 0x88085ae6,
    0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d,
    0x3e6e77db, 0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5,
    0x47b2cf7f, 0x30b5ffe9, 0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605,
    0xcdd70693, 0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
    0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};

const unsigned short CRC_INIT = 0x3AA3;

uint16_t crc16_update(uint16_t crc, uint8_t ch)
{
    uint16_t tmp;
    uint16_t msg;

    msg = 0x00ff & (uint16_t)ch;
    tmp = crc ^ msg;
    crc = (crc >> 8) ^ crc_tab16[tmp & 0xff];

    return crc;
}

uint32_t crc32_update(uint32_t crc, uint8_t ch)
{
    uint32_t tmp;
    uint32_t msg;

    msg = 0x000000ffL & (uint32_t)ch;
    tmp = crc ^ msg;
    crc = (crc >> 8) ^ crc_tab32[tmp & 0xff];
    return crc;
}

uint16_t sdk_stream_crc16_calc(const uint8_t *pMsg, size_t nLen)
{
    size_t i;
    uint16_t wCRC = CRC_INIT;

    for (i = 0; i < nLen; i++)
    {
        wCRC = crc16_update(wCRC, pMsg[i]);
    }

    return wCRC;
}

uint32_t sdk_stream_crc32_calc(const uint8_t *pMsg, size_t nLen)
{
    size_t i;
    uint32_t wCRC = CRC_INIT;

    for (i = 0; i < nLen; i++)
    {
        wCRC = crc32_update(wCRC, pMsg[i]);
    }

    return wCRC;
}

/*! @note
 *  A full command, [===========XOOOOOOO]
 *    [===========XOOOOOOO------------------##########] curr filter buffer
 *          ^                                    ^
 *          |                                    |_____ remain data
 *          |__________________________________________ cur cmd data
 *
 *    after prepare
 *
 *    [OOOOOOO------------------------------##########]
 *
 *    [===========X] has been clear
 *
 *    pre-cmd 7 byte has been saved, and continue to filter
 * */
typedef void (*ptr_aes256_codec)(aes256_context *ctx, unsigned char *buf);
using namespace DJI::onboardSDK;

void sdk_stream_prepare_lambda(SDKFilter *p_filter)
{
    unsigned int bytes_to_move = sizeof(Header) - 1;
    unsigned int index_of_move = p_filter->recvIndex - bytes_to_move;

    memmove(p_filter->recvBuf, p_filter->recvBuf + index_of_move, bytes_to_move);
    memset(p_filter->recvBuf + bytes_to_move, 0, index_of_move);
    p_filter->recvIndex = bytes_to_move;
}

void encodeData(SDKFilter *p_filter, Header *p_head, ptr_aes256_codec codec_func)
{
    aes256_context ctx;
    unsigned int buf_i;
    unsigned int loop_blk;
    unsigned int data_len;
    unsigned int data_idx;
    unsigned char *data_ptr;

    if (p_head->enc == 0)
        return;
    if (p_head->length == sizeof(Header))
        return;
    if (p_head->length <= sizeof(Header) + _SDK_CRC_DATA_SIZE)
        return;

    data_ptr = (unsigned char *)p_head + sizeof(Header);
    data_len = p_head->length - _SDK_CRC_DATA_SIZE - sizeof(Header);
    loop_blk = data_len / 16;
    data_idx = 0;

    aes256_init(&ctx, p_filter->sdkKey);
    for (buf_i = 0; buf_i < loop_blk; buf_i++)
    {
        codec_func(&ctx, data_ptr + data_idx);
        data_idx += 16;
    }
    aes256_done(&ctx);

    if (codec_func == aes256_decrypt_ecb)
        p_head->length = p_head->length - p_head->padding; // minus padding length;
}

void DJI::onboardSDK::CoreAPI::callApp(SDKFilter *p_filter)
{
    // pass current data to handler
    Header *p_head = (Header *)p_filter->recvBuf;
    encodeData(p_filter, p_head, aes256_decrypt_ecb);
    appHandler((Header *)p_filter->recvBuf);
    sdk_stream_prepare_lambda(p_filter);
}

bool CoreAPI::decodeACKStatus(unsigned short ack)
{
    switch (ack)
    {
        case ACK_COMMON_SUCCESS:
            API_LOG(driver, STATUS_LOG, "SUCCESS.");
            return true;
        case ACK_COMMON_KEYERROR:
            API_LOG(driver, ERROR_LOG, "Wrong encode Key, Activate again.");
            return false;
        case ACK_COMMON_NO_AUTHORIZATION:
            API_LOG(driver, ERROR_LOG, "Pleasd obtain control and retry.");
            return false;
        case ACK_COMMON_NO_RIGHTS:
            API_LOG(driver, ERROR_LOG, "Need higher Level access.");
            return false;
        case AC_COMMON_NO_RESPONSE:
            API_LOG(driver, ERROR_LOG, "Check your Hardware connection and retry.");
            return false;
        default:
            API_LOG(driver, ERROR_LOG, "Unkown ACK code: 0x%x", ack);
            return false;
    }
}

void sdk_stream_shift_data_lambda(SDKFilter *p_filter)
{
    if (p_filter->recvIndex)
    {
        p_filter->recvIndex--;
        if (p_filter->recvIndex)
        {
            memmove(p_filter->recvBuf, p_filter->recvBuf + 1, p_filter->recvIndex);
        }
    }
}

//! @note push data to filter buffer
void CoreAPI::storeData(SDKFilter *p_filter, unsigned char in_data)
{
    if (p_filter->recvIndex < _SDK_MAX_RECV_SIZE)
    {
        p_filter->recvBuf[p_filter->recvIndex] = in_data;
        p_filter->recvIndex++;
    }
    else
    {
        API_LOG(driver, ERROR_LOG, "buffer overflow");
        memset(p_filter->recvBuf, 0, p_filter->recvIndex);
        p_filter->recvIndex = 0;
    }
}

// this function will move the data part to buffer end,
// head part will move left
//
//  1. there no re-use data
//  |------------------------------------------| <= cache
//                                             ^
//                                             reuse_index
//  [12345678][ data Part ]--------------------| 1. p_filter
//  [12345678]---------------------[ data Part ] 2. move data to end
//  [2345678]----------------------[ data Part ] 3. forward head
//  [2345678]------------[ data need to re-use ] 4. final mem layout
//
//  2. already has re-use data
//  |---------------------------------[rev data] <= cache
//                                    ^
//                                    reuse_index, the data already used
//  [12345678][ data Part ]-----------[rev data] 1. p_filter
//  [12345678]-----------[ data Part ][rev data] 2. move data to end
//  [2345678]------------[ data Part ][rev data] 3. forward head
//  [2345678]------------[ data need to re-use ] 4. final mem layout
//
// the re-use data will loop later

void sdk_stream_update_reuse_part_lambda(SDKFilter *p_filter)
{
    unsigned char *p_buf = p_filter->recvBuf;
    unsigned short bytes_to_move = p_filter->recvIndex - sizeof(Header);
    unsigned char *p_src = p_buf + sizeof(Header);

    unsigned short n_dest_index = p_filter->reuseIndex - bytes_to_move;
    unsigned char *p_dest = p_buf + n_dest_index;

    memmove(p_dest, p_src, bytes_to_move);

    p_filter->recvIndex = sizeof(Header);
    sdk_stream_shift_data_lambda(p_filter);

    p_filter->reuseIndex = n_dest_index;
    p_filter->reuseCount++;
}

void DJI::onboardSDK::CoreAPI::verifyData(SDKFilter *p_filter)
{
    Header *p_head = (Header *)(p_filter->recvBuf);
    if (_SDK_CALC_CRC_TAIL(p_head, p_head->length) == 0)
    {
        callApp(p_filter);
    }
    else
    {
        //! @note data crc fail, re-use the data part
        sdk_stream_update_reuse_part_lambda(p_filter);
    }
}

void DJI::onboardSDK::CoreAPI::verifyHead(SDKFilter *p_filter)
{
    Header *p_head = (Header *)(p_filter->recvBuf);

    if ((p_head->sof == _SDK_SOF) && (p_head->version == 0) &&
        (p_head->length <= _SDK_MAX_RECV_SIZE) && (p_head->reversed0 == 0) &&
        (p_head->reversed1 == 0) && (_SDK_CALC_CRC_HEAD(p_head, sizeof(Header)) == 0))
    {
        // check if this head is a ack or simple package
        if (p_head->length == sizeof(Header))
        {
            callApp(p_filter);
        }
    }
    else
    {
        sdk_stream_shift_data_lambda(p_filter);
    }
}

void DJI::onboardSDK::CoreAPI::checkStream(SDKFilter *p_filter)
{
    Header *p_head = (Header *)(p_filter->recvBuf);

    if (p_filter->recvIndex < sizeof(Header))
    {
        // Continue receive data, nothing to do
        return;
    }
    else if (p_filter->recvIndex == sizeof(Header))
    {
        // recv a full-head
        verifyHead(p_filter);
    }
    else if (p_filter->recvIndex == p_head->length)
    {
        verifyData(p_filter);
    }
}

void DJI::onboardSDK::CoreAPI::streamHandler(SDKFilter *p_filter, unsigned char in_data)
{
    storeData(p_filter, in_data);
    checkStream(p_filter);
}

void DJI::onboardSDK::CoreAPI::byteHandler(const uint8_t in_data)
{
    filter.reuseCount = 0;
    filter.reuseIndex = _SDK_MAX_RECV_SIZE;

    streamHandler(&filter, in_data);

    /*! @note Just think a command as below
        *
        * [123456HHD1234567===HHHH------------------] --- is buf un-used part
        *
        * if after recv full of above, but crc failed, we throw all data?
        * NO!
        * Just throw ONE BYTE, we move like below
        *
        * [123456HH------------------D1234567===HHHH]
        *
        * Use the buffer high part to re-loop, try to find a new command
        *
        * if new cmd also fail, and buf like below
        *
        * [56HHD1234567----------------------===HHHH]
        *
        * throw one byte, buf looks like
        *
        * [6HHD123-----------------------4567===HHHH]
        *
        * the command tail part move to buffer right
        * */
    if (filter.reuseCount != 0)
    {
        while (filter.reuseIndex < _SDK_MAX_RECV_SIZE)
        {
            /*! @note because reuse_index maybe re-located, so reuse_index must
             * be
             *  always point to un-used index
             *  re-loop the buffered data
             *  */
            streamHandler(&filter, filter.recvBuf[filter.reuseIndex++]);
        }
        filter.reuseCount = 0;
    }
}

void CoreAPI::byteStreamHandler(uint8_t *buffer __UNUSED, size_t size __UNUSED)
{
    //! @todo implement stream handler
}

void calculateCRC(void *p_data)
{
    Header *p_head = (Header *)p_data;
    unsigned char *p_byte = (unsigned char *)p_data;
    unsigned int index_of_crc2;

    if (p_head->sof != _SDK_SOF)
        return;
    if (p_head->version != 0)
        return;
    if (p_head->length > _SDK_MAX_RECV_SIZE)
        return;
    if (p_head->length > sizeof(Header) && p_head->length < _SDK_FULL_DATA_SIZE_MIN)
        return;

    p_head->crc = sdk_stream_crc16_calc(p_byte, _SDK_HEAD_DATA_LEN);

    if (p_head->length >= _SDK_FULL_DATA_SIZE_MIN)
    {
        index_of_crc2 = p_head->length - _SDK_CRC_DATA_SIZE;
        _SDK_U32_SET(p_byte + index_of_crc2, sdk_stream_crc32_calc(p_byte, index_of_crc2));
    }
}

void transformTwoByte(const char *pstr, unsigned char *pdata)
{
    int i;
    char temp_area[3];
    unsigned int temp8;
    temp_area[0] = temp_area[1] = temp_area[2] = 0;

    for (i = 0; i < 32; i++)
    {
        temp_area[0] = pstr[0];
        temp_area[1] = pstr[1];
        sscanf(temp_area, "%x", &temp8);
        pdata[i] = temp8;
        pstr += 2;
    }
}

unsigned short DJI::onboardSDK::CoreAPI::encrypt(unsigned char *pdest,
                                                 const unsigned char *psrc,
                                                 unsigned short w_len, unsigned char is_ack,
                                                 unsigned char is_enc, unsigned char session_id,
                                                 unsigned short seq_num)
{
    unsigned short data_len;

    Header *p_head = (Header *)pdest;

    if (w_len > 1024)
        return 0;

    if (filter.encode == 0 && is_enc)
    {
        API_LOG(driver, ERROR_LOG, "Can not send encode data, no available key");
        return 0;
    }
    if (w_len == 0 || psrc == 0)
        data_len = (unsigned short)sizeof(Header);
    else
        data_len = (unsigned short)sizeof(Header) + _SDK_CRC_DATA_SIZE + w_len;

    if (is_enc)
        data_len = data_len + (16 - w_len % 16);

    API_LOG(driver, DEBUG_LOG, "data len: %d\n", data_len);

    p_head->sof = _SDK_SOF;
    p_head->length = data_len;
    p_head->version = 0;
    p_head->sessionID = session_id;
    p_head->isAck = is_ack ? 1 : 0;
    p_head->reversed0 = 0;

    p_head->padding = is_enc ? (16 - w_len % 16) : 0;
    p_head->enc = is_enc ? 1 : 0;
    p_head->reversed1 = 0;

    p_head->sequenceNumber = seq_num;
    p_head->crc = 0;

    if (psrc && w_len)
        memcpy(pdest + sizeof(Header), psrc, w_len);
    encodeData(&filter, p_head, aes256_encrypt_ecb);

    calculateCRC(pdest);

    return data_len;
}
