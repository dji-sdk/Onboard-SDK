#ifndef __DJI_CONFIG_H__
#define __DJI_CONFIG_H__

#include <stdint.h>
#define MEMORY_SIZE 1024 // unit is byte
#define BUFFER_SIZE 1024
#define ACK_SIZE 10

//#define API_DEBUG_DATA
#define API_ERROR_DATA
#define API_STATUS_DATA

#define MAKE_VERSION(a, b, c, d)                                               \
    (((a << 24) & 0xff000000) | ((b << 16) & 0x00ff0000) |                     \
     ((c << 8) & 0x0000ff00) | (d & 0x000000ff))

#define SDK_VERSION_3_0
#ifdef SDK_VERSION_3_0
#define SDK_VERSION (MAKE_VERSION(3, 0, 10, 0))
#endif

#endif //__DJI_CONFIG_H__
