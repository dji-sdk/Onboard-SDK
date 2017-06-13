/*! @file DJI_Config.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Optional macro definitions for DJI Onboard SDK. Use for debugging.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef DJI_CONFIG_H
#define DJI_CONFIG_H

#include <stdint.h>
#define DJI_MEMORY_SIZE 1024 // unit is byte
#define DJI_BUFFER_SIZE 1024
#define DJI_ACK_SIZE 10

//! @note The static memory flag means DJI onboardSDK library will not alloc
//! memory from heap.
//! @todo Not supported in this release.

//#define DJI_STATIC_MEMORY

//! Uncomment these macros to access various messages from the API.

//#define DJI_API_MISSION_DATA
//#define DJI_API_TRACE_DATA
//#define DJI_API_DEBUG_DATA
//#define DJI_API_BUFFER_DATA
//#define DJI_API_RTK_DEBUG
#define DJI_API_ERROR_DATA
#define DJI_API_STATUS_DATA

//! @note if you do NOT want to use AES encrypt, comment this macro below
//#define DJI_USE_ENCRYPT

//! @todo Not supported in this release.
//#define DJI_USE_SIMULATION

#include <DJI_Version.h>

#endif // DJI_CONFIG_H
