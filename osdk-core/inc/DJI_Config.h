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
#define MEMORY_SIZE 1024 // unit is byte
#define BUFFER_SIZE 1024
#define ACK_SIZE 10

//! @note The static memory flag means DJI onboardSDK library will not alloc memory from heap.
//! @todo Not supported in this release.

//#define STATIC_MEMORY

//! Uncomment these macros to access various messages from the API. 

//#define API_MISSION_DATA
//#define API_TRACE_DATA
//#define API_DEBUG_DATA
//#define API_BUFFER_DATA
//#define API_RTK_DEBUG
#define API_ERROR_DATA
#define API_STATUS_DATA

//! @note if you do NOT want to use AES encrypt, comment this macro below
//#define USE_ENCRYPT

//! @todo Not supported in this release.
//#define USE_SIMULATION

#include <DJI_Version.h>

#endif // DJI_CONFIG_H
