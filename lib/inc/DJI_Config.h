/*! @brief
 *  @file DJI_Config.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Configuration optional Micro definitions for DJI onboardSDK library.
 *
 *  @attention
 *  Project configuration:
 *  None
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Dec 16, 2015
 *  -* @author william.wu
 *
 * */

#ifndef DJI_CONFIG_H
#define DJI_CONFIG_H

#include <stdint.h>
#define MEMORY_SIZE 1024 // unit is byte
#define BUFFER_SIZE 1024
#define ACK_SIZE 10

//! @note it means DJI onboardSDK library will not alloc memory from heap.
//! @todo not available yet, only affect WayPoint
//#define STATIC_MEMORY

//#define API_DEBUG_DATA
#define API_ERROR_DATA
#define API_STATUS_DATA

//#define SDK_VERSION_2_3
#define SDK_VERSION_3_1
//#define SDK_VERSION_3_0_A3
//#define SDK_VERSION_3_1_A3
#include <DJI_Version.h>

#endif // DJI_CONFIG_H
