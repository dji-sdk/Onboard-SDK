/*! @brief
 *  @file DJI_Link.h
 *  @version 3.0
 *  @date Dec 4, 2015
 *
 *  @abstract
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Nov 15, 2015
 *  -* @author william.wu
 *  -*
 *  -* @version V2.0
 *  -* C-like DJI-onboard-SDK library
 *  -* @date Mar 12, 2015
 *  -* @author wuyuwei
 *
 * */
#ifndef DJI_LINK_H
#define DJI_LINK_H

#define ACK_SESSION_IDLE 0
#define ACK_SESSION_PROCESS 1
#define ACK_SESSION_USING 2
#define CMD_SESSION_0 0
#define CMD_SESSION_1 1
#define CMD_SESSION_AUTO 32


#define POLL_TICK 20 // unit is ms

#include "DJI_Type.h"

#endif // DJI_LINK_H
