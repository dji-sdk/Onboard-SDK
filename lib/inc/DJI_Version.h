/*! @brief
 *  @file DJI_Version.h
 *  @version 3.0
 *  @date Dec 16, 2015
 *
 *  @abstract
 *  Version control definition for DJI onboardSDK library
 *  Maintain officially
 *
 *  @attention
 *  Maintain officially, readonly for users
 *  Do not modify any definition in this file,
 *  if you are not sure what are you doing exactlly,
 *  or we will not provide any support.
 *
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Dec 16, 2015
 *  -* @author william.wu
 *
 * */

/*! @attention
 *  Maintain officially, readonly for users
 *  Do not modify any definition in this file,
 *  if you are not sure what are you doing exactlly,
 *  or we will not provide any support.
 * */

#ifndef DJI_VERSION_H
#define DJI_VERSION_H

#define MAKE_VERSION(a, b, c, d)                                               \
    (((a << 24) & 0xff000000) | ((b << 16) & 0x00ff0000) |                     \
     ((c << 8) & 0x0000ff00) | (d & 0x000000ff))

#ifdef SDK_VERSION_2_3
#define SDK_VERSION (MAKE_VERSION(2, 3, 10, 0))
#endif

#ifdef SDK_VERSION_3_0
#define SDK_VERSION (MAKE_VERSION(3, 0, 10, 0))
#endif

#ifdef SDK_VERSION_3_1
#define SDK_VERSION (MAKE_VERSION(3, 1, 10, 0))
#endif

#endif // DJI_VERSION_H

