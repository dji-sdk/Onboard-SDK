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

#include <stdint.h>

#define MAKE_VERSION(a, b, c, d)                                                               \
    (((a << 24) & 0xff000000) | ((b << 16) & 0x00ff0000) | ((c << 8) & 0x0000ff00) |           \
     (d & 0x000000ff))

namespace DJI
{
namespace onboardSDK
{
#ifdef SDK_VERSION_2_3
const uint32_t SDK_VERSION = (MAKE_VERSION(2, 3, 10, 0));
#endif

#ifdef SDK_VERSION_3_0
const uint32_t SDK_VERSION = (MAKE_VERSION(3, 0, 10, 0));
#endif

#ifdef SDK_VERSION_3_1
const uint32_t SDK_VERSION = (MAKE_VERSION(3, 1, 10, 0));
#endif

#ifdef SDK_VERSION_3_0_A3
const uint32_t SDK_VERSION = (MAKE_VERSION(3, 0, 100, 0));
#endif

#ifdef SDK_VERSION_3_1_A3
const uint32_t SDK_VERSION = (MAKE_VERSION(3, 1, 100, 0));
#endif

//! @todo better version control structure
const uint32_t sdkVersionM100_23 = (MAKE_VERSION(2, 3, 10, 0));
const uint32_t sdkVersionM100_30 = (MAKE_VERSION(3, 0, 10, 0));
const uint32_t sdkVersionM100_31 = (MAKE_VERSION(3, 1, 10, 0));
const uint32_t sdkVersionA3_23 = (MAKE_VERSION(2, 3, 100, 0));
const uint32_t sdkVersionA3_30 = (MAKE_VERSION(3, 0, 100, 0));
const uint32_t sdkVersionA3_31 = (MAKE_VERSION(3, 1, 100, 0));

} // namespace DJI
} // namespace onboardSDK



#endif // DJI_VERSION_H
