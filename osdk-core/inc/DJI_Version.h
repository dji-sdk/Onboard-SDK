/*! @file DJI_Version.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Drone/SDK Version definition for DJI onboardSDK library
 *  Officially Maintained
 *  
 *  @copyright
 *  Copyright 2016 DJI. All rights reserved.
 * */

/*! @attention
 *  Do not modify any definition in this file
 *  if you are not sure what are you doing.
 *  DJI will not provide any support for changes made to this file.
 * */

#ifndef DJI_VERSION_H
#define DJI_VERSION_H

#include <stdint.h>

#define MAKE_VERSION(a, b, c, d)                                                    \
  (((a << 24) & 0xff000000) | ((b << 16) & 0x00ff0000) | ((c << 8) & 0x0000ff00) |  \
  (d & 0x000000ff))

namespace DJI
{
namespace onboardSDK
{
//! Different version strings define SDK/Drone combination. Only the ones listed below are available.
typedef uint32_t Version;

const Version versionM100_23 = (MAKE_VERSION(2, 3, 10, 0));
const Version versionM100_31 = (MAKE_VERSION(3, 1, 10, 0));
const Version versionA3_31 = (MAKE_VERSION(3, 1, 100, 0));
const Version versionA3_32 = (MAKE_VERSION(3, 2, 0, 0));

#ifdef SDK_DEV
#include "dev.h"
#endif // SDK_DEV

} // namespace DJI
} // namespace onboardSDK

#endif // DJI_VERSION_H
