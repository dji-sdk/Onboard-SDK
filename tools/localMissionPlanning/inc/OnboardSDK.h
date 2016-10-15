/*! @file OnboardSDK.h
 *
 *  @brief
 *  Includes for running an OSDK-based app outside of the OSDK samples
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

//DJI Onboard SDK Core Lib Headers
#include <DJI_HardDriver.h>
#include <DJI_Camera.h>
#include <DJI_Flight.h>
#include <DJI_API.h>
#include <DJI_Type.h>

//DJI Onboard SDK Linux High-Level API Headers
#include <LinuxSerialDevice.h>
#include <LinuxThread.h>
#include <LinuxSetup.h>
#include <LinuxCleanup.h>
#include <ReadUserConfig.h>
#include <LinuxFlight.h>