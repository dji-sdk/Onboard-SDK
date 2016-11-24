/** @file DJI_Logging.h
 *  @version 3.1.9
 *  @date August 15th, 2016
 *
 *  @brief
 *  Implement logging functions for the SDK.
 *
 *  @copyright 2016 DJI. All right reserved.
 *
 */

#ifndef ONBOARDSDK_DJI_LOGGING_H
#define ONBOARDSDK_DJI_LOGGING_H

#include "DJI_API.h"
#include "DJI_Type.h"

#ifdef API_TRACE_DATA
namespace DJI {
namespace onboardSDK {

typedef struct __Command {
  uint8_t set_id;
  uint8_t id;
} __Command;

typedef struct __ActivationGetProtocolVersionCommand {
  uint8_t set_id;
  uint8_t id;
  uint8_t val;
} __ActivationGetProtocolVersionCommand;

enum __ActivationGetProtocolVersionAckCodes {
  AUTOPILOT_ACTIVATED = 0x0000,
  AUTOPILOT_NOT_ACTIVATED = 0xFF01
};

typedef struct __ActivationGetProtocolVersionAck {
  __ActivationGetProtocolVersionAckCodes status;
  uint32_t crc;
  uint8_t version[32];
} __ActivationGetProtocolVersionAck;

void printFrame(HardDriver *hardDriver, Header *header, bool toAircraft);
}
}
#endif  // API_TRACE_DATA
#endif  // ONBOARDSDK_DJI_LOGGING_H
