/*! @file MobileCommand.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Receive command from mobile OSDK App.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef SAMPLE_STM32_ONBOARDSDK_STM32_USER_MOBILECOMMAND_H_
#define SAMPLE_STM32_ONBOARDSDK_STM32_USER_MOBILECOMMAND_H_

#include "stdint.h"
#include "DJI_API.h"

using namespace DJI::onboardSDK;

typedef struct ackReturnToMobile{
  uint16_t cmdID;
  uint16_t ack;
} ackReturnToMobile;

void mobileCommandHandler(CoreAPI* api, Flight* flight);




#endif /* SAMPLE_STM32_ONBOARDSDK_STM32_USER_MOBILECOMMAND_H_ */
