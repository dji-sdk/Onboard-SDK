/*! @file AppKey.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Put your APP_KEY and APP_ID here.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */
#ifndef STM32PINGEXAMPLE_SRC_APPKEY_H_
#define STM32PINGEXAMPLE_SRC_APPKEY_H_

#include "DJI_API.h"
#include "DJI_Type.h"
#include "DJI_Version.h"

/*
 * Register at https://developer.dji.com/register/ to get APP_KEY and APP_ID
 */
static char APP_KEY[65] = "12345abcde";
static uint32_t APP_ID =  1234567;
static DJI::onboardSDK::Version userDroneVersion = DJI::onboardSDK::versionM100_31;

DJI::onboardSDK::ActivateData userActivationInfo = {
    APP_ID,
    0,
    userDroneVersion,
    {0},
    APP_KEY
};


#endif /* STM32PINGEXAMPLE_SRC_APPKEY_H_ */
