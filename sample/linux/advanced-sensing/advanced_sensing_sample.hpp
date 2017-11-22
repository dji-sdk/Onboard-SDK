/*! @file advanced-sensing_sample.cpp
 *  @version 3.3.2
 *  @date Aug 25 2017
 *
 *  @brief
 *  AdvancedSensing API usage in a Linux environment.
 *  Provides an example callback function to subscribe to stereo images
 *  (Optional) With OpenCV installed, user can visualize the images
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#ifndef ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H
#define ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H

// DJI OSDK includes
#include "dji_vehicle.hpp"

// C++
#include <iostream>

// Helpers
#include <dji_linux_helpers.hpp>

#ifdef OPEN_CV_INSTALLED
  #include <opencv2/opencv.hpp>
#endif

static void stereoImg240pCallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::RecvContainer recvFrame, DJI::OSDK::UserData userData);

static void stereoImgVGACallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::RecvContainer recvFrame, DJI::OSDK::UserData userData);


#endif //ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H
