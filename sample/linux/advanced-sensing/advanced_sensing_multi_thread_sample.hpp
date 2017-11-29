/*! @file advanced_sensing_multi_thread_sample.cpp
 *  @version 3.4
 *  @date Oct 10 2017
 *
 *  @brief
 *  AdvancedSensing API usage in a Linux environment.
 *  Provides an example callback function to subscribe to stereo images
 *  and an example class to process images in a separate thread
 *  (Optional) With OpenCV installed, user can visualize the images
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 */

#ifndef ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H
#define ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H

// DJI OSDK includes
#include "dji_vehicle.hpp"

// C++
#include <iostream>

// Helpers
#include <dji_linux_helpers.hpp>

// Third party library
#ifdef OPEN_CV_INSTALLED
  #include <opencv2/opencv.hpp>
#endif

// Utility
#include "utility_thread.hpp"

ImageProcessContainer* image_process_container_ptr;

static void storeStereoImg240pCallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::RecvContainer recvFrame, DJI::OSDK::UserData userData);

static void storeStereoImgVGACallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::RecvContainer recvFrame, DJI::OSDK::UserData userData);


#endif //ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H
