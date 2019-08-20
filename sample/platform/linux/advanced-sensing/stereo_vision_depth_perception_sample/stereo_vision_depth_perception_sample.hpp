#ifndef ONBOARDSDK_ADVANCED_SENSING_DEPTH_PERCEPTION_SAMPLE_HPP
#define ONBOARDSDK_ADVANCED_SENSING_DEPTH_PERCEPTION_SAMPLE_HPP

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
#include "stereo_process_container.hpp"

ImageProcessContainer* image_process_container_ptr;

static void storeStereoImgVGACallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::RecvContainer recvFrame, DJI::OSDK::UserData userData);


#endif //ONBOARDSDK_ADVANCED_SENSING_DEPTH_PERCEPTION_SAMPLE_HPP
