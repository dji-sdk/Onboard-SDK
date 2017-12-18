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
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H
#define ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H

// DJI OSDK includes
#include "dji_vehicle.hpp"

// C++
#include <iostream>

// Helpers
#include "dji_linux_helpers.hpp"

// Third party library
#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#endif

// Utility
#include "utility_thread.hpp"

ImageProcessContainer* image_process_container_ptr;

static void storeStereoImg240pCallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::RecvContainer recvFrame, DJI::OSDK::UserData userData);

static void storeStereoImgVGACallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::RecvContainer recvFrame, DJI::OSDK::UserData userData);


#endif //ONBOARDSDK_ADVANCED_SENSING_SAMPLE_H
