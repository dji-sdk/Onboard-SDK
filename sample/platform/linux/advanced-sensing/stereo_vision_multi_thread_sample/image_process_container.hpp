/*! @file image_process_container.hpp
 *  @version 3.4
 *  @date Oct 10 2017
 *
 *  @brief
 *  A simple class to allow user to copy images
 *  from reading thread and process them in
 *  a separate image_processing thread
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

#ifndef ONBOARDSDK_IMAGE_CONTAINER_H
#define ONBOARDSDK_IMAGE_CONTAINER_H

#include "utility_thread.hpp"
#include "dji_vehicle.hpp"

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#endif

class UtilityThread;

class ImageProcessContainer
{
public:

  typedef void (*ImgCallBack) (DJI::OSDK::Vehicle* vehicle, DJI::OSDK::UserData userData);

  typedef struct ImgCallBackHandler
  {
    ImgCallBack         callback;
    DJI::OSDK::UserData userData;
  } ImgCallBackHandler;

  ImageProcessContainer(DJI::OSDK::Vehicle *vehicle);
  virtual ~ImageProcessContainer();

  static const int IMG_PROCESS_THREAD = 1;

  // copy images into this container
  void copyVGAImg(const DJI::OSDK::ACK::StereoVGAImgData* img);
  void copy240pImg(const DJI::OSDK::ACK::StereoImgData* img);

  // sample cb functions to process img
  static void displayStereoVGACallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::UserData userData);
  static void displayStereo240pCallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::UserData userData);

  // keep polling to see if there's any new data coming in
  bool newDataPoll();

  // use this function to register callback
  void setCallbackHandler(ImgCallBack callback, DJI::OSDK::UserData userData);

  UtilityThread* getImgProcessThread();

protected:
  bool initImgProcessThread();

protected:
  DJI::OSDK::ACK::StereoVGAImgData stereoVGAImg;
  DJI::OSDK::ACK::StereoImgData    stereo240pImg;

  bool gotNew240pImg;
  bool gotNewVGAImg;

  ImgCallBackHandler imgCallBackHandler;

  DJI::OSDK::Vehicle* vehicle_ptr;
  UtilityThread*      imgProcessThread;
};


#endif //ONBOARDSDK_IMAGE_CONTAINER_H
