/*! @file image_process_container.hpp
 *  @version 3.4
 *  @date Oct 10 2017
 *
 *  @brief
 *  A simple class to allow user to copy images
 *  from reading thread and process them in
 *  a separate image_processing thread
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 */

#ifndef ONBOARDSDK_IMAGE_CONTAINER_H
#define ONBOARDSDK_IMAGE_CONTAINER_H

#include "utility_thread.hpp"
#include "dji_vehicle.hpp"

#ifdef OPEN_CV_INSTALLED
  #include <opencv2/opencv.hpp>
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
  ~ImageProcessContainer();

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

private:
  bool initImgProcessThread();

private:
  DJI::OSDK::ACK::StereoVGAImgData stereoVGAImg;
  DJI::OSDK::ACK::StereoImgData    stereo240pImg;

  bool gotNew240pImg;
  bool gotNewVGAImg;

  ImgCallBackHandler imgCallBackHandler;

  DJI::OSDK::Vehicle* vehicle_ptr;
  UtilityThread*      imgProcessThread;
};


#endif //ONBOARDSDK_IMAGE_CONTAINER_H