#ifndef ONBOARDSDK_STEREO_PROCESS_CONTAINER_H
#define ONBOARDSDK_STEREO_PROCESS_CONTAINER_H

#include "image_process_container.hpp"
#include "stereo_frame.hpp"
#include "camera_param.hpp"
#include <chrono>


class StereoProcessContainer : public ImageProcessContainer
{
public:
  StereoProcessContainer(DJI::OSDK::Vehicle *vehicle);
  virtual ~StereoProcessContainer();

public:
  bool initStereoFrame();

  static void displayStereoRectImgCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                           DJI::OSDK::UserData userData);

  static void displayStereoDisparityCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                             DJI::OSDK::UserData userData);

  static void displayStereoFilteredDisparityCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                                     DJI::OSDK::UserData userData);

  static void displayStereoPtCloudCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                           DJI::OSDK::UserData userData);

protected:
  void visualizeRectImgHelper();

  // The disparity map return by stereoFrame is the actual pixel diff
  // The image will be very dimmed if visualize directly
  // this function scales it for visualization purpose
  void visualizeDisparityMapHelper();

protected:
  M210_STEREO::CameraParam::Ptr camera_left_ptr;
  M210_STEREO::CameraParam::Ptr camera_right_ptr;

  M210_STEREO::StereoFrame::Ptr stereo_frame_ptr;

  // for visualization purpose
  bool is_disp_filterd;
};


#endif //ONBOARDSDK_STEREO_PROCESS_CONTAINER_H
