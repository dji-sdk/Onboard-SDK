/** @file DJI_Camera.h
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Camera/Gimbal API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#ifndef DJI_CAMERA_H
#define DJI_CAMERA_H

#include "DJI_API.h"

namespace DJI
{
namespace onboardSDK
{
//! Camera class for controlling camera and gimbal-related functions available through open protocol 
class Camera
{
  public:
  enum CAMERA_CODE
  {
    CODE_GIMBAL_SPEED = 0x1A,
    CODE_GIMBAL_ANGLE = 0x1B,
    CODE_CAMERA_SHOT = 0x20,
    CODE_CAMERA_VIDEO_START = 0x21,
    CODE_CAMERA_VIDEO_STOP = 0x22
  };

  public:
  Camera(CoreAPI *ControlAPI = 0);

  //! @note all camera control functions are none-callback function.
  //! The setCamera function handles camera-specific tasks (Take picture, Start/stop video)
  void setCamera(CAMERA_CODE camera_cmd);
  void setGimbalAngle(GimbalAngleData *data);
  void setGimbalSpeed(GimbalSpeedData *data);

  GimbalData getGimbal() const;
  float32_t getYaw() const;
  float32_t getRoll() const;
  float32_t getPitch() const;
  bool isYawLimit() const;
  bool isRollLimit() const;
  bool isPitchLimit() const;

  public: //! @note Access method
  CoreAPI *getApi() const;
  void setApi(CoreAPI *value);

  private:
  CoreAPI *api;
};
} // namespace onboardSDK
} // namespace DJI

#endif // DJI_CAMERA_H
