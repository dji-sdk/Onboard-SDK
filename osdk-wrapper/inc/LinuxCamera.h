/*! @file LinuxCamera.h
 *  @version 3.1.9
 *  @date September 20 2016
 *
 *  @brief
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef LINUXCAMERA_H
#define LINUXCAMERA_H

#include <DJI_Type.h>
#include <DJI_API.h>
#include <DJI_Camera.h>
#include "LinuxSetup.h"

using namespace DJI::onboardSDK;

// Be precise here
struct RotationAngle{
   float32_t roll;
   float32_t pitch;
   float32_t yaw;
 };
class GimbalContainer{
 public:
  int roll;
  int pitch;
  int yaw;
  int duration;
  int isAbsolute;
  bool yaw_cmd_ignore;
  bool pitch_cmd_ignore;
  bool roll_cmd_ignore;
  RotationAngle initialAngle;
  RotationAngle currentAngle;
  
  void set(int roll,
	  int pitch,
	  int yaw,
	  int duration,
	  int isAbsolute,
	  bool yaw_cmd_ignore,
          bool pitch_cmd_ignore,
          bool roll_cmd_ignore,
          RotationAngle initialAngle,
	  RotationAngle currentAngle){
    this->roll = roll;
    this->pitch = pitch; 
    this->yaw = yaw;
    this->duration = duration;
    this->isAbsolute = isAbsolute;
    this->yaw_cmd_ignore = yaw_cmd_ignore;
    this->pitch_cmd_ignore = yaw_cmd_ignore;
    this->roll_cmd_ignore = roll_cmd_ignore;
    this->initialAngle = initialAngle; 
    this->currentAngle = currentAngle;
  }
};

struct ResultContainer{
  int angle[3];
  int error[3];
};

void gimbalAngleControlSample(Camera *camera, int timeout = 1);
void gimbalSpeedControlSample(Camera* camera, int timeout = 1);
void takePictureControl(Camera *camera, int timeout = 1);
void takeVideoControl(Camera *camera, int timeout = 1);

// Helper functions
void waitForGimbal(Camera *camera);
void doSetGimbalAngle(Camera *camera, GimbalContainer gimbal);
ResultContainer calculatePrecisionError(Camera *camera, GimbalContainer *gimbal);
int calculateAngle(int currentAngle, int newAngle);
void displayResult(ResultContainer *resultContainer);

#endif //LINUXCAMERA_H

