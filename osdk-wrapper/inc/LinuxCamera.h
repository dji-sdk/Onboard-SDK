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

struct GimbalContainer{
  int roll = 0;
  int pitch = 0;
  int yaw = 0;
  int duration = 0;
  int isAbsolute = 0;
  bool yaw_cmd_ignore = false;
  bool pitch_cmd_ignore = false;
  bool roll_cmd_ignore = false;
  RotationAngle initialAngle;
  RotationAngle currentAngle;
  GimbalContainer( int roll = 0,
		   int pitch = 0,
		   int yaw = 0,
		   int duration = 0,
		   int isAbsolute = 0,
		   RotationAngle initialAngle = {},
		   RotationAngle currentAngle = {}):
		     roll(roll), pitch(pitch), yaw(yaw),
		     duration(duration),isAbsolute(isAbsolute),
		     initialAngle(initialAngle), currentAngle(currentAngle){}
};

struct ResultContainer{
  int angle[3] = {0};
  int error[3] = {0};
};

void gimbalAngleControlSample(Camera *camera, int timeout = 1);
void gimbalSpeedControlSample(Camera* camera, int timeout = 1);
void takePictureControl(Camera *camera, int timeout = 1);
void takeVideoControl(Camera *camera, int timeout = 1);

// Helper functions
void waitForGimbal(Camera *camera);
void doSetGimbalAngle(Camera *camera, GimbalContainer *gimbal);
ResultContainer calculatePrecisionError(Camera *camera, GimbalContainer *gimbal);
int calculateAngle(int currentAngle, int newAngle);
void displayResult(ResultContainer *resultContainer);

#endif //LINUXCAMERA_H
