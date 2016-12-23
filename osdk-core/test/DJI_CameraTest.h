#ifndef ONBOARDSDK_DJI_CAMERATEST_H
#define ONBOARDSDK_DJI_CAMERATEST_H

#include "DJI_APITest.h"
#include <math.h>

class GimbalContainer {
 public:
  struct RotationAngle {
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
  };

  GimbalContainer(float32_t roll = 0,
	       float32_t pitch = 0,
	       float32_t yaw = 0,
	       int duration = 0,
	       int isAbsolute = 0,
	       RotationAngle inertialFrame = {},
	       RotationAngle gimbalFrame = {}):
		 roll(roll), pitch(pitch), yaw(yaw),
		 duration(duration),isAbsolute(isAbsolute),
		 inertialFrame(inertialFrame), gimbalFrame(gimbalFrame){}

  float32_t roll = 0;
  float32_t pitch = 0;
  float32_t yaw = 0;
  int duration = 0;
  int isAbsolute = 0;
  bool yaw_cmd_ignore = false;
  bool pitch_cmd_ignore = false;
  bool roll_cmd_ignore = false;
  RotationAngle inertialFrame;
  RotationAngle gimbalFrame;
};

class DJI_CameraTest : public DJI_APITest,
		       public ::testing::WithParamInterface<GimbalContainer>{
 protected:
  virtual void SetUp();
  virtual void TearDown();

  Camera *camera;
  struct ResultContainer{
    float32_t angle[3] = {0.0};
    float32_t error[3] = {0.0};
  };

  const float PRECISION_DELTA = 5.0;
  GimbalContainer gimbal;

  ResultContainer setGimbalAngleControl(GimbalContainer gimbal);
  float32_t calculateAngle(float32_t referenceAngle, float32_t newGimbalAngle);
  ResultContainer calculatePrecisionError(GimbalContainer gimbal);
  void doPrecisionErrorTest(ResultContainer *result);
  void getCurrentAngle(GimbalContainer *gimbal);
  void getInertialFrame(GimbalContainer *gimbal);
  GimbalContainer::RotationAngle quaternion2euler(QuaternionData q);
  void waitForGimbal();
  void display();
};

#endif
