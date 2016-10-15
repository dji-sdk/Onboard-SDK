#ifndef ONBOARDSDK_DJI_CAMERATEST_H
#define ONBOARDSDK_DJI_CAMERATEST_H

#include "DJI_APITest.h"

class GimbalContainer {
 public:
  struct RotationAngle {
    float32_t roll;
    float32_t pitch;
    float32_t yaw;
  };

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
};

class DJI_CameraTest : public DJI_APITest,
		       public ::testing::WithParamInterface<GimbalContainer>{
 protected:
  virtual void SetUp();
  virtual void TearDown();

  Camera *camera;
  struct ResultContainer{
    int angle[3] = {0};
    int error[3] = {0};
  };

  const int PRECISION_DELTA = 1;
  GimbalContainer gimbal;
  GimbalContainer::RotationAngle initialAngle;

  ResultContainer setGimbalAngleControl(GimbalContainer gimbal);
  int calculateAngle(int currentAngle, int newAngle);
  ResultContainer calculatePrecisionError(GimbalContainer gimbal);
  void doPrecisionErrorTest(ResultContainer *result);
  void getCurrentAngle(GimbalContainer *gimbal);
  void getInitialAngle(GimbalContainer *gimbal);
  void waitForGimbal();
};

#endif
