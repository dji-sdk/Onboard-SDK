#include "DJI_CameraTest.h"

void DJI_CameraTest::SetUp() {
  DJI_APITest::SetUp();

  activateDroneStandard();
  setControlStandard();

  camera = new Camera(DJI_APITest::api);

  // Wait for Gimbal to sync
  waitForGimbal();

}

void DJI_CameraTest::TearDown() {
  releaseControlStandard();
  delete camera;
  DJI_APITest::TearDown();
}

::testing::AssertionResult ResultGimbalAngleNear(const float32_t angle,
						 const float32_t precisionError,
						 const float32_t delta) {
  std::cout << "angle = " << std::fixed << std::setprecision(3) << angle
    << ", precision error = " << std::fixed << std::setprecision(3)
    << precisionError << " degree(s)\n";

  if (precisionError <= delta)
    return ::testing::AssertionSuccess();
  else
    return ::testing::AssertionFailure();
}

void DJI_CameraTest::doPrecisionErrorTest(ResultContainer *result) {
  int length = sizeof(result->angle) / sizeof(int);

  std::cout << "New Gimbal Angle [roll, pitch, yaw]:\n";
  for(int i = 0; i < length; i++)
    EXPECT_TRUE(ResultGimbalAngleNear(result->angle[i], result->error[i], PRECISION_DELTA));

  std::cout << std::endl;
}

/*
 * @brief
 *
 * Yaw angle unit 0.1 degrees, input range [-3200, 3200]
 * Roll angle unit 0.1 degrees, input range [-350, 350]
 * Pitch angle unit 0.1 degrees, input angle [-900, 300]
 *
 * isAbsolute bit - control flag:
 *
 * 0 - incremental control, the angle reference is the current Gimbal
 *  location
 *
 * 1 - absolute control, the angle reference is related to configuration
 * in DJI Go App
 *
 * yaw_cmd_ignore bit - Yaw invalid bit:
 *
 * 0 - gimbal will follow the command in Yaw
 *
 * 1 - gimbal will maintain position in Yaw
 *
 * roll_cmd_ignore bit - Roll invalid bit, same meaning as Yaw invalid bit
 * pitch_cmd_ignore bit - Pitch invalid bit, same meaning as Yaw invalid bit
 */
DJI_CameraTest::ResultContainer DJI_CameraTest::setGimbalAngleControl(GimbalContainer gimbal) {

  // Set control of the camera
  camera->setCamera(Camera::CAMERA_CODE::CODE_GIMBAL_ANGLE);

  GimbalAngleData gimbalAngle;
  gimbalAngle.roll = gimbal.roll;
  gimbalAngle.pitch = gimbal.pitch;
  gimbalAngle.yaw = gimbal.yaw;
  gimbalAngle.duration = gimbal.duration;
  gimbalAngle.mode |= 0;
  gimbalAngle.mode |= gimbal.isAbsolute;
  gimbalAngle.mode |= gimbal.yaw_cmd_ignore << 1;
  gimbalAngle.mode |= gimbal.roll_cmd_ignore << 2;
  gimbalAngle.mode |= gimbal.pitch_cmd_ignore << 3;

  // Get rotation angle for precision error assessment
  if(gimbal.isAbsolute)
    getInertialFrame(&gimbal);
  else
    getCurrentAngle(&gimbal);

  camera->setGimbalAngle(&gimbalAngle);
  // Give time for gimbal to sync
  sleep(4);

  return calculatePrecisionError(gimbal);
}

/**
 * @brief
 *
 * Calculate precision error
 */
DJI_CameraTest::ResultContainer DJI_CameraTest::calculatePrecisionError(GimbalContainer gimbal) {
  ResultContainer result;
  float32_t expectedAngle;

  for(int i = 0; i < 3; i++) {
    if(i == 0){
      result.angle[i] = camera->getGimbal().roll;
      if(gimbal.isAbsolute)
        expectedAngle = calculateAngle(gimbal.inertialFrame.roll, gimbal.roll);
      else
        expectedAngle = calculateAngle(gimbal.gimbalFrame.roll, gimbal.roll);
    }else if(i == 1){
      result.angle[i] = camera->getGimbal().pitch;
      if(gimbal.isAbsolute)
        expectedAngle = calculateAngle(gimbal.inertialFrame.pitch, gimbal.pitch);
      else
        expectedAngle = calculateAngle(gimbal.gimbalFrame.pitch, gimbal.pitch);
    }else if( i == 2){
      result.angle[i] = camera->getGimbal().yaw;
      if(gimbal.isAbsolute)
        expectedAngle = calculateAngle(gimbal.inertialFrame.yaw, gimbal.yaw);
      else
        expectedAngle = calculateAngle(gimbal.gimbalFrame.yaw, gimbal.yaw);
    }
    result.error[i] = abs(expectedAngle - result.angle[i]);
  }
  return result;
}

/**
 * @brief
 *
 * Calculate new Gimbal angle
 */
float32_t DJI_CameraTest::calculateAngle(float32_t referenceAngle, float32_t newGimbalAngle) {
  float32_t n = (newGimbalAngle / 10) + referenceAngle;
  float32_t absAngle = std::abs(n);

  if(absAngle > 180 && n < 0)
    return n + 360;
  else if (absAngle > 180 && n > 0)
    return n - 360;
  else
    return n;
}

void DJI_CameraTest::getCurrentAngle(GimbalContainer *gimbal) {
  gimbal->gimbalFrame.roll = camera->getGimbal().roll;
  gimbal->gimbalFrame.pitch = camera->getGimbal().pitch;
  gimbal->gimbalFrame.yaw = camera->getGimbal().yaw;
}

void DJI_CameraTest::getInertialFrame(GimbalContainer *gimbal) {
  GimbalContainer::RotationAngle euler = quaternion2euler(api->getBroadcastData().q);

  gimbal->inertialFrame.roll = euler.roll;
  gimbal->inertialFrame.pitch = euler.pitch;
  gimbal->inertialFrame.yaw = euler.yaw;
}

/**
 *@brief
 *
 *Calculate inertial frame
 */
GimbalContainer::RotationAngle DJI_CameraTest::quaternion2euler(QuaternionData q){
  GimbalContainer::RotationAngle euler;

  float32_t yaw = atan2(2 * (q.q0 * q.q3 + q.q1 * q.q2), 1 - 2 * (q.q2 * q.q2 + q.q3 * q.q3));
  float32_t pitch = asin(2 * (q.q0 * q.q2 - q.q3 * q.q1));
  float32_t roll = atan2(2 * (q.q0 * q.q1 + q.q2 * q.q3), 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2));

  // Convert to degrees
  float32_t r = roll  * 180.0 / M_PI;
  float32_t p = pitch  * 180.0 / M_PI;
  float32_t y = yaw  * 180.0 / M_PI;

  euler.roll = r;
  euler.pitch = p;
  euler.yaw = y;

  return euler;
}

void DJI_CameraTest::waitForGimbal() {
  GimbalContainer::RotationAngle rAngle;

  do{
      rAngle.roll = camera->getGimbal().roll;
      rAngle.pitch = camera->getGimbal().pitch;
      rAngle.yaw = camera->getGimbal().yaw;
      sleep(1);
  }
  while(fabs(camera->getGimbal().roll - rAngle.roll) >= 0.099 ||
      fabs(camera->getGimbal().pitch - rAngle.pitch) >= 0.099 ||
      fabs(camera->getGimbal().yaw - rAngle.yaw) >= 0.099);
}

void DJI_CameraTest::display() {
  getInertialFrame(&gimbal);
  // In the simulattion mode, Yaw will be in relation to body frame
  std::cout << "\nInertial Frame [ "
      << gimbal.inertialFrame.roll
      << ", " << gimbal.inertialFrame.pitch
      << ", " << gimbal.inertialFrame.yaw << " ]\n";

  std::cout << "Setting Gimbal angle to [ "
      << GetParam().roll << ", "
      << GetParam().pitch << ", "
      << GetParam().yaw << " ], control: "
      << GetParam().isAbsolute << ", duration: "
      << GetParam().duration << "\n";
}

INSTANTIATE_TEST_CASE_P(
    DISABLED_setGimbalAngle, DJI_CameraTest, testing::Values(

  // Roll - absolute control
  /*
   * Roll angle fails to move to its min value of
   * -35 degrees. It goes only down to -29 degrees.
   */
  //GimbalContainer{-350, 0, 0, 20, 1},
  GimbalContainer{-250, 0, 0, 20, 1},
  GimbalContainer{-100, 0, 0, 20, 1},
  GimbalContainer{-55, 0, 0, 20, 1},
  /*
   * Roll angle fails to move to its max value of
   * 35 degrees. It goes only up to 29 degrees.
   */
  //GimbalContainer{350, 0, 0, 20, 1},
  GimbalContainer{250, 0, 0, 20, 1},
  GimbalContainer{100, 0, 0, 20, 1},
  GimbalContainer{55, 0, 0, 20, 1},

  // Roll - incremental control
  /*
   * Roll angle fails to move to its min value of
   * -35 degrees. It goes only down to -29 degrees.
   */
  //GimbalContainer{-350, 0, 0, 20, 0},
  GimbalContainer{-290, 0, 0, 20, 0},
  GimbalContainer{-280, 0, 0, 20, 0},
  GimbalContainer{-5, 0, 0, 20, 0},
  GimbalContainer{-45, 0, 0, 20, 0},

  // Pitch - absolute
  GimbalContainer{0, -300, 0, 20, 1},
  GimbalContainer{0, -500, 0, 20, 1},
  GimbalContainer{0, -700, 0, 20, 1},
  /**
   *@note
   *Rotating 90 degree in pitch direction will cause gimbal lock problem,
   *in which the value of roll and yaw are not reliable.
   */
  //GimbalContainer{0, -900, 0, 20, 1},
  GimbalContainer{0, 300, 0, 20, 1},
  GimbalContainer{0, 200, 0, 20, 1},
  GimbalContainer{0, 100, 0, 20, 1},

  // Pitch - incremental
  GimbalContainer{0, 300, 0, 20, 0},
  GimbalContainer{0, 200, 0, 20, 0},
  GimbalContainer{0, 100, 0, 20, 0},

  // Yaw - absolute
  GimbalContainer{0, 0, 100, 10, 1},
  GimbalContainer{0, 0, 200, 10, 1},
  GimbalContainer{0, 0, 300, 10, 1},
  GimbalContainer{0, 0, 400, 10, 1},
  GimbalContainer{0, 0, 600, 10, 1},
  GimbalContainer{0, 0, 1200, 10, 1},
  GimbalContainer{0, 0, 2000, 10, 1},
  GimbalContainer{0, 0, 2500, 10, 1},
  GimbalContainer{0, 0, 3000, 10, 1},
  GimbalContainer{0, 0, 3100, 10, 1},

  // Test cases that will lead to locked state
  GimbalContainer{0, 0, 1000, 10, 0},
  GimbalContainer{0, 0, -1000, 10, 1},
  GimbalContainer{0, 0, 500, 10, 0},  //LOCKED if not re-set
  GimbalContainer{0, 0, -500, 10, 0},
  GimbalContainer{0, 0, 1200, 10, 0}, //LOCKED if not re-set
  GimbalContainer{0, 0, -1200, 10, 0},

  // Yaw - incremental
  GimbalContainer{0, 0, -1000, 20, 0},
  GimbalContainer{0, 0, 500, 20, 0},
  GimbalContainer{0, 0, -3100, 20, 0},
  GimbalContainer{0, 0, 2800, 20, 0},
  GimbalContainer{0, 0, -1200, 20, 0},

  // Combinations
  GimbalContainer{45, 0, 2500, 20, 1},
  GimbalContainer{-45, 0, -2500, 20, 1},
  GimbalContainer{0, -250, 1000, 20, 1},
  GimbalContainer{45, 0, 2500, 20, 0},
  GimbalContainer{-45, 0, 2500, 20, 0},
  GimbalContainer{0, 250, 1000, 20, 0},
  GimbalContainer{0, -250, 1000, 20, 0}
 ));

TEST_P(DJI_CameraTest, DISABLED_setGimbalAngle) {
  // Re-set gimbal
  gimbal = GimbalContainer(0,0,0,20,1);
  setGimbalAngleControl(gimbal);

  display();

  ResultContainer result = setGimbalAngleControl(GetParam());
  doPrecisionErrorTest(&result);
}

INSTANTIATE_TEST_CASE_P(
    DISABLED_ThreeAxisChange_SIM_OFF, DJI_CameraTest, testing::Values(
  // Test cases that will lead to locked state
  // if not reset
  GimbalContainer{50, 50, 50, 10, 1},
  GimbalContainer{-50, -50, -50, 10, 1},
  GimbalContainer{-50, 100, -500, 10, 1},
  GimbalContainer{100, -50, 550, 10, 0},
  GimbalContainer{35, 70, 2100, 10, 0},
  GimbalContainer{-50, 200, -3100, 10, 0},
  GimbalContainer{15, 15, 15, 10, 1},

  // Duration = 20
  GimbalContainer{50, 50, 50, 20, 1},
  GimbalContainer{-50, -50, -50, 20, 1},
  GimbalContainer{-50, 100, -500, 20, 1},
  GimbalContainer{100, -50, 550, 20, 0},
  GimbalContainer{35, 70, 2100, 20, 0},
  GimbalContainer{-50, 200, -3100, 20, 0},
  GimbalContainer{15, 15, 15, 20, 1}

/*  GimbalContainer{290, 250, 1000, 10, 0},
  GimbalContainer{-290, -250, -1000, 10, 0},
  GimbalContainer{280, 250, 1000, 10, 0},
  GimbalContainer{-280, -250, -1000, 10, 1},
  //GimbalContainer{290, 250, 1000, 20, 1},
  //GimbalContainer{-290, 250, -1000, 20, 1},
  GimbalContainer{280, 250, 1000, 10, 1},
  GimbalContainer{-280, -250, -1000, 10, 1},
  GimbalContainer{0, 250, 1000, 10, 1},


  GimbalContainer{290, 250, 1000, 20, 0},
  GimbalContainer{-290, -250, -1000, 20, 0},
  GimbalContainer{280, 250, 1000, 20, 0},
  GimbalContainer{-280, -250, -1000, 20, 1},
  //GimbalContainer{290, 250, 1000, 20, 1},
  //GimbalContainer{-290, 250, -1000, 20, 1},
  GimbalContainer{280, 250, 1000, 20, 1},
  GimbalContainer{-280, -250, -1000, 20, 1},
  GimbalContainer{0, 250, 1000, 20, 1}
*/ ));

TEST_P(DJI_CameraTest, DISABLED_ThreeAxisChange_SIM_OFF) {
  /** Dont't use simulator
   * In simulation mode, Gimbal rotation angle will
   * be set to [0,0,0]. We want to test with respect to
   * inertial frame
   */
  stop_simulator();

  // Re-set gimbal
  gimbal = GimbalContainer(0,0,0,20,1);
  setGimbalAngleControl(gimbal);

  display();

  ResultContainer result = setGimbalAngleControl(GetParam());
  doPrecisionErrorTest(&result);
}

INSTANTIATE_TEST_CASE_P(
    DISABLED_ThreeAxisChange_SIM_ON, DJI_CameraTest, testing::Values(
  // Test cases that will lead to locked state
  // if not reset
  GimbalContainer{50, 50, 50, 10, 1},
  GimbalContainer{-50, -50, -50, 10, 1},
  GimbalContainer{-50, 100, -500, 10, 1},
  GimbalContainer{100, -50, 550, 10, 0},
  GimbalContainer{35, 70, 2100, 10, 0},
  GimbalContainer{-50, 200, -3100, 10, 0},
  GimbalContainer{15, 15, 15, 10, 1},

  // Duration = 20
  GimbalContainer{50, 50, 50, 20, 1},
  GimbalContainer{-50, -50, -50, 20, 1},
  GimbalContainer{-50, 100, -500, 20, 1},
  GimbalContainer{100, -50, 550, 20, 0},
  GimbalContainer{35, 70, 2100, 20, 0},
  GimbalContainer{-50, 200, -3100, 20, 0},
  GimbalContainer{15, 15, 15, 20, 1} 
 ));

TEST_P(DJI_CameraTest, DISABLED_ThreeAxisChange_SIM_ON) {
  /** Use simulator
   * In simulation mode, Gimbal rotation angle will
   * be set to [0,0,0]. Inertial frame values will be
   * set to all zeroes
   */
  if(sim_control_enabled) {
    ASSERT_EQ(abs(camera->getGimbal().roll), 0);
    ASSERT_EQ(abs(camera->getGimbal().pitch), 0);
    ASSERT_EQ(abs(camera->getGimbal().yaw), 0);
  }

  // Re-set gimbal
  gimbal = GimbalContainer(0,0,0,20,1);
  setGimbalAngleControl(gimbal);

  display();

  ResultContainer result = setGimbalAngleControl(GetParam());
  doPrecisionErrorTest(&result);
}

