/*! @file LinuxCamera.cpp
 *  @version 3.1.9
 *  @date September 20 2016
 *
 *  @brief
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include <LinuxCamera.h>

void gimbalAngleControlSample(Camera *camera, int timeout) {
 
  GimbalContainer gimbal;  
  GimbalAngleData gimbalAngle;
  ResultContainer result;
  RotationAngle initialAngle = {};
  RotationAngle currentAngle = {};
  
  // Set control of the camera
  camera->setCamera(Camera::CAMERA_CODE::CODE_GIMBAL_ANGLE);

  std::cout << "Gimbal Angles Description [roll, pitch, yaw]: \n\n"
    << "Roll angle: unit 0.1º, input range [-350,350]\n"
    << "Pitch angle: unit 0.1º, input range [-900,300]\n"
    << "Yaw angle: unit 0.1º, input range [-3200,3200]\n\n"
    << "(NOTE: Yaw rotation angle represented in  [-π, π] range (see simulator output))\n\n";

  // Wait for Gimbal to sync
  waitForGimbal(camera);

  // Get Gimbal initial values
  initialAngle.roll = camera->getGimbal().roll;
  initialAngle.pitch = camera->getGimbal().pitch;
  initialAngle.yaw = camera->getGimbal().yaw;

  std::cout << "Initial Gimbal rotation angle: [" <<
      initialAngle.roll << ", " << initialAngle.pitch <<
      ", " << initialAngle.yaw << "]\n\n";

  // Re-set Gimbal to initial values
  gimbal.set(0,0,0,20,1,false,false,false,initialAngle,currentAngle);
  doSetGimbalAngle(camera, gimbal);
  result = calculatePrecisionError(camera,&gimbal);

  std::cout << "Setting new Gimbal rotation angle to [0,20,200] using incremental control:\n";
  // Get current gimbal data to calc precision error in post processing
  currentAngle.roll = camera->getGimbal().roll;
  currentAngle.pitch = camera->getGimbal().pitch;
  currentAngle.yaw = camera->getGimbal().yaw;
  
  gimbal.set(0,200,200,20,0,false,false,false,initialAngle,currentAngle);
  doSetGimbalAngle(camera, gimbal);
  result = calculatePrecisionError(camera,&gimbal);
  displayResult(&result);

  std::cout << "Setting new Gimbal rotation angle to [0,-50,-200] using absolute control:\n";
  gimbal.set(0,-500,-200,20,1,false,false,false,initialAngle,currentAngle);
  doSetGimbalAngle(camera, gimbal);
  result = calculatePrecisionError(camera,&gimbal);
  displayResult(&result);

  std::cout << "Setting new Gimbal rotation angle to [25,0,150] using absolute control:\n";
  gimbal.set(25,0,150,20,1,false,false,false,initialAngle,currentAngle);
  doSetGimbalAngle(camera, gimbal);
  result = calculatePrecisionError(camera,&gimbal);
  displayResult(&result);

  std::cout << "Setting new Gimbal rotation angle to [5,0,100] using incremental control:\n";
  // Get current gimbal data to calc precision error in post processing
  currentAngle.roll = camera->getGimbal().roll;
  currentAngle.pitch = camera->getGimbal().pitch;
  currentAngle.yaw = camera->getGimbal().yaw;

  gimbal.set(5,0,100,20,0,false,false,false,initialAngle,currentAngle);
  doSetGimbalAngle(camera, gimbal);
  result = calculatePrecisionError(camera,&gimbal);
  displayResult(&result);

  // Re-set Gimbal to initial values so that current Gimbal rotation 
  // angle does not effect further Linux samples and a use does not
  // have to power cycle an aircraft to re-set the rotation angle  
  gimbal.set(0,0,0,20,1,false,false,false,initialAngle,currentAngle);
  doSetGimbalAngle(camera, gimbal);
  result = calculatePrecisionError(camera,&gimbal);
  
  // Re-set struct values
  gimbal = {};
  result = {};
}

/**
 * @brief
 *
 * Roll - unit 0.1 degrees/second input rate [-1800, 1800]
 * Pitch - unit 0.1 degrees/second input rate [-1800, 1800]
 * Yaw - unit 0.1 degrees/second input rate [-1800, 1800]
 */
void gimbalSpeedControlSample(Camera* camera, int timeout) {
  // Set control of the camera
  camera->setCamera(Camera::CAMERA_CODE::CODE_GIMBAL_SPEED);

  GimbalSpeedData gimbalSpeed;

  std::cout << "Gimbal Speed Description: \n\n"
    << "Roll - unit 0.1 degrees/second input rate [-1800, 1800]\n"
    << "Pitch - unit 0.1 degrees/second input rate [-1800, 1800]\n"
    << "Yaw - unit 0.1 degrees/second input rate [-1800, 1800]\n\n";

  std::cout << "Setting Roll rate to 50.\n";
  gimbalSpeed.roll = 500;
  gimbalSpeed.pitch = 0;
  gimbalSpeed.yaw = 0;

  camera->setGimbalSpeed(&gimbalSpeed);

  // Give time for gimbal to sync
  sleep(4);

  std::cout << "Setting Pitch rate to 100.\n";
  gimbalSpeed.roll = 0;
  gimbalSpeed.pitch = 1000;
  gimbalSpeed.yaw = 0;

  camera->setGimbalSpeed(&gimbalSpeed);

  // Give time for gimbal to sync
  sleep(4);

  std::cout << "Setting Yaw rate to 150.\n";
  gimbalSpeed.roll = 0;
  gimbalSpeed.pitch = 0;
  gimbalSpeed.yaw = 1500;

  camera->setGimbalSpeed(&gimbalSpeed);

  // Give time for gimbal to sync
  sleep(4);
}

void takePictureControl(Camera *camera, int timeout) {
  std::cout << "Ensure SD card is present.\n";
  // Set control of the camera
  camera->setCamera(Camera::CAMERA_CODE::CODE_CAMERA_SHOT);
  std::cout << "Check DJI GO App or SD card for a new picture.\n";
}

void takeVideoControl(Camera *camera, int timeout) {
  std::cout << "Ensure SD card is present.\n";
  // Set control of the camera
  camera->setCamera(Camera::CAMERA_CODE::CODE_CAMERA_VIDEO_START);

  // Take video for 5 seconds
  sleep(5);

  // Set control of the camera
  camera->setCamera(Camera::CAMERA_CODE::CODE_CAMERA_VIDEO_STOP);
  std::cout << "Check DJI GO App or SD card for a new video.\n";
}

void waitForGimbal(Camera *camera) {
  RotationAngle rAngle;

  std::cout << "Waiting for Gimbal to sync rotation angle...\n\n";

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

void doSetGimbalAngle(Camera *camera, GimbalContainer gimbal) {
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

  camera->setGimbalAngle(&gimbalAngle);
  // Give time for gimbal to sync
  sleep(4);
}

ResultContainer calculatePrecisionError(Camera *camera, GimbalContainer *gimbal) {
  ResultContainer result;
  int expectedAngle;

  // Calculate precision error
  for(int i = 0; i < 3; i++) {
    if(i == 0){
      result.angle[i] = camera->getGimbal().roll;
      if(gimbal->isAbsolute)
        expectedAngle = calculateAngle(gimbal->initialAngle.roll, gimbal->roll);
      else
	expectedAngle = calculateAngle(gimbal->currentAngle.roll, gimbal->roll);
    }else if(i == 1){
      result.angle[i] = camera->getGimbal().pitch;
      if(gimbal->isAbsolute)
        expectedAngle = calculateAngle(gimbal->initialAngle.pitch, gimbal->pitch);
      else
	expectedAngle = calculateAngle(gimbal->currentAngle.pitch, gimbal->pitch);
    }else if( i == 2){
      result.angle[i] = camera->getGimbal().yaw;
      if(gimbal->isAbsolute)
        expectedAngle = calculateAngle(gimbal->initialAngle.yaw, gimbal->yaw);
      else
	expectedAngle = calculateAngle(gimbal->currentAngle.yaw, gimbal->yaw);
    }
    result.error[i] = abs(expectedAngle - result.angle[i]);
  }
  return result;
}

/**
 * @brief
 *
 * Keep angle precision to integer values
 */
int calculateAngle(int currentAngle, int newAngle) {
  int n = (newAngle / 10) + currentAngle;
  int absAngle = std::abs(n);

  if(absAngle > 180 && n < 0)
    return n + 360;
  else if (absAngle > 180 && n > 0)
    return n - 360;
  else
    return n;
}

void displayResult(ResultContainer *resultContainer) {
  std::cout << "New Gimbal rotation angle is [";
  for(int i = 0; i < 3; i++)
    std::cout << resultContainer->angle[i] << " ";

  std::cout << "], ";

  std::cout << "with precision error: [";
  for(int i = 0; i < 3; i++)
    std::cout << resultContainer->error[i] << " ";

  std::cout << "]\n\n";
}

