/*! @file TrajectoryRecorder.h
 *
 *  @brief
 *  See TrajectoryRecorder class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef TRAJECTORYRECORDER_H
#define TRAJECTORYRECORDER_H

#include <DJI_API.h>
#include <DJI_Type.h>
#include <CartesianFrame.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Core>

//! Not supported in the current release
class TrajectoryRecorder {
 public:
  TrajectoryRecorder(){};
  TrajectoryRecorder(DJI::onboardSDK::CoreAPI* api);
  FILE * createFile();
  void record();
 private:
  DJI::onboardSDK::CoreAPI* api;
  bool continueRecording = false;
};

#endif //ONBOARDSDK_INTERNAL_TRAJECTORYRECORDER_H
