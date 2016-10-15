/*! @file TrajectoryInfrastructure.h
 *
 *  @brief
 *  See TrajectoryInfrastructure class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef ONBOARDSDK_TRAJECTORY_INFRASTRUCTURE_H
#define ONBOARDSDK_TRAJECTORY_INFRASTRUCTURE_H

//! Declare helper functions

#include <SE3Controller.h>
#include <OnboardSDK.h>
#include <curve.h>
#include <TrajectoryFollower.h>
#include <TrajectoryRecorder.h>
#include <ParametricSpiral.h>

#ifndef C_PI
#define C_PI (double) 3.141592653589793
#endif

//! Wrappers for all parts of the trajectory functionality.
//! These are the functions you will call from your programs linking to this library
namespace TrajectoryInfrastructure {
  //! Handle state broadcast data from OSDK
  int startStateBroadcast(DJI::onboardSDK::CoreAPI *api);
  //! Not supported in this release - Trajectory Recorder state broadcast settings.
  int startRecordBroadcast(DJI::onboardSDK::CoreAPI *api);
  //! Not supported in this release - This function takes among other things a csv file
  //! as an argument and executes that mission
  void executeFromCsv(CoreAPI *api, Flight *flight, Camera *camera, char *trajFile, double timescale);
  //! This function takes in a json file with parameters for generating a trajectory
  void executeFromParams(CoreAPI *api, Flight *flight, CartesianFrame* localFrame, Eigen::Vector3d originLLA, Camera *camera, std::string trajParamsFile);
  //! Not supported in this release - Farm out the recorder to a different thread
  void *recordInThread(void *recorderPtr);
}



#endif //ONBOARDSDK_TRAJECTORY_INFRASTRUCTURE_H
