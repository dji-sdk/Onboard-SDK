/*! @file TrajectoryFollower.h
 *
 *  @brief
 *  See TrajectoryFollower class.
 *
 *  @copyright 2016 DJI Research LLC. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef ONBOARDSDK_FOLLOWTRAJECTORY_H
#define ONBOARDSDK_FOLLOWTRAJECTORY_H

#include "OnboardSDK.h"
#include "CartesianFrame.h"
#include "curve.h"
#include "SE3Controller.h"

//! @brief Function for following a trajectory. Uses CoreAPI, Flight, LocalFrame and Curve classes.
//! @details This function iterates over a sampled trajectory (assumed to be feasible) and calls
//! a position controller to reach each desired setpoint.
//! As a user, you will not have to interface with this class.
class TrajectoryFollower{
 public:
  int followTrajectory(DJI::onboardSDK::CoreAPI *osdkApi, DJI::onboardSDK::Flight *flight, CartesianFrame *localFrame,
                       Curve *trajectory, DJI::onboardSDK::Camera *camera);

};



#endif //ONBOARDSDK_FOLLOWTRAJECTORY_H
