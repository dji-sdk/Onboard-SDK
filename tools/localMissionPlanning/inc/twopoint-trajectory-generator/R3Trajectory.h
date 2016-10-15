/*! @file R3Trajectory.h
 *
 *  @brief
 *  See R3Trajectory class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef R3TRAJECTORY_H
#define R3TRAJECTORY_H

#include "FeasibleTrajectory.h"

//! DO NOT INCREASE THESE. The drone will attempt to execute infeasible maneuvers.
#define VEL_LIMIT 18
#define Z_VEL_LIMIT 5

//! @brief This class ties together x,y,z feasible trajectories and evaluates them for combined feasiblity
//! As a user, you do not need to interface with this class.
class R3Trajectory {
 public:
  //Constructor takes in initial and final conditions for all three R3 trajectories
  R3Trajectory(Eigen::Matrix<double, 6, 1> xInitFinalCond,
               Eigen::Matrix<double, 6, 1> yInitFinalCond,
               Eigen::Matrix<double, 6, 1> zInitFinalCond);

  //Equalize the times for all three trajectories
  void equalizeTimes();

  //Make the overall acc and vel feasible
  bool makeR3AccFeasible();
  bool makeR3VelFeasible();

  //Get pointers to the individual trajectories
  FeasibleTrajectory *getXTrajectory() { return &xTrajectory; }
  FeasibleTrajectory *getYTrajectory() { return &yTrajectory; }
  FeasibleTrajectory *getZTrajectory() { return &zTrajectory; }

  //Setters & getters for end time
  double getEndTime() { return endTime; }
  void setEndTime(double desiredEndTime) { endTime = desiredEndTime; }

 private:
  FeasibleTrajectory xTrajectory;
  FeasibleTrajectory yTrajectory;
  FeasibleTrajectory zTrajectory;

  double endTime;
};

#endif //R3TRAJECTORY_H
