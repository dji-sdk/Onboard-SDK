/*! @file CartesianFrame.h
 *
 *  @brief
 *  See CartesianFrame class
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef CARTESIAN_FRAME_H
#define CARTESIAN_FRAME_H

#include <math.h>
#include <Eigen/Core>
#include "ControlUtils.h"
#include "NavigationConstants.h"


//! @brief CartesianFrame class handles storing a cartesian coordinate frame origin and then converting LLA (spherical coordinates) to that frame
//! As a user, you do not have to interface with this class.
class CartesianFrame {

 public:
  CartesianFrame();
  CartesianFrame(Eigen::Vector3d originLLAIn);
  ~CartesianFrame();

  //set the origin with a LLA vector
  void setOriginLLA(Eigen::Vector3d originLLAIn);

  //get the cartesian frame xyz position vector given an LLA vector
  Eigen::Vector3d getPosition(Eigen::Vector3d posLLAIn);  //(const Eigen::MatrixXf& posLLAIn);

  //get the previously set LLA origin
  Eigen::Vector3d getOriginLLA();

  //get notification if local origin is set or not
  bool getisSet();

 private:
  Eigen::Vector3d originLLA;  //the local plane origin in ECEF frame
  bool isSet;  //has the origin been set?

};

#endif /* CARTESIAN_FRAME_H */
