/*! @file SE3Controller.h
 *
 *  @brief
 *  See SE3Controller class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef SE3CONTROLLER_H
#define SE3CONTROLLER_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#ifdef DEBUG_CONTROLLER
#include <iostream>
#endif

#define THRUST_SCALE (float) 100/65

//! @brief SE(3) Position Controller.
//! @details Outer loop of SE(3) Controller as defined in T. Lee, M. Leok, N. McClamroch (2010).
//! Inner Loop is run using DJI Onboard SDK Movement Control
//! As a user, you do not have to interface with these functions.
class SE3Controller {
 public:
  Eigen::Vector4d trackPosition(Eigen::Vector3d xDesired,
                                Eigen::Vector3d xCurrent,
                                Eigen::Vector3d x_dotDesired,
                                Eigen::Vector3d x_dotCurrent,
                                Eigen::Vector3d x_dot_dotDesired,
                                Eigen::Quaterniond qCur,
                                float psiDes);
  SE3Controller();
  void setKp(Eigen::Vector3d userKp) { Kp = userKp; }
  void setKd(Eigen::Vector3d userKd) { Kd = userKd; }
  void setMass(float userMass) { m = userMass; }

 private:
  float m;
  float g;
  Eigen::Vector3d Kp;
  Eigen::Vector3d Kd;
};

#endif