/*! @file poly5Trajectory.h
 *
 *  @brief
 *  See Poly5Trajectory class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef POLY5TRAJECTORY_H
#define POLY5TRAJECTORY_H

#include <Eigen/Core>

//! @brief This class represents the smallest unit of the trajectory - a 5th order polynomial
//! As a user, you do not need to interface with this class.
class Poly5Trajectory {
 private:
  //Polynomial Coefficients
  double a, b, c, d, e, f;
  //End time for trajectory. Start time assumed to be 0
  double t_f;

 public:
  //Constructor with all polynomial params
  Poly5Trajectory(double user_a, double user_b, double user_c, double user_d, double user_e, double user_f, double user_tF);
  //Constructor with Eigen
  Poly5Trajectory(Eigen::Matrix<double, 6, 1> coeffVec, double user_tf);
  //Lazy constructor - please use setters
  Poly5Trajectory() {};

  //Getters for time-parameterized trajectory and its various derivatives
  double getPos(double tQuery);
  double getVel(double tQuery);
  double getAcc(double tQuery);
  double getJerk(double tQuery);
  double getSnap(double tQuery);

  //Setters for the various individual coefficients.
  void setA(double user_a) { a = user_a; }
  void setB(double user_b) { b = user_b; }
  void setC(double user_c) { c = user_c; }
  void setD(double user_d) { d = user_d; }
  void setE(double user_e) { e = user_e; }
  void setF(double user_f) { f = user_f; }
  void setTf(double user_tf) { t_f = user_tf; }

  //Setter for coefficients using Eigen Matrix
  void setCoeffs(Eigen::Matrix<double, 6, 1> coeffVec, double user_tf);

  //Getter for coefficients as Eigen Matrix
  Eigen::Matrix<double, 6, 1> getCoeffs();
};

#endif //POLY5TRAJECTORY_H
