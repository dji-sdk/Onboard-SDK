/*! @file FeasibleTrajectory.h
 *
 *  @brief
 *  See FeasibleTrajectory class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef FEASIBLETRAJECTORY_H
#define FEASIBLETRAJECTORY_H

#define EPS 1e-6
#define grav 9.81

#define SAFETY_MARGIN 1.0
#define ACC_LIMIT (M_PI*grav*sqrt(2)/6 - SAFETY_MARGIN)

#include <iostream>
#include <Eigen/Dense>
#include "Poly5Trajectory.h"


//! @brief This class takes a Poly5Trajectory and makes it satisfy initial & final conditions, as well as dynamic constraints
//! As a user, you do not need to interface with this class.
class FeasibleTrajectory {
 public:
  // Constructor takes in initial and final conditions (pos, vel, acc) for a two-point feasible trajectory
  FeasibleTrajectory(Eigen::Matrix<double, 6, 1> initFinalCond);

  // This function will attempt to satisfy dynamic feasibility by rescaling if needed.
  // Return values: 0 => no changes made, traj was safe; 1 => changes made, traj is now safe;
  // -1 => changes cannot be made, traj unattainable at final condition
  int satisfyAccFeasibility();

  // Solve linear system to get coefficients for polynomial w.r.t initial & final conditions and a chosen end time.
  void solveLinearSystem();

  // Generate an end time for the trajectory using initial and final conditions as heuristics
  void generateCandidateTf();

  // Set the constraint matrix for the linear system
  void setConstraints(Eigen::Matrix<double, 6, 1> initFinalCond) { constrCoeffMatB = initFinalCond; }

  // Get the A matrix of the linear system (this is composed of nonlinear combinations of polynomial coeffs and end time)
  void generateMatA();

  //Solve a standard quadratic equation.
  Eigen::Vector2d solveQuadratic(double a, double b, double c, double &realRootsFlag);

  //Evaluate extrema for validity using our maxima-finding algorithm
  bool checkIfValidMaxima(double t_query);

  //This is the only function external users need to call after the constructor. Does as the name says.
  bool makeFeasible();

  //Accessor for current end time
  double getCurrentTf() { return candidate_tf; }

  //Setter for end time
  void setCandidateRampupTime(double user_tf) { candidate_tf = user_tf; }

  //Getter for the raw polynomial trajectory
  Poly5Trajectory *getTraj() { return &rawTraj; }

 private:
  // The candidate final time candidate_tf is the time, starting from now, that we will
  // reach the start of the desired trajectory
  double candidate_tf;

  // The matrix A appears in the linear system Ax=b obtained from the constraints on the polynomial
  // x is the vector of coefficients for the 5th order polynomial
  // A is the matrix of coefficients for this linear system
  // (do not confuse these coeffs with those in vector x; those are variables for this system)
  // B is the vector of initial and final conditions that comprise the RHS
  Eigen::Matrix<double, 6, 6> constrCoeffMatA;
  Eigen::Matrix<double, 6, 1> constrCoeffMatB;
  // This is the trajectory without checks on feasibility
  Poly5Trajectory rawTraj;
};

#endif //FEASIBLETRAJECTORY_H
