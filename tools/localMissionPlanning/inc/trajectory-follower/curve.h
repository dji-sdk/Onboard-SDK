/*! @file curve.h
 *
 *  @brief
 *  See Curve class.
 *
 *  @copyright 2016 DJI Research LLC. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef DJI_CURVE_H
#define DJI_CURVE_H

#include <string>
#include <vector>
#include <FeasibleTrajectory.h>
#include <CartesianFrame.h>
#include <Eigen/Core>
#include <csv.h>
#include <R3Trajectory.h>
#include <iostream>

#define C_PI (double) 3.141592653589793

typedef struct {
  double tau;
  double x;
  double y;
  double z;
  double yaw;
} CurvePtPos;

typedef struct {
  double tau;
  double vx;
  double vy;
  double vz;
} CurvePtVel;

typedef struct {
  double tau;
  double ax;
  double ay;
  double az;
} CurvePtAcc;

//! @brief The Curve class is the core of the DJI Onboard SDK trajectory follower.
//! As a user, you do not have to interface with this class.
class Curve {
  friend class ParametricSpiral;
 public:
  // initialize the trajectory from file.
  Curve(std::string curve_path);
  Curve(){};

  // Read CSV file containing info about curve
  void readCsv(std::string curve_path);

  // Check if GPS coordinates for the curve exist in the file
  void handleGPS(char *gpsLine);

  // Change the coordinates of the curve if you need to move to GPS
  void moveToGpsLocation(Eigen::Vector3d originLLA);

  // retrieve a point a given time.
  // return: 
  //    CurvePtPos (x,y,z,yaw) at time t.
  //    yaw is guaranteed to be between [-\pi, pi).
  CurvePtPos pos_at(double t);

  // retrieve (numerical derivative) velocity at a given time.
  CurvePtVel vel_at(double t);

  // retrieve (numerical derivative) acceleration at time t.
  CurvePtAcc acc_at(double t);

  // whether time t is in the trajectory interval.
  bool contains(double t);

  // re-center the curve if first pt is not (0, 0, 0).
  void recenter();

  // Timescale
  void timescale(double speedfactor);

  // Validate timescale - see if the desired trajectory is too aggressive.
  double validate_timescale();

  // Add ramp-up for getting to the start of the trajectory - should this really be done here?
  void addRampUp(CurvePtPos curPos, CurvePtVel curVel);

  //Get to start point
  void getToStartPoint(CurvePtPos curPos);

  //Getter functions
  CurvePtPos getStartPt() { return start_pt; }
  double getStartTime() { return time_start; }
  double getEndTime() { return time_end; }
  bool getTakePictures() {return takePictures; }
  int getPictureIntervalSec() {return pictureIntervalSec; }
  bool getTakeVideo() {return takeVideo; }
  double getRampUpTime() { return rampUpTime; }

  //Print GPS location of the desired curve
  void printGPS();

 private:
  // Data from file goes here
  std::vector<CurvePtPos> path;
  std::vector<CurvePtVel> pathVel;
  std::vector<CurvePtAcc> pathAcc;

  //Checks to see if these values have been set
  bool csvHasDerivatives;

  //Picture taking
  bool takePictures;
  double pictureIntervalSec;
  //Video
  bool takeVideo;

  std::vector<CurvePtPos> scaledTrajectory;

  //Time bookkeeping
  double time_start, time_end;
  int time_i;
  double rampUpTime;

  CurvePtPos start_pt;

  //Member variables for GPS location of curve
  double latTraj;
  double lonTraj;
  double altTraj;
  bool gpsSet;
};

#endif //DJI_TRAJECTORY_H
