/*! @file ParamtericSpiral.h
 *
 *  @brief
 *  See ParamtericSpiral class.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef ONBOARDSDK_PARAMETRICSPIRAL_H
#define ONBOARDSDK_PARAMETRICSPIRAL_H

#include <curve.h>
#include <cmath>
#include <fstream>
#include <rapidjson/document.h>


/*!@brief: This class has methods for generating a geolocated, analytical spiral
 * given a set of parameters.
 * @details: Also has methods for checking feasibility of the planned spiral and for reading json files
 */
//! As a user, you do not have to interface with these functions directly.
class ParametricSpiral {
 public:
  ParametricSpiral(double start_radius, double end_radius, double start_angle, int rotations, double horizontal_speed, double vertical_speed, Curve* spiralTrajectory);
  ParametricSpiral(const std::string &spiralParamFile, Curve* spiralTrajectory);
  void generate();
  bool checkFeasibility();
  void readJson();

 private:
  void generateEndTime();
  void validateHeights();
  double xFunc(double theta);
  double yFunc(double theta);
  double radius(double theta);
  double tFunc(double theta);
  double thetaFunc(double t);
  double zFunc(double theta);

  std::string _spiralParamFile;
  Curve* _spiralTrajectory;
  double _start_radius;
  double _end_radius;
  double _start_angle;
  int _rotations;
  double _horizontal_speed;
  double _pitch;
  double _end_time;
  double _start_agl;

  //! Temp parameters
  double _xOffset;
  double _yOffset;
  //! This is used to pick between MSL and AGL
  double _zOffset;

  //! GPs handling
  double _gps_lat;
  double _gps_lon;
  double _gps_msl;

  bool _isMslValid;
  bool _isAglValid;
};

#endif //ONBOARDSDK_PARAMETRICSPIRAL_H
