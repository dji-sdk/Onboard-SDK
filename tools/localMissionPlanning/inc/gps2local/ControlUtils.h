/*! @file ControlUtils.h
 *
 *  @brief
 *  A variety of tools for converting between frames and evaluating relevant quantities for vector algebra and rotations.
 *  Eigen-compatible.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef _CONTROLUTILS_H
#define _CONTROLUTILS_H

#include <math.h>
#include <Eigen/Core>
#include "NavigationConstants.h"

#define SIGMA                1.0e-8
#define sign(arg)            (arg>=0 ? 1:-1)

//! As a user, you do not have to interface with these functions.
namespace ControlUtils {

Eigen::Vector3d lla2ecef(Eigen::Vector3d LLA);
Eigen::Vector3d ecef2lla(Eigen::Vector3d xyz);

Eigen::Matrix3d vec2mat(Eigen::Vector3d vec);
Eigen::Matrix3d cECEF2NED(Eigen::Vector3d LLA);
Eigen::Vector3d earthRateNED(Eigen::Vector3d LLA);
Eigen::Vector3d transportRateNED(Eigen::Vector3d llaDot, Eigen::Vector3d LLA);
Eigen::Vector3d gravityNED(Eigen::Vector3d LLA);

double psiGPSTrue(Eigen::Vector3d velNED);

double distancePt2LineSegment(Eigen::Vector3d pt, Eigen::Vector3d ls1, Eigen::Vector3d ls2);
double proportionPtAlongProjectedLine(Eigen::Vector3d pt, Eigen::Vector3d ls1, Eigen::Vector3d ls2);
double angleBetweenVectors(Eigen::Vector3d v1, Eigen::Vector3d v2);
double curvatureBetweenVectors(Eigen::Vector3d firstPt, Eigen::Vector3d lastPt, double angleBetweenVectors);

double boundTo180(double angle);
double sat(double input, double bound_value);
void printM(Eigen::MatrixXd tmp);

};

#endif


