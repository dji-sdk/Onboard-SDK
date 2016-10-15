/*! @file NavigationConstants.h
 *
 *  @brief
 *  Useful navigational constants for going back and forth between various coordinate frames
 *
 *  @copyright 2016 DJI. All rights reserved.
 *  Unauthorized copying of this file via any medium is strictly prohibited.
 *  Proprietary and confidential.
 */

#ifndef _NavigationConstants_h
#define _NavigationConstants_h

//! As a user, you do not need to use these constants.
//@formatter:off
#define r2d                   (180.0/M_PI)
#define d2r                   (M_PI/180.0)
#define A_EARTH               6378137.0
#define flattening            (1/298.257223563)
#define NAV_E2                ((2-flattening)*flattening)
#define E_MAJOR               A_EARTH
#define E_MINOR               6.356752314245216e+006
#define E_MAJ2E_MIN20E_MIN    4.284131151324081e+004
#define E_ECC2EMAJOR          4.269767270710535e+004
#define E_ECC2                6.694379990129618e-003
#define EarthRateECEF         7.292115e-5
#define GRAVITY               9.8066500
//@formatter:on

#endif
