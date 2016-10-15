/*! @file LocalNavigation.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Functions to implement navigation on local frame.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef SAMPLE_STM32_ONBOARDSDK_STM32_USER_LOCALNAVIGATION_H_
#define SAMPLE_STM32_ONBOARDSDK_STM32_USER_LOCALNAVIGATION_H_

#include "DJI_Type.h"
using namespace DJI::onboardSDK;

#define C_EARTH ((float32_t) 6378137.0f)
#define C_PI ((float32_t) 3.14159265359f)
#define DEG2RAD(DEG) ((float32_t)((DEG)*((C_PI)/(180.0f))))

enum LOCAL_NAV
{
  LOCAL_NAV_SUCCESS = 0,
  LOCAL_NAV_FAIL = 1
};

class LocalPosition
{
public:
  float32_t x;
  float32_t y;
  float32_t z;

  LocalPosition() {x=0.0f; y=0.0f; z=0.0f;}
  LocalPosition(float32_t x0, float32_t y0, float32_t z0)
  {
    x=x0; y=y0; z=z0;
  }
};

typedef struct
{
  PositionData currentPositionLLA;
  PositionData homePositionLLA;

  FlightStatus prevFlightStatus;
  FlightStatus curFlightStatus;

  uint32_t     homePositionSetFlag;

  float32_t    roseCurveTheta;
  uint32_t     localNavExampleRunningFlag;

} LocalNavigationStatus;


//void gps_convert_ned_deg(float32_t &ned_x, float32_t &ned_y,
//    float32_t gps_t_lon, float32_t gps_t_lat,
//    float32_t gps_r_lon, float32_t gps_r_lat);


//void gps_convert_ned_rad(float32_t &ned_x, float32_t &ned_y,
//    float32_t gps_t_lon, float32_t gps_t_lat,
//    float32_t gps_r_lon, float32_t gps_r_lat);

//LocalPosition gps_convert_ned(PositionData loc, PositionData homeLocationLLA);

uint32_t startLocalNavExample();
uint32_t stopLocalNavExample();
void updateCurveFollowing();
void myRecvCallback(DJI::onboardSDK::CoreAPI * myApi, Header *myHeader, DJI::UserData myUserData);


#endif /* SAMPLE_STM32_ONBOARDSDK_STM32_USER_LOCALNAVIGATION_H_ */
