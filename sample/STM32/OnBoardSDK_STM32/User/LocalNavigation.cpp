/*! @file LocalNavigation.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Functions to implement navigation on local frame.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "LocalNavigation.h"

#include "DJI_API.h"
#include "arm_math.h"
#include "math.h"
#include "main.h"

using namespace DJI::onboardSDK;
extern Flight flight;

uint8_t myFreq[16] =
{
  BROADCAST_FREQ_10HZ, //0 - Timestamp
  BROADCAST_FREQ_10HZ, //1 - Attitude Quaterniouns
  BROADCAST_FREQ_10HZ, //2 - Acceleration
  BROADCAST_FREQ_50HZ, //3 - Velocity (Ground Frame)
  BROADCAST_FREQ_10HZ, //4 - Angular Velocity (Body Frame)
  BROADCAST_FREQ_50HZ, //5 - Position
  BROADCAST_FREQ_10HZ, //6 - Magnetometer
  BROADCAST_FREQ_10HZ, //7 - RC Channels Data
  BROADCAST_FREQ_10HZ, //8 - Gimbal Data
  BROADCAST_FREQ_10HZ, //9 - Flight Status
  BROADCAST_FREQ_10HZ, //10 - Battery Level
  BROADCAST_FREQ_10HZ  //11 - Control Information
};

//static void gps_convert_ned_deg(float32_t &ned_x, float32_t &ned_y,
//    float32_t gps_t_lon, float32_t gps_t_lat,
//    float32_t gps_r_lon, float32_t gps_r_lat)
//{
//  float32_t d_lon = gps_t_lon - gps_r_lon;
//  float32_t d_lat = gps_t_lat - gps_r_lat;
//  ned_x = DEG2RAD(d_lat) * C_EARTH;
//  ned_y = DEG2RAD(d_lon) * C_EARTH * arm_cos_f32(DEG2RAD(gps_t_lat));
//};

static void gps_convert_ned_rad(float32_t &ned_x, float32_t &ned_y,
    float32_t gps_t_lon, float32_t gps_t_lat,
    float32_t gps_r_lon, float32_t gps_r_lat)
{
  float32_t d_lon = gps_t_lon - gps_r_lon;
  float32_t d_lat = gps_t_lat - gps_r_lat;
  ned_x = d_lat * C_EARTH;
  ned_y = d_lon * C_EARTH * arm_cos_f32(gps_t_lat);
};

/*
 * @brief Convert from GPS Lat Lon Alt to local Cartesian coordinate with origin given by homeLocationLLA
 */
static LocalPosition gps_convert_ned(PositionData loc, PositionData homeLocationLLA)
{
  LocalPosition local;
  gps_convert_ned_rad(local.x, local.y,
      (float32_t)loc.longitude, (float32_t)loc.latitude,
      (float32_t)homeLocationLLA.longitude, (float32_t)homeLocationLLA.latitude
    );
    local.z = loc.altitude;
    return local;
}

LocalNavigationStatus droneState;

void myRecvCallback(DJI::onboardSDK::CoreAPI * myApi, Header *myHeader, DJI::UserData myUserData)
{
  unsigned char *pdata = ((unsigned char *)myHeader) + sizeof(Header);
  unsigned short *enableFlag;

  LocalNavigationStatus *s = (LocalNavigationStatus *)myUserData;

  if ((*(pdata) == SET_BROADCAST) && (*(pdata+1) == CODE_BROADCAST))
  {
    pdata += 2;
    enableFlag = (unsigned short *)pdata;

    if ( (*enableFlag) & (1<<5) ) //bit5: GPS location
    {
      s->currentPositionLLA = myApi->getBroadcastData().pos;
      if (s->localNavExampleRunningFlag == 1 &&
          s->curFlightStatus == 3 &&
          s->homePositionSetFlag ==1)
      {
        updateCurveFollowing();
      }
    }

    if ( (*enableFlag) & (1<<9) ) //bit 9 Flight Status
    {
      s->prevFlightStatus = s->curFlightStatus;
      s->curFlightStatus  = myApi->getFlightStatus();

      if ((s->prevFlightStatus == 1) && (s->curFlightStatus == 3 ))
      {
        s->homePositionLLA = myApi->getBroadcastData().pos;
        if(s->homePositionSetFlag == 0)
        {
          printf("Home Location Set!\r\n");
        }
        else
        {
          printf("Home Location Updated!\r\n");
        }
        s->homePositionSetFlag = 1;
      }
    }
  }
}

/*
 * @brief Follow a rose shaped curve at height 25m using movement control
 */
void updateCurveFollowing()
{
  if (droneState.currentPositionLLA.height < 24.5f)
  {
    flight.setMovementControl(0x91, 0.0f, 0.0f, 25.0f, 0.0);
    return;
  }
  else
  {
    droneState.roseCurveTheta += 0.002f;
    float32_t rtgt = 25.0f * arm_sin_f32(2.0f*droneState.roseCurveTheta);
    float32_t xtgt = rtgt * arm_cos_f32(droneState.roseCurveTheta);
    float32_t ytgt = rtgt * arm_sin_f32(droneState.roseCurveTheta);

    LocalPosition lp = gps_convert_ned(droneState.currentPositionLLA, droneState.homePositionLLA);

    flight.setMovementControl(0x91, xtgt-lp.x, ytgt-lp.y, 25.0f, 0.0);

    if (droneState.roseCurveTheta >  6.28f)
    {
      droneState.roseCurveTheta =  0.0f;
      droneState.localNavExampleRunningFlag = 0;
    }
  }
}

/*
 * @brief To start local navigation (curve following), you need to take off first.
 */
uint32_t startLocalNavExample()
{
  if (droneState.homePositionSetFlag != 1 ||
      droneState.currentPositionLLA.height < 0.5f ||
      droneState.curFlightStatus != 3)
  {
    droneState.roseCurveTheta =  0.0f;
    droneState.localNavExampleRunningFlag = 0;
    printf("Please take off first!\r\n");
    return LOCAL_NAV_FAIL;
  }
  else
  {
    droneState.roseCurveTheta =  0.0f;
    droneState.localNavExampleRunningFlag = 1;
    return LOCAL_NAV_SUCCESS;
  }
}

uint32_t stopLocalNavExample()
{
  droneState.roseCurveTheta =  0.0f;
  droneState.localNavExampleRunningFlag = 0;
  return 0;
}
