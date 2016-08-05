/*! @file HotPoint.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Function to use the hotpoint functionality of the onboard SDK.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "HotPoint.h"

extern CoreAPI *coreApi;
HotPoint hotpoint(coreApi);
GPSPositionData myGPSData;
HotPointData myHotPointData;

void tryHotpoint(float64_t altitude_m, float32_t angular_speed_deg_per_s, float32_t radius_m)
{

  myGPSData.altitude = altitude_m;
  myGPSData.latitude = coreApi->getBroadcastData().pos.latitude;
  myGPSData.longitude = coreApi->getBroadcastData().pos.longitude;

  hotpoint.setHotPoint(myGPSData);
  hotpoint.setYawRate(angular_speed_deg_per_s);
  hotpoint.setRadius(radius_m);
  hotpoint.start();
}

void stopHotpoint()
{
  hotpoint.stop();
}
