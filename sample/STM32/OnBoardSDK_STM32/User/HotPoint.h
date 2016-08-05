/*! @file HotPoint.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Function to use the hotpoint functionality of the onboard SDK.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef HOTPOINT_H
#define HOTPOINT_H

#include "main.h"

void tryHotpoint(float64_t altitude_m, float32_t angular_speed_deg_per_s, float32_t radius_m);
void stopHotpoint();


#endif //HOTPOINT_H

