/** @file dji_mission_base.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Mission-Base abstract class for DJI OSDK library
 *  @details This is a low-level abstraction for having
 *  commonality between all missions.
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef ONBOARDSDK_DJI_MISSIONBASE_H
#define ONBOARDSDK_DJI_MISSIONBASE_H

#include "dji_command.hpp"
#include "dji_type.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

typedef enum MissionType {
  MISSION_MODE_A,
  MISSION_WAYPOINT,
  MISSION_HOTPOINT,
  MISSION_FOLLOW,
  MISSION_IOC
} MissionType;

class Vehicle;

/*! @brief Mission Base class for commonality between SDK Missions.
 *
 * @details You can inherit from this class if making a custom mission.
 */
class MissionBase
{
public:
  MissionBase(Vehicle* vehicle = 0)
    : vehicle(vehicle)
  {
  }
  virtual ~MissionBase()
  {
  }

  virtual void start(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode start(int timer) = 0;

  virtual void stop(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode stop(int timer) = 0;

  virtual void pause(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode pause(int timer) = 0;

  virtual void resume(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode resume(int timer) = 0;

protected:
  Vehicle* vehicle;
}; // class MissionBase

} // OSDK
} // DJI

#endif // ONBOARDSDK_DJI_MISSIONBASE_H
