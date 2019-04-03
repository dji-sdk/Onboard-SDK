/** @file dji_mission_manager.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Mission-Manager API for DJI OSDK library
 *  @details This is a high-level abstraction for handling/chaining missions
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

#ifndef ONBOARDSDK_DJI_MISSIONMANAGER_H
#define ONBOARDSDK_DJI_MISSIONMANAGER_H

#include "dji_hotpoint.hpp"
#include "dji_waypoint.hpp"
#include <vector>

namespace DJI
{
namespace OSDK
{

enum DJI_MISSION_TYPE
{
  WAYPOINT = 0,
  HOTPOINT = 1,
};

enum MISSION_ACTION
{
  START  = 0,
  STOP   = 1,
  PAUSE  = 2,
  RESUME = 3,
};

class WaypointMission;
class HotpointMission;

/*! @brief MissionManager class for chaining/managing missions
 *
 */
class MissionManager
{
public:
  MissionManager(Vehicle* vehiclePtr = 0);
  ~MissionManager();

  /*! @brief
   *
   *  init missions, could be hotpt or waypt, blocking calls
   *
   *  @param type mission type enum
   *  @param timeout timeout
   *  @param missionData initData for the mission (void ptr)
   */
  ACK::ErrorCode init(DJI_MISSION_TYPE type, int timeout,
                      UserData missionData = 0);
  /*! @brief
   *
   *  init missions, could be hotpt or waypt, non-blocking calls
   *
   *  @param type mission type enum
   *  @param callback user specified callback
   *  @param missionData initData for the mission (void ptr)
   */
  void init(DJI_MISSION_TYPE type, VehicleCallBack callback = 0,
            UserData missionData = 0);
  /*! @brief
   *
   *  a callback function for waypoint non-blocking calls
   *
   *  @param recvFrame RecvContainer populated by the protocolLayer
   *  @param userData void ptr for user usage
   */
  static void missionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                              UserData userData);

private:
  /*! @brief
   *
   *  init waypt mission, blocking calls
   *
   *  @param timeout timeout
   *  @param wayptData initData for the waypt (void ptr)
   */
  ACK::ErrorCode initWayptMission(int timeout = 10, UserData wayptData = 0);
  /*! @brief
   *
   *  init waypt mission, non-blocking calls
   *
   *  @param timeout timeout
   *  @param wayptData initData for the waypt (void ptr)
   */
  void initWayptMission(VehicleCallBack callback = 0, UserData wayptData = 0);
  /*! @brief
   *
   *  init hot pt mission, blocking calls
   *
   *  @param timeout timeout
   *  @param wayptData initData for hotpt (void ptr)
   */
  ACK::ErrorCode initHotptMission(int timeout = 10, UserData wayptData = 0);
  /*! @brief
   *
   *  init hot pt mission, non-blocking calls
   *
   *  @param timeout timeout
   *  @param wayptData initData for hotpt (void ptr)
   */
  void initHotptMission(VehicleCallBack callback = 0, UserData wayptData = 0);

public:
  /*! @brief
   *
   *  get waypt ptr from waypt container
   *
   *  @param index index of waypt container
   */
  WaypointMission* getWaypt(int index);
  /*! @brief
   *
   *  get hotpt ptr from hotpt container
   *
   *  @param index index of hotpt container
   */
  HotpointMission* getHotpt(int index);
  /*! @brief
   *
   *  print the status of the mission manager
   *
   */
  void printInfo();

  Vehicle*         vehicle;
  WaypointMission* wpMission;
  HotpointMission* hpMission;

  std::vector<WaypointMission*> wpMissionVector;
  std::vector<HotpointMission*> hpMissionVector;
};

} // OSDK
} // DJI

#endif // ONBOARDSDK_DJI_MISSIONMANAGER_H
