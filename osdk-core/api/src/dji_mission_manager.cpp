/** @file dji_mission_manager.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Mission Manager API for DJI OSDK library
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

#include "dji_mission_manager.hpp"

using namespace DJI;
using namespace DJI::OSDK;

MissionManager::MissionManager(Vehicle* vehiclePtr)
  : vehicle(vehiclePtr)
  , wpMission(NULL)
  , hpMission(NULL)
{
}

MissionManager::~MissionManager()
{
  for (int i = 0; i < wpMissionVector.size(); ++i) {
    delete wpMissionVector[i];
  }
  for (int i = 0; i < hpMissionVector.size(); ++i) {
    delete hpMissionVector[i];
  }
}

ACK::ErrorCode
MissionManager::init(DJI_MISSION_TYPE type, int timeout, UserData missionData)
{
  if (type == WAYPOINT)
  {
    return this->initWayptMission(timeout, missionData);
  }
  else if (type == HOTPOINT)
  {
    return this->initHotptMission(timeout, missionData);
  }
  else
  {
    DERROR("Cannot recognize the mission type provided\n");
    // @todo return a false ack
    ACK::ErrorCode ack;
    ack.info.cmd_set = OpenProtocolCMD::CMDSet::mission;
    ack.data = ErrorCode::MissionACK::Common::INVALID_COMMAND;
    return ack;
  }
}

void
MissionManager::init(DJI_MISSION_TYPE type, VehicleCallBack callback,
                     UserData missionData)
{
  if (type == WAYPOINT)
  {
    this->initWayptMission(callback, missionData);
  }
  else if (type == HOTPOINT)
  {
    // @note hotpoint init() doesn't have blocking calls, timeout use 10
    this->initHotptMission(10, missionData);
  }
  else
  {
    DERROR("Cannot recognize the mission type provided\n");
  }
}

ACK::ErrorCode
MissionManager::initWayptMission(int timeout, UserData wayptData)
{
  WaypointMission* newMission = new WaypointMission(this->vehicle);
  wpMissionVector.push_back(newMission);
  wpMission = wpMissionVector.back();

#ifdef WAYPT2_CORE
  // Do nothing
#else
  // @todo timeout needs to be defined somewhere globally
  return wpMission->init((WayPointInitSettings*)wayptData, timeout);
#endif
}

void
MissionManager::initWayptMission(VehicleCallBack callback, UserData wayptData)
{
  WaypointMission* newMission = new WaypointMission(this->vehicle);
  wpMissionVector.push_back(newMission);
  wpMission = wpMissionVector.back();

#ifdef WAYPT2_CORE
  // Do nothing
#else
  wpMission->init((WayPointInitSettings*)wayptData, callback, wayptData);
#endif
}

ACK::ErrorCode
MissionManager::initHotptMission(int timeout, UserData hotptData)
{
  HotpointMission* newMission = new HotpointMission(this->vehicle);
  hpMissionVector.push_back(newMission);
  hpMission = hpMissionVector.back();

  if (hotptData)
  {
    hpMission->setData((HotPointSettings*)hotptData);
  }
  else
  {
    hpMission->initData();
  }

  // @todo this initData() does not return ack
  ACK::ErrorCode ack;
  ack.info.cmd_set = OpenProtocolCMD::CMDSet::mission;
  ack.data = ErrorCode::MissionACK::Common::SUCCESS;

  return ack;
}

void
MissionManager::initHotptMission(VehicleCallBack callback, UserData wayptData)
{
  /*! @note hotpoint init() doesn't have blocking calls
   *        put it here for future uses
   */
}

void
MissionManager::missionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                                UserData userData)
{
  char           func[50];
  ACK::ErrorCode ack;

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <=
      sizeof(ACK::ErrorCode))
  {
    ack.info = recvFrame.recvInfo;
    ack.data = recvFrame.recvData.missionACK;

    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }
  }
  else
  {
    DERROR("ACK is exception,sequence %d\n", recvFrame.recvInfo.seqNumber);
  }
}

WaypointMission*
MissionManager::getWaypt(int index)
{
  if (index >= wpMissionVector.size())
  {
    DERROR("The waypt index does not exist in Mission Manager\n");
    return NULL;
  }

  return wpMissionVector[index];
}

HotpointMission*
MissionManager::getHotpt(int index)
{
  if (index >= hpMissionVector.size())
  {
    DERROR("The hotpt index does not exist in Mission Manager\n");
    return NULL;
  }

  return hpMissionVector[index];
}

void
MissionManager::printInfo()
{
  DSTATUS("Mission Manager status: \n");
  DSTATUS("There are %d waypt missions and %d hotpoint missions\n",
          wpMissionVector.size(), hpMissionVector.size());
}
