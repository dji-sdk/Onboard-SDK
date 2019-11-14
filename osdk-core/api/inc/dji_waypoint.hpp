/** @file dji_waypoint.hpp
 *  @version 3.8
 *  @date April 2019
 *
 *  @brief Implementation of GPS Waypoint Missions for DJI OSDK
 *
 *  @Copyright (c) 2016-2019 DJI
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

#ifndef DJI_WAYPOINT_H
#define DJI_WAYPOINT_H

#include "dji_mission_base.hpp"

#ifdef WAYPT2_CORE
#include "dji_waypointv2_interface.hpp"
#endif

namespace DJI
{
namespace OSDK
{

/*! @brief APIs for GPS Waypoint Missions
 *
 *  @details This class inherits from MissionBase and can be used with
 *  MissionManager.
 */
class WaypointMission : public MissionBase
{
public:
  WaypointMission(Vehicle* vehicle = 0);
  ~WaypointMission();

  const double RAD_2_DEGREE = 57.2957795;

  VehicleCallBackHandler wayPointEventCallback;
  VehicleCallBackHandler wayPointCallback;

  /*! @brief
   *
   *  init waypoint mission settings
   *
   *  @param Info action command from DJI_ControllerCMD.h
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void init(WayPointInitSettings* Info = 0, VehicleCallBack callback = 0,
            UserData userData = 0);
  /*! @brief
   *
   *  init waypoint mission settings
   *
   *  @param Info action command from DJI_ControllerCMD.h
   *  @param timeout timeout to wait for ACK
   */
  ACK::ErrorCode init(WayPointInitSettings* Info, int timer);
  /*! @brief
   *
   *  start the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void start(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  start the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode start(int timer);
  /*! @brief
   *
   *  stop the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void stop(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  stop the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode stop(int timer);
  /*! @brief
   *
   *  pause the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void pause(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  pause the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode pause(int timer);
  /*! @brief
   *
   *  resume the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void resume(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  resume the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode resume(int timer);
  /*! @brief
   *
   *  setting waypt init data
   *
   *  @param value user specified WayPointInitData
   */
  void setInfo(const WayPointInitSettings& value);
  /*!
   * @brief Read WayPoint mission settings from the flight controller
   *
   * @return Information about uploaded WayPoint mission, error if
   * mission settings not uploaded.
   *
   */
  ACK::WayPointInit getWaypointSettings(int timer);
  /*!
   * @brief Read WayPoint mission settings from the flight controller
   *
   * Information about uploaded WayPoint mission, error if
   * mission settings not uploaded will be handled in a user defined or
   * default callback.
   */
  void getWaypointSettings(VehicleCallBack callback, UserData userData);
  /*!
   * @brief Read WayPoint index settings from the flight controller
   *
   * @return Information about uploaded WayPoint index, error if
   * index not uploaded.
   */
  ACK::WayPointIndex getIndex(uint8_t index, int timer);
  /*!
   * @brief Read WayPoint index settings from the flight controller
   *
   * Information about uploaded WayPoint index, error if
   * index not uploaded will be handled in a user defined or
   * default callback.
   */
  void getIndex(uint8_t index, VehicleCallBack callback, UserData userData);
  /*! @brief
   *
   *  setting waypt data to the waypt container with specified idx
   *
   *  @param value user specified WayPointData
   *  @param pos the index of the waypt
   */
  void setIndex(WayPointSettings* value, size_t pos);
  /*! @brief
   *
   *  setting waypt init data
   *
   *  @param value user specified WayPointInitData
   */
  bool uploadIndexData(WayPointSettings* data, VehicleCallBack callback = 0,
                       UserData userData = 0);
  /*! @brief
   *
   *  setting waypt init data
   *
   *  @param value user specified WayPointInitData
   *  @param timer timeout to wait for ACK
   */
  ACK::WayPointIndex uploadIndexData(WayPointSettings* data, int timer);
  /*! @brief
   *
   *  getting waypt idle velocity
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void readIdleVelocity(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  getting waypt idle velocity
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode readIdleVelocity(int timeout);
  /*! @brief
   *
   *  setting waypt idle velocity
   *
   *  @param meterPreSecond specified velocity
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void updateIdleVelocity(float32_t       meterPreSecond,
                          VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  setting waypt idle velocity
   *
   *  @param meterPreSecond specified velocity
   *  @param timer timeout to wait for ACK
   */
  ACK::WayPointVelocity updateIdleVelocity(float32_t meterPreSecond,
                                           int       timeout);
  /*! @brief
   *
   *  A callback function for setting idle velocity non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void idleVelocityCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                   UserData userData);
  /*! @brief
   *
   *  A callback function for reading initialization data non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void getWaypointSettingsCallback(Vehicle*      vehicle,
                                          RecvContainer recvFrame,
                                          UserData      userData);
  /*! @brief
   *
   *  A callback function for getting waypoint information for a specified index
   * (non-blocking call)
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void getIndexCallback(Vehicle* vehicle, RecvContainer recvFrame,
                               UserData userData);
  /*! @brief
   *
   *  A callback function for uploading waypt index non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void uploadIndexDataCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                      UserData userData);
  /*! @brief
   *
   *  Set waypoint push data callback
   *
   *  @param callback callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  void setWaypointEventCallback(VehicleCallBack callback, UserData userData);
  /*! @brief
   *
   *  Set waypoint callback
   *
   *  @param callback callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  void setWaypointCallback(VehicleCallBack callback, UserData userData);

#ifdef WAYPT2_CORE
  /*! @brief
   *
   *  update push data from the drone to the internal waypt core library
   *
   *  @param cmd_id
   *  @param cmd seq_num
   *  @param raw data
   *  @param length of the raw data
   */
  void updateV2PushData(uint8_t cmd_id, uint16_t seq_num, const void *data, int data_length);
  /*! @brief
   *
   *  start the waypt mission
   *
   *  @param register a callback function for error code
   */
  void startV2(WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  stop the waypt mission
   *
   *  @param register a callback function for error code
   */
  void stopV2(WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  pause the waypt mission
   *
   *  @param register a callback function for error code
   */
  void pauseV2(WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  resume the waypt mission
   *
   *  @param register a callback function for error code
   */
  void resumeV2(WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  get current speed of the waypt mission
   *
   *  @param register a callback function for error code
   */
  void getCurrentSpeed(std::function<void(float cruise_speed, WaypointV2Interface::CommonErrorCode error_code)> callback);
  /*! @brief
   *
   *  set current speed of the waypt mission
   *
   *  @param register a callback function for error code
   */
  void setCurrentSpeed(float speed, WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  upload a waypt mission with new data strucutre
   *
   *  @param mission settings
   *  @param register a callback function for error code
   */
  void uploadMissionV2(const dji::waypointv2::WaypointMission &waypointMission,
                       WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  upload a waypt mission with old data strucutre
   *
   *  @param mission settings
   *  @param a vector of GPS waypoints
   *  @param register a callback function for error code
   */
  void uploadMissionV2(const WayPointInitSettings &info,
                       const std::vector<WayPointSettings> &waypts,
                       WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  download a waypt mission with new data strucutre
   *
   *  @param mission setting data struct to be written
   *  @param register a callback function for error code
   */
  bool DownloadMissionV2(dji::waypointv2::WaypointMission &outMission,
                         WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  upload a action which work in parallel with waypt mission
   *
   *  @param action settings
   *  @param register a callback function for error code
   */
  void uploadActionV2(const std::vector<dji::waypointv2::WaypointActionConfig> &actions,
                      WaypointV2Interface::CommonErrorCallback errorCallback);
  /*! @brief
   *
   *  get current mission state
   *
   */
  inline dji::waypointv2::AbstractionState getCurrentState() { return waypointV2Interface.getCurrentState(); }
  /*! @brief
   *
   *  get previous mission state
   *
   */
  inline dji::waypointv2::AbstractionState getPrevState() { return waypointV2Interface.getPrevState(); }
  /*! @brief
   *
   *  get current action state
   *
   */
  inline dji::waypointv2::ActionState getCurrentActionState() { return waypointV2Interface.getCurrentActionState(); }
  /*! @brief
   *
   *  get previous action state
   *
   */
  inline dji::waypointv2::ActionState getPrevActionState() { return waypointV2Interface.getPrevActionState(); }
#endif

private:
  WayPointInitSettings info;
  WayPointSettings*    index;

#ifdef WAYPT2_CORE
  WaypointV2Interface waypointV2Interface;
#endif
};

} // namespace OSDK
} // namespace DJI

#endif // DJI_WAYPOINT_H
