/** @file dji_waypoint_v2.hpp
 *  @version 4.0
 *  @date April 2019
 *
 *  @brief Implementation of GPS Waypoint Missions for DJI OSDK
 *
 *  @Copyright (c) 2016-2020 DJI
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

#ifndef DJI_WAYPOINT_MISSION_V2_HPP
#define DJI_WAYPOINT_MISSION_V2_HPP

#include <vector>
#include "dji_mission_base.hpp"
#include "dji_waypoint_v2_action.hpp"

namespace DJI
{
namespace OSDK
{
 /*! The waypoint operator is the only object that controls, runs and monitors
  *  Waypoint v2 Missions.
  *  */
  class WaypointV2MissionOperator
  {
  public:
    const uint16_t MAX_WAYPOINT_NUM_SIGNAL_PUSH = 260;

    /**
     *  All the possible state of ``WaypointV2MissionOperator``.
     */
    enum DJIWaypointV2MissionState{
      /**
       *  The state of the operator is unknown. It is the initial state when the operator
       *  is just created.
       */
        DJIWaypointV2MissionStateUnWaypointActionActuatorknown = -1,

      /**
       *  The connection OSDK device, remote controller and aircraft is
       *  broken.
       */
        DJIWaypointV2MissionStateDisconnected = 0,

      /**
       *  Raed to execute the mission.
       */
        DJIWaypointV2MissionStateReadyToExecute = 1,

      /**
       *  The execution is started successfully.
       */
        DJIWaypointV2MissionStateExecuting = 2,

      /**
       *  Waypoint mission is paused successfully.
       */
        DJIWaypointV2MissionStateInterrupted = 3,

      /**
       *  Waypoint mission is restarted after interrupted.
       */
        DJIWaypointV2MissionStateResumeAfterInterrupted = 4,

      /**
       *  Waypoint mission is exited.
       */
        DJIWaypointV2MissionStateExitMission = 5,

      /**
       *  Waypoint mission is finished.
       */
        DJIWaypointV2MissionStateFinishedMission = 6,
    };

    typedef uint8_t  RetCodeType;

    /*! Common ack of waypoint 2.0*/
    typedef uint32_t WaypointV2CommonAck;

    /*! Common ack of waypoint 2.0*/
    typedef float32_t GlobalCruiseSpeed;

    #pragma pack(1)
    typedef struct UploadMissionRawAck
    {
      uint32_t result;
      uint16_t startIndex;
      uint16_t endIndex;
    }UploadMissionRawAck;

    typedef struct UploadActionsRawAck
    {
      uint32_t result;
      uint16_t errorActionId;
    }UploadActionSRawAck;

    typedef struct DownloadMissionRsp
   {
     uint16_t startIndex;
     uint16_t endIndex;
   }DownloadMissionRsp;

    typedef struct DownloadMissionAck
   {
     uint32_t result;
     uint16_t startIndex;
     uint16_t endIndex;
   }DownloadMissionAck;

    typedef struct GetGlobalCruiseVelAck{
      uint32_t result;
      /*!Unit: cm/s*/
      uint16_t globalCruiseVel;
    }GetGlobalCruiseVelAck;

    typedef struct GetRemainRamAck
    {
      uint16_t totalMemory;
      uint16_t remainMemory;
    }getRemainRamAck;

    typedef struct GetWaypontStartEndIndexAck
    {
      uint32_t result;
      uint16_t startIndex;
      uint16_t endIndex;
    }GetWaypontStartEndIndexAck;

    typedef struct MissionStateCommanData
    {

      uint16_t curWaypointIndex;
      uint8_t  stateDetail:4;
      uint8_t  state:4;
      uint16_t velocity;
      uint8_t  config;
    }MissionStateCommanData;

    typedef struct MissionStatePushAck
    {
      uint8_t commonDataVersion = 1;
      uint16_t commonDataLen;
      MissionStateCommanData data;
    }MissionStatePushAck;

    typedef union Eventdata
    {
      /*ID:0x01*/
      uint8_t interruptReason;

      /*ID:0x02*/
      uint8_t RecoverProcess;

      /*ID:0x03*/
      uint8_t finishReason;

      /*ID:0x10*/
      uint16_t waypointIndex;

      /*ID:0x11*/
       struct MissionExecEvent{
        uint8_t CurrentMissionExecNum;
        uint8_t finishedAllExecNum:1;
        uint8_t reserved:7;
      }MissionExecEvent;

      /*ID:0x12*/
      uint8_t avoidState;

      /*ID:0x20*/
       struct MissionValidityEvent {
        uint8_t misValidityFlag;
        float32_t estimateRunTime;
      }MissionValidityEvent;

      /*ID:0x30*/
       struct ActionExecEvent{
        uint16_t actionId;
        uint8_t preActuatorState;
        uint8_t curActuatorState;
        uint32_t result;
      };
    }Eventdata;

    typedef struct MissionEventPushAck
    {
      uint8_t event ;
      uint16_t FCTimestamp;
      Eventdata data;
    }MissionEventPushAck;
    #pragma pack()

    WaypointV2MissionOperator(Vehicle* vehiclePtr);

    ~WaypointV2MissionOperator();

    ErrorCode::ErrorCodeType init(WayPointV2InitSettings* info, int timeout);

    ErrorCode::ErrorCodeType start(int timeout);
    /*! @brief
     *
     *  stop the waypt mission
     *
     *  @param register a callback function for error code
     */
    ErrorCode::ErrorCodeType stop(int timeout);
    /*! @brief
     *
     *  pause the waypt mission
     *
     *  @param register a callback function for error code
     */
    ErrorCode::ErrorCodeType pause(int timeout);
    /*! @brief
     *
     *  resume the waypt mission
     *
     *  @param register a callback function for error code
     */
    ErrorCode::ErrorCodeType resume(int timeout);

    ErrorCode::ErrorCodeType uploadMission(const std::vector<WaypointV2> &mission,int timeout);

    ErrorCode::ErrorCodeType downloadMission(std::vector<WaypointV2> &vector, int timeout);

    ErrorCode::ErrorCodeType getGlogalCruiseSpeed(GlobalCruiseSpeed &cruiseSpeed,int timeout);

    ErrorCode::ErrorCodeType setGlogalCruiseSpeed(const GlobalCruiseSpeed &cruiseSpeed,int timeout);

    ErrorCode::ErrorCodeType uploadActionV2(std::vector<DJIWaypointV2Action> &actions,int timeout);

    ErrorCode::ErrorCodeType downloadActionV2(int timeout);

    ErrorCode::ErrorCodeType getActionRemainMemory(GetRemainRamAck &remainRamAck, int timeout);

    ErrorCode::ErrorCodeType getWaypointIndexInList(GetWaypontStartEndIndexAck &startEndIndexAck, int timeout);

    void RegisterMissionStateCallback();

    void RegisterMissionEventCallback();


    /*! @brief
     *
     *  get current mission state
     *
     */
    inline DJIWaypointV2MissionState getCurrentState() { return currentState; }
    /*! @brief
     *
     *  get previous mission state
     *
     */
    inline DJIWaypointV2MissionState getPrevState() { return prevState; }

    void setPrevState(DJIWaypointV2MissionState state) {prevState = state; }
    void setCurrentState(DJIWaypointV2MissionState state) {currentState = state; }

  private:
    DJIWaypointV2MissionState currentState;
    DJIWaypointV2MissionState prevState;
    Vehicle *vehiclePtr;
  };

} // namespace OSDK
} // namespace DJI

#endif // DJI_WAYPOINT_MISSION_V2_HPP
