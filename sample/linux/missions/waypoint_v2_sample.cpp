/*! @file waypoint_v2_sample.cpp
 *  @version 3.8
 *  @date Mar 07 2019
 *
 *  @brief
 *  main for Waypoint Missions V2 API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions through
 *  the Mission Manager API.
 *
 *  @Copyright (c) 2019 DJI
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

#include "waypoint_v2_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

std::mutex m;
std::condition_variable condVar;
std::string funcName;

WaypointV2Interface::CommonErrorCallback defaultErrorCB =
  []
    (WaypointV2Interface::CommonErrorCode errorCode)
  {
    DSTATUS("%s return error code: %d\n", funcName.c_str(), errorCode);
    condVar.notify_one();
  };

bool
setUpSubscription(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;

  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int       freq            = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
  int       numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  // Start listening to the telemetry data
  subscribeStatus =
    vehicle->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
    if (ACK::getError(ack))
    {
      DSTATUS("Error unsubscribing; please restart the drone/FC to get "
        "back to a clean state.\n");
    }
    return false;
  }
  return true;
}

bool
teardownSubscription(DJI::OSDK::Vehicle* vehicle, const int pkgIndex,
                     int responseTimeout)
{
  ACK::ErrorCode ack =
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
  if (ACK::getError(ack))
  {
    DSTATUS("Error unsubscribing; please restart the drone/FC to get back "
      "to a clean state.\n");
    return false;
  }
  return true;
}

bool
createAndUploadWaypointMission(Vehicle *vehicle, uint8_t numWaypoints, int responseTimeout)
{
  if(!vehicle->isM210V2())
  {
    DSTATUS("This sample only supports M210 V2!\n");
    return false;
  }

  // Waypoint Mission : Initialization
  WayPointInitSettings fdata;
  setWaypointInitDefaults(&fdata);

  fdata.indexNumber = numWaypoints + 1; // We add 1 to get the aircarft back to the start.

  float64_t increment = 0.000001;
  float32_t start_alt = 10;

  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
  fdata.latitude  = subscribeGPosition.latitude;
  fdata.longitude = subscribeGPosition.longitude;
  fdata.altitude  = subscribeGPosition.altitude;

  vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);

  vehicle->missionManager->printInfo();
  DSTATUS("Initializing Waypoint Mission..\n");

  // Waypoint Mission: Create Waypoints
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(vehicle, numWaypoints, increment, start_alt);
  DSTATUS("Creating Waypoints..\n");

  int waitTime = 0;
  int interval = 1;
  int timeOut = 10;
  dji::waypointv2::AbstractionState currentState = vehicle->missionManager->wpMission->getCurrentState();
  while(currentState != dji::waypointv2::ReadyToUpload && waitTime < timeOut)
  {
    sleep(interval);
    currentState = vehicle->missionManager->wpMission->getCurrentState();
    waitTime += interval;
  }
  DSTATUS("Waited %d secs for WP2 state change to ReadyToUpload..\n", waitTime);


  DSTATUS("Uploading Waypoints..\n");
  funcName = "uploadMissionV2";
  // Waypoint Mission: Upload Waypoints
  vehicle->missionManager->wpMission->uploadMissionV2(fdata, generatedWaypts, defaultErrorCB);
  // DefalutErrorCB is running on a separate thread
  // We wait here for up to 1 sec to
  // 1. get the response of the callback
  // 2. wait for the end of the execution
  {
    std::unique_lock<std::mutex> lk(m);
    condVar.wait_for(lk, std::chrono::seconds(1));
  }

  return true;
}

bool
createActions(DJI::OSDK::Vehicle *vehicle,
              dji::waypointv2::WaypointActionActuatorType actuatorType,
              dji::waypointv2::WaypointActionTriggerType triggerType,
              std::vector<dji::waypointv2::WaypointActionConfig> &actions)
{
  if(!vehicle->isM210V2())
  {
    DSTATUS("This sample only supports M210 V2!\n");
    return false;
  }

  using namespace dji::waypointv2;
  static int actionCounter = 0;
  static int actuatorIdx = 0;

  // protocol buffer is responsible to free these memories
  // ref: https://developers.google.com/protocol-buffers/docs/reference/cpp-generated
  // search for "set_allocated_foo"
  WaypointActionConfig* action = new WaypointActionConfig();
  WaypointActionActuator* actuator = new WaypointActionActuator();
  WaypointActionTrigger* trigger = new WaypointActionTrigger();

  auto errorLambda = [&](){
    if(action) delete action;
    if(actuator) delete actuator;
    if(trigger) delete trigger;
    return false;
  };

  switch (actuatorType)
  {
    case dji::waypointv2::Camera:
    {
      WaypointActionOperatorCamera* cameraParam = new WaypointActionOperatorCamera();
      ActuatorCameraShootPhotoParam* shootPhotoParam = new ActuatorCameraShootPhotoParam();
      shootPhotoParam->set_retry_times(2);
      cameraParam->set_action_type(ShootSinglePhoto);
      cameraParam->set_allocated_shootphoto_param(shootPhotoParam);
      actuator->set_actuator_index(actuatorIdx);
      ++actuatorIdx;
      actuator->set_actuator_type(dji::waypointv2::Camera);
      actuator->set_allocated_camera_operator(cameraParam);
    }
      break;
    case dji::waypointv2::Gimbal:
    {
      WaypointActionOperatorGimbal* gimbalParam = new WaypointActionOperatorGimbal();
      ActuatorGimbalRotateParam* rotateGimbalParam = new ActuatorGimbalRotateParam();
      rotateGimbalParam->set_is_absolute(false);
      rotateGimbalParam->set_gimbal_roll(-10);
      rotateGimbalParam->set_gimbal_yaw(-90);
      rotateGimbalParam->set_gimbal_pitch(-30);
      rotateGimbalParam->set_duration_time(3);
      rotateGimbalParam->set_is_aircraft_heading_coordinate_system(true);
      gimbalParam->set_action_type(RotateGimbal);
      gimbalParam->set_allocated_rotate_param(rotateGimbalParam);
      actuator->set_actuator_index(actuatorIdx);
      ++actuatorIdx;
      actuator->set_actuator_type(dji::waypointv2::Gimbal);
      actuator->set_allocated_gimbal_operator(gimbalParam);
    }
      break;
    case dji::waypointv2::AircraftControl:
    {
      WaypointActionOperatorAircraftControl* aircraftControlParam = new WaypointActionOperatorAircraftControl();
      ActuatorAircraftControlRotateYawParam* rotateYawParam = new ActuatorAircraftControlRotateYawParam();
      rotateYawParam->set_is_relative(true);
      rotateYawParam->set_yaw_angle(180);
      rotateYawParam->set_is_clockwise(true);
      aircraftControlParam->set_action_type(RotateYaw);
      aircraftControlParam->set_allocated_rotate_yaw_param(rotateYawParam);
      actuator->set_actuator_index(actuatorIdx);
      ++actuatorIdx;
      actuator->set_actuator_type(dji::waypointv2::AircraftControl);
      actuator->set_allocated_aircraft_control_operator(aircraftControlParam);
    }
      break;
    default:
    {
      DSTATUS("User input the wrong actuator type\n");
      return errorLambda();
    }
      break;
  }

  switch (triggerType)
  {
    case ReachPoints:
    {
      /// This trigger is not supported yet
      DERROR("This trigger is not supported yet\n");
      return errorLambda();
//      WaypointActionReachPointsTriggerParam *reachPointsTriggerParam = new WaypointActionReachPointsTriggerParam();
//      reachPointsTriggerParam->set_start_index(2);
//      reachPointsTriggerParam->set_end_index(5);
//      reachPointsTriggerParam->set_interval_count(1);
//      reachPointsTriggerParam->set_auto_exit_count(4);
//      trigger->set_trigger_type(ReachPoints);
//      trigger->set_allocated_reach_point_param(reachPointsTriggerParam);
    }
      break;
    case Associate:
    {
      if(actionCounter == 0)
      {
        DERROR("This trigger only works with previous action, please upload other action first\n");
        return errorLambda();
      }
      WaypointActionAssociateTriggerParam* associateTriggerParam = new WaypointActionAssociateTriggerParam();
      associateTriggerParam->set_associate_type(ExecuteAfterAssoiate); // trigger after action#0 is executed
      associateTriggerParam->set_associate_action_id(0); // associate with the first action id: 0
      associateTriggerParam->set_waiting_time(0.1);
      trigger->set_trigger_type(Associate);
      trigger->set_allocated_associate_param(associateTriggerParam);
    }
      break;
    case Trajectory:
    {
      /// This trigger is not supported yet
      DERROR("This trigger is not supported yet\n");
      return errorLambda();
//      WaypointActionTrajectoryTriggerParam* trajectoryTriggerParam = new WaypointActionTrajectoryTriggerParam();
//      trajectoryTriggerParam->set_start_index(1);
//      // So far only support one idx apart, i.e. endIdx - startIdx == 1
//      trajectoryTriggerParam->set_end_index(2);
//      trigger->set_trigger_type(Trajectory);
//      trigger->set_allocated_trajectory_param(trajectoryTriggerParam);
    }
      break;
    case SimpleInterval:
    {
      WaypointActionSimpleIntervalParam* simpleIntervalParam = new WaypointActionSimpleIntervalParam();
      simpleIntervalParam->set_start_index(2);
      simpleIntervalParam->set_interval_type(IntervalTypeTime);
      simpleIntervalParam->set_interval_value(0.5);
      trigger->set_trigger_type(SimpleInterval);
      trigger->set_allocated_simple_interval_param(simpleIntervalParam);
    }
      break;
    case SimpleReachPoint:
    {
      WaypointActionSimpleReachPointParam* simpleReachPointParam = new WaypointActionSimpleReachPointParam();
      simpleReachPointParam->set_start_index(2);
      simpleReachPointParam->set_auto_exit_count(1);
      trigger->set_trigger_type(SimpleReachPoint);
      trigger->set_allocated_simple_reach_point_param(simpleReachPointParam);
    }
      break;
    default:
    {
      DERROR("User input the wrong trigger type\n");
      return errorLambda();
    }
      break;
  }


  action->set_action_id(actionCounter);
  ++actionCounter;
  action->set_allocated_actuator(actuator);
  action->set_allocated_trigger(trigger);

  actions.push_back(*action);
  delete action;
}

bool uploadActions(DJI::OSDK::Vehicle *vehicle,
                   std::vector<dji::waypointv2::WaypointActionConfig> &actions)
{
  if(!vehicle->isM210V2())
  {
    DSTATUS("This sample only supports M210 V2!\n");
    return false;
  }

  DSTATUS("Uploading actions..\n");
  funcName = "uploadActionV2";
  vehicle->missionManager->wpMission->uploadActionV2(actions, defaultErrorCB);
  {
    std::unique_lock<std::mutex> lk(m);
    condVar.wait_for(lk, std::chrono::seconds(1));
  }
}

bool
startWaypointMission(DJI::OSDK::Vehicle *vehicle)
{
  if(!vehicle->isM210V2())
  {
    DSTATUS("This sample only supports M210 V2!\n");
    return false;
  }

  DSTATUS("Starting Waypoints mission..\n");
  funcName = "startV2";
  vehicle->missionManager->wpMission->startV2(defaultErrorCB);
  {
    std::unique_lock<std::mutex> lk(m);
    condVar.wait_for(lk, std::chrono::seconds(1));
  }

  //wait until in the middle of execution
  sleep(8);

  DSTATUS("Compare init data ....\n");
  dji::waypointv2::WaypointMission mission;
  funcName = "DownloadMissionV2";
  bool successDownloaded = vehicle->missionManager->wpMission->DownloadMissionV2(mission, defaultErrorCB);
    {
      std::unique_lock<std::mutex> lk(m);
      condVar.wait_for(lk, std::chrono::seconds(1));
    }

  if(successDownloaded)
  {
    DSTATUS("Init data total waypoints: %d, max speed: %f\n",
            (int)mission.total_len(),
            mission.global_max_velocity());
  }

  //wait until in the middle of execution
  sleep(16);

  float currentSpeed = -1;
  funcName = "getCurrentSpeed";
  vehicle->missionManager->wpMission->getCurrentSpeed(
    [&currentSpeed](float cruiseSpeed,
                    WaypointV2Interface::CommonErrorCode error_code){
      if(error_code == 0)
      {
        currentSpeed = cruiseSpeed;
      }
      condVar.notify_one();
    });
  {
    std::unique_lock<std::mutex> lk(m);
    condVar.wait_for(lk, std::chrono::seconds(1));
  }
  DSTATUS("Current speed: %f\n", currentSpeed);


  funcName = "setCurrentSpeed";
  vehicle->missionManager->wpMission->setCurrentSpeed(1.0, defaultErrorCB);
  {
    std::unique_lock<std::mutex> lk(m);
    condVar.wait_for(lk, std::chrono::seconds(1));
  }

  //wait until in the middle of execution
  sleep(1);

  currentSpeed = -1.0;
  vehicle->missionManager->wpMission->getCurrentSpeed(
    [&currentSpeed](float cruiseSpeed,
                    WaypointV2Interface::CommonErrorCode error_code){
      if(error_code == 0)
      {
        currentSpeed = cruiseSpeed;
      }
      condVar.notify_one();
    });
  {
    std::unique_lock<std::mutex> lk(m);
    condVar.wait_for(lk, std::chrono::seconds(1));
  }
  DSTATUS("Current speed: %f\n", currentSpeed);
}

void
setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

void
setWaypointInitDefaults(WayPointInitSettings* fdata)
{
  fdata->maxVelocity    = 10;
  fdata->idleVelocity   = 8;
  fdata->finishAction   = 0;
  fdata->executiveTimes = 1;
  fdata->yawMode        = 0;
  fdata->traceMode      = 0;
  fdata->RCLostAction   = 1;
  fdata->gimbalPitch    = 0;
  fdata->latitude       = 0;
  fdata->longitude      = 0;
  fdata->altitude       = 0;
}

std::vector<DJI::OSDK::WayPointSettings>
createWaypoints(DJI::OSDK::Vehicle* vehicle, int numWaypoints,
                float64_t distanceIncrement, float32_t start_alt)
{
  // Create Start Waypoint
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition broadcastGPosition;

  if (!vehicle->isM100() && !vehicle->isLegacyM600())
  {
    subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = start_alt;
    DSTATUS("Waypoint created at (LLA): %lf \t%lf \t%lf\n",
           subscribeGPosition.latitude, subscribeGPosition.longitude,
           start_alt);
  }
  else
  {
    broadcastGPosition = vehicle->broadcast->getGlobalPosition();
    start_wp.latitude  = broadcastGPosition.latitude;
    start_wp.longitude = broadcastGPosition.longitude;
    start_wp.altitude  = start_alt;
    DSTATUS("Waypoint created at (LLA): %lf \t%lf \t%lf\n",
           broadcastGPosition.latitude, broadcastGPosition.longitude,
           start_alt);
  }

  std::vector<DJI::OSDK::WayPointSettings> wpVector =
    generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
  return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings>
generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment,
                         int num_wp)
{
  // Let's create a vector to store our waypoints in.
  std::vector<DJI::OSDK::WayPointSettings> wp_list;

  // Some calculation for the polygon
  float64_t extAngle = 2 * M_PI / num_wp;

  // First waypoint
  start_data->index = 0;
  wp_list.push_back(*start_data);

  // Iterative algorithm
  for (int i = 1; i < num_wp; i++)
  {
    WayPointSettings  wp;
    WayPointSettings* prevWp = &wp_list[i - 1];
    setWaypointDefaults(&wp);
    wp.index     = i;
    wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
    wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
    wp.altitude  = (prevWp->altitude + 1);
    wp_list.push_back(wp);
  }

  // Come back home
  start_data->index = num_wp;
  wp_list.push_back(*start_data);

  return wp_list;
}
