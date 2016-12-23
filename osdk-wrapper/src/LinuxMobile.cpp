/*! @file LinuxMobile.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Framework for executing Onboard SDK commands based on 
 *  mobile commands sent using Data Transparent Transmission.
 *
 *  Calls functions from the new Linux example based on user input
 *  in the companion iOS app
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "LinuxMobile.h"

using namespace DJI;
using namespace DJI::onboardSDK;

/*! Poll at 10Hz, waiting for data from Mobile OSDK App. The Onboard SDK
    receives this data, parses it and sets appropriate member variables
    of the API object. This loop checks to see if any of the members are
    set, and if so then executes that command. 

    The spin exits after ~150 mins.
!*/
void mobileCommandSpin(CoreAPI *api, Flight *flight, WayPoint *waypointObj, Camera *camera, char **trajFiles, int argc)
{
#ifdef USE_PRECISION_MISSIONS
  //! Set up a local frame for Trajectory following
  BroadcastData data = api->getBroadcastData();
  Eigen::Vector3d originLLA(data.pos.latitude, data.pos.longitude, data.pos.altitude);
  CartesianFrame localFrame(originLLA);
  TrajectoryFollower* follower;
  Trajectory* trajectory;

  //! Extract the drone version from the UserConfig params
  std::string droneVer;

  if (UserConfig::targetVersion == versionM100_23 || UserConfig::targetVersion == versionM100_31)
    droneVer = "M100";
  else if (UserConfig::targetVersion == versionA3_31)
    droneVer = "A3";
  else {
    // default case - M100
    droneVer = "M100";
  }

  //! Read the runtime args and populate variables
  std::string pathToSpiral;
  std::string paramTuningFile;

  if (argc > 3) {
    pathToSpiral = std::string(trajFiles[2]);
    paramTuningFile = std::string(trajFiles[3]);
  } else if (argc == 3) {
    pathToSpiral = std::string(trajFiles[2]);
    paramTuningFile = std::string("");
  } else {
    pathToSpiral = std::string("");
    paramTuningFile = std::string("");
  }

  //! Set up the follower using the tuning parameters supplied
  if (!pathToSpiral.empty()) {
    TrajectoryInfrastructure::startStateBroadcast(api);
    follower = TrajectoryInfrastructure::setupFollower(api,
                                                       flight,
                                                       &localFrame,
                                                       camera,
                                                       droneVer,
                                                       paramTuningFile);
  } else {
    follower = NULL;
    std::cout << "You need to supply a trajectory as a program argument.\n";
  }

  //! Set up the trajectory using the trajectory parameters supplied
  trajectory = TrajectoryInfrastructure::setupTrajectory(pathToSpiral);
#endif //! USE_PRECISION_MISSIONS

  int t = 0;
  ackReturnToMobile returnACKMobile;
  while (t < 100000)
  {
    if (api->getObtainControlMobileCMD())
    {
      api->setObtainControlMobileCMD(false);
      ackReturnData takeControlStatus = takeControl(api);
      returnACKMobile.cmdID = 2;
      returnACKMobile.ack = takeControlStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getReleaseControlMobileCMD())
    {
      api->setReleaseControlMobileCMD(false);
      ackReturnData releaseControlStatus = releaseControl(api);
      returnACKMobile.cmdID = 3;
      returnACKMobile.ack = releaseControlStatus.ack;      
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getActivateMobileCMD())
    {
      api->setActivateMobileCMD(false);
      ackReturnData activateStatus = activate(api);
      returnACKMobile.cmdID = 4;
      returnACKMobile.ack = activateStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getTakeOffMobileCMD())
    {
      api->setTakeOffMobileCMD(false);
      ackReturnData takeoffStatus = monitoredTakeoff(api, flight);
      returnACKMobile.cmdID = 7;
      returnACKMobile.ack = takeoffStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getLandingMobileCMD())
    {
      api->setLandingMobileCMD(false);
      ackReturnData landingStatus = landing(api, flight);
      returnACKMobile.cmdID = 8;
      returnACKMobile.ack = landingStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getArmMobileCMD())
    {
      api->setArmMobileCMD(false);
      ackReturnData armStatus = arm(flight);
      returnACKMobile.cmdID = 5;
      returnACKMobile.ack = armStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getDisArmMobileCMD())
    {
      api->setDisArmMobileCMD(false);
      ackReturnData disArmStatus = disArm(flight);
      returnACKMobile.cmdID = 6;
      returnACKMobile.ack = disArmStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getGoHomeMobileCMD())
    {
      api->setGoHomeMobileCMD(false);
      ackReturnData goHomeStatus = goHome(flight);
      returnACKMobile.cmdID = 9;
      returnACKMobile.ack = goHomeStatus.ack;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getDrawSqrMobileCMD())
    {
      api->setDrawSqrMobileCMD(false);
      int drawSqrPosCtrlStatus = drawSqrPosCtrlSample(api, flight);
      returnACKMobile.cmdID = 62;
      returnACKMobile.ack = drawSqrPosCtrlStatus;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getWayPointTestMobileCMD())
    {
      api->setWayPointTestMobileCMD(false);
      wayPointMissionExample(api, waypointObj,1);
      returnACKMobile.cmdID = 65;
      returnACKMobile.ack = 1; //Always true since example does not return an ACK
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getGimbalCtrlMobileCMD())
    {
      api->setGimbalCtrlMobileCMD(false);
      gimbalAngleControlSample(camera);
      returnACKMobile.cmdID = 64;
      returnACKMobile.ack = 1; //Always true since example does not return an ACK
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }

    if (api->getStartLASMapLoggingCMD())
    {
      api->setStartLASMapLoggingCMD(false);
      system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
      system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:= . &");
      returnACKMobile.cmdID = 20;
      returnACKMobile.ack = 1;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }

    if (api->getStopLASMapLoggingCMD())
    {
      api->setStopLASMapLoggingCMD(false);
      system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry   &");
      returnACKMobile.cmdID = 21;
      returnACKMobile.ack = 1;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }

#ifdef USE_PRECISION_MISSIONS
    if (api->getPrecisionMissionsCMD()) {

      api->setPrecisionMissionsCMD(false);

      //! Return ACK as an indication that we got the signal - NOT as a successful completion indication
      returnACKMobile.cmdID = 24;
      returnACKMobile.ack = 1;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

      //! Check if the aircraft has taken off. If not, make it do so.
      data = api->getBroadcastData();
      if (data.status < 2) {
        std::cout << "Aircraft has not taken off. Taking off now...\n";
        ackReturnData takeoffStatus = monitoredTakeoff(api, flight, 1);
        if (takeoffStatus.status == -1) {
          //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
          returnACKMobile.cmdID = 24;
          returnACKMobile.ack = 3;
          api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));
          continue;
        }
      }

      TrajectoryInfrastructure::startStateBroadcast(api);

      //! Run precision missions without LiDAR features
      uint16_t trajectoryAck = TrajectoryInfrastructure::executeFromParams(api, &localFrame, originLLA, trajectory, follower);

      //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
      returnACKMobile.cmdID = 24;
      returnACKMobile.ack = trajectoryAck;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

    }
    if (api->getPrecisionMissionsCollisionAvoidanceCMD()) {

      api->setPrecisionMissionsCollisionAvoidanceCMD(false);

      //! Return ACK as an indication that we got the signal - NOT as a successful completion indication
      returnACKMobile.cmdID = 25;
      returnACKMobile.ack = 1;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

      //! Check if the aircraft has taken off. If not, make it do so.
      data = api->getBroadcastData();
      if (data.status < 2) {
        std::cout << "Aircraft has not taken off. Taking off now...\n";
        ackReturnData takeoffStatus = monitoredTakeoff(api, flight, 1);
        if (takeoffStatus.status == -1) {
          //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
          returnACKMobile.cmdID = 25;
          returnACKMobile.ack = 3;
          api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));
          continue;
        }
      }

      TrajectoryInfrastructure::startStateBroadcast(api);

      //! Run precision missions with Collision Avoidance
      trajectory->enableCollisionAvoidance(true);
      uint16_t trajectoryAck = TrajectoryInfrastructure::executeFromParams(api, &localFrame, originLLA, trajectory, follower);

      //! Un-set the flag in case you want to do another mission
      trajectory->enableCollisionAvoidance(false);

      //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
      returnACKMobile.cmdID = 25;
      returnACKMobile.ack = trajectoryAck;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

    }
    if (api->getPrecisionMissionsLidarMappingCMD()) {

      api->setPrecisionMissionsLidarMappingCMD(false);

      //! Return ACK as an indication that we got the signal - NOT as a successful completion indication
      returnACKMobile.cmdID = 26;
      returnACKMobile.ack = 1;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

      //! Check if the aircraft has taken off. If not, make it do so.
      data = api->getBroadcastData();
      if (data.status < 2) {
        std::cout << "Aircraft has not taken off. Taking off now...\n";
        ackReturnData takeoffStatus = monitoredTakeoff(api, flight, 1);
        if (takeoffStatus.status == -1) {
          //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
          returnACKMobile.cmdID = 26;
          returnACKMobile.ack = 3;
          api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));
          continue;
        }
      }

      TrajectoryInfrastructure::startStateBroadcast(api);

      //! Run precision missions with LiDAR Mapping
      trajectory->enableLidarMapping(true);
      uint16_t trajectoryAck = TrajectoryInfrastructure::executeFromParams(api, &localFrame, originLLA, trajectory, follower);

      //! Un-set the flag in case you want to do another mission with different settings
      trajectory->enableLidarMapping(false);

      //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
      returnACKMobile.cmdID = 26;
      returnACKMobile.ack = trajectoryAck;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

    }
    if (api->getPrecisionMissionsCollisionAvoidanceLidarMappingCMD()) {

      api->setPrecisionMissionsCollisionAvoidanceLidarMappingCMD(false);

      //! Return ACK as an indication that we got the signal - NOT as a successful completion indication
      returnACKMobile.cmdID = 27;
      returnACKMobile.ack = 1;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

      //! Check if the aircraft has taken off. If not, make it do so.
      data = api->getBroadcastData();
      if (data.status < 2) {
        std::cout << "Aircraft has not taken off. Taking off now...\n";
        ackReturnData takeoffStatus = monitoredTakeoff(api, flight, 1);
        if (takeoffStatus.status == -1) {
          //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
          returnACKMobile.cmdID = 27;
          returnACKMobile.ack = 3;
          api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));
          continue;
        }
      }

      TrajectoryInfrastructure::startStateBroadcast(api);

      //! Run precision missions with Collision Avoidance AND LiDAR Mapping
      trajectory->enableCollisionAvoidance(true);
      trajectory->enableLidarMapping(true);
      uint16_t trajectoryAck = TrajectoryInfrastructure::executeFromParams(api, &localFrame, originLLA, trajectory, follower);

      //! Un-set the flag in case you want to do another mission with different settings
      trajectory->enableCollisionAvoidance(false);
      trajectory->enableLidarMapping(false);

      //! Return ACK as return value from the trajectory execution : 2 = SUCCESS and 3 = FAILURE
      returnACKMobile.cmdID = 27;
      returnACKMobile.ack = trajectoryAck;
      api->sendToMobile((uint8_t *) (&returnACKMobile), sizeof(returnACKMobile));

    }
#endif
    usleep(100000);
    t++;
  }
}

/*! Non-Blocking Mobile command spin
 * Poll at 10Hz, waiting for data from Mobile OSDK App. The Onboard SDK
    receives this data, parses it and sets appropriate member variables
    of the API object. This loop checks to see if any of the members are
    set, and if so then executes that command.

    The spin exits after ~150 mins.
!*/
void mobileCommandSpinNonBlocking(CoreAPI* api, Flight* flight, WayPoint* waypointObj)
{
  int t = 0;
  ackReturnToMobile returnACKMobile;
  while (t < 100000)
  {
    if (api->getObtainControlMobileCMD())
    {
      api->setObtainControlMobileCMD(false);
      takeControlNonBlocking(api);
      usleep(100000);
      returnACKMobile.cmdID = 2;
      returnACKMobile.ack = api->missionACKUnion.simpleACK;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile)); }
    if (api->getReleaseControlMobileCMD())
    {
      api->setReleaseControlMobileCMD(false);
      releaseControlNonBlocking(api);
      usleep(100000);
      returnACKMobile.cmdID = 3;
      returnACKMobile.ack = api->missionACKUnion.simpleACK;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getActivateMobileCMD())
    {
      api->setActivateMobileCMD(false);
      activateNonBlocking(api);
      usleep(100000);
      returnACKMobile.cmdID = 4;
      returnACKMobile.ack = api->missionACKUnion.simpleACK;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getTakeOffMobileCMD())
    {
      api->setTakeOffMobileCMD(false);
      takeoffNonBlocking(flight);
      usleep(100000);
      returnACKMobile.cmdID = 7;
      returnACKMobile.ack = api->missionACKUnion.simpleACK;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getLandingMobileCMD())
    {
      api->setLandingMobileCMD(false);
      landingNonBlocking(flight);
      usleep(100000);
      returnACKMobile.cmdID = 8;
      returnACKMobile.ack = api->missionACKUnion.simpleACK;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getArmMobileCMD())
    {
      api->setArmMobileCMD(false);
      armNonBlocking(flight);
      usleep(100000);
      returnACKMobile.cmdID = 5;
      returnACKMobile.ack = api->missionACKUnion.simpleACK;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    if (api->getDisArmMobileCMD())
    {
      api->setDisArmMobileCMD(false);
      disArmNonBlocking(flight);
      usleep(100000);
      returnACKMobile.cmdID = 6;
      returnACKMobile.ack = api->missionACKUnion.simpleACK;
      api->sendToMobile((uint8_t*)(&returnACKMobile),sizeof(returnACKMobile));
    }
    usleep(100000);
    t++;
  }
}
