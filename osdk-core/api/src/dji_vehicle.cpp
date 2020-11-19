/** @file dji_vehicle.cpp
 *  @version 4.0.0
 *  @date April 2017
 *
 *  @brief
 *  Vehicle API for DJI onboardSDK library
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

#include "dji_vehicle.hpp"
#include "osdk_device_id.h"
#include "dji_linker.hpp"
#include "osdk_firewall.hpp"
#include "dji_internal_command.hpp"
#include <new>

using namespace DJI;
using namespace DJI::OSDK;

#ifndef DJIOSDK_MAJOR_VERSION
#define DJIOSDK_MAJOR_VERSION 0
#endif

#ifndef DJIOSDK_MINOR_VERSION
#define DJIOSDK_MINOR_VERSION 0
#endif

#ifndef DJIOSDK_PATCH_VERSION
#define DJIOSDK_PATCH_VERSION 0
#endif

#ifndef DJIOSDK_IS_DEBUG
#define DJIOSDK_IS_DEBUG 1
#endif

#ifndef DJIOSDK_HARDWARE_TYPE
#define DJIOSDK_HARDWARE_TYPE 0
#endif

#ifndef DJIOSDK_OPERATOR_TYPE
#define DJIOSDK_OPERATOR_TYPE 0
#endif

const  uint8_t       Vehicle::kOSDKSendId                    = 129;
const  uint32_t      Vehicle::kHeartBeatPackSendTimeInterval = 1000;
static uint8_t       osdkConnectFCFlag                       = 0;
       HeartBeatPack Vehicle::heartBeatPack                  = { kOSDKSendId,PROTOCOL_SDK,0, { 0 }};
       uint8_t       Vehicle::fcLostConnectCount             = 0;

Vehicle::Vehicle(Linker* linker)
  : linker(linker)
  , legacyLinker(NULL)
  , camera(NULL)
  , gimbal(NULL)
  , control(NULL)
  , broadcast(NULL)
  , subscribe(NULL)
  , mfio(NULL)
  , mobileDevice(NULL)
  , hardSync(NULL)
  , payloadDevice(NULL)
  , cameraManager(NULL)
  , gimbalManager(NULL)
  , psdkManager(NULL)
  , flightController(NULL)
  , missionManager(NULL)
#if defined(__linux__)
  , waypointV2Mission(NULL)
#endif
  , firewall(NULL)
  , djiBattery(NULL)
#if defined(__linux__)
	, djiHms(NULL)
  , mopServer(NULL)
#endif
#ifdef ADVANCED_SENSING
  , advancedSensing(NULL)
  , advSensingErrorPrintOnce(false)
#endif
{
  ackErrorCode.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  sendHeartbeatToFCHandle = NULL;
}

bool
Vehicle::init()
{
  if (!initOSDKHeartBeatThread())
  {
    DERROR("Failed to initialize OSDKHeartBeatThread!\n");
    return false;
  }

  if (!initLegacyLinker())
  {
    DERROR("Failed to initialize LegacyLinker!\n");
    return false;
  }

  /*
   * Initialize subscriber if supported
   */
  if (!initSubscriber())
  {
    DERROR("Failed to initialize subscriber!\n");
    return false;
  }

  /*
   * Initialize broadcast if supported
   */
  if (!initBroadcast())
  {
    DERROR("Failed to initialize Broadcast!\n");
    return false;
  }

  /*
   * @note Initialize Movement Control,
   * @note it will be replaced by FlightActions and FlightController in the future.
   */
  if (!initControl())
  {
    DERROR("Failed to initialize Control!\n");
    return false;
  }

  /*
   * @note Initialize external components
   * like Camera and MFIO
   */
  if (!initCamera())
  {
    DERROR("Failed to initialize Camera!\n");
    return false;
  }

  /*
   * Initialize MFIO if supported
   */
  if (!initMFIO())
  {
    DERROR("Failed to initialize MFIO!\n");
    return false;
  }

  if(!initGimbal())
  {
    DERROR("Failed to initialize Gimbal!\n");
    return false;
  }

  /*
   * Initialize Mobile Device Abstraction
   */
  if (!initMobileDevice())
  {
    DERROR("Failed to initialize Mobile Device!\n");
  }

  if (!initPayloadDevice())
  {
    DERROR("Failed to initialize Payload Device!\n");
  }

  if (!initCameraManager())
  {
    DERROR("Failed to initialize PayloadManager!\n");
  }

  if (!initPSDKManager())
  {
    DERROR("Failed to initialize PSDKManager!\n");
  }

  if (!initGimbalManager())
  {
    DERROR("Failed to initialize GimbalManager!\n");
  }

  if (!initMissionManager())
  {
    DERROR("Failed to initialize Mission Manager!\n");
    return false;
  }
#if defined(__linux__)
  if (!initWaypointV2Mission())
  {
    DERROR("Failed to initialize WaypointV2Mission!\n");
    return false;
  }
#endif
  if (!initHardSync())
  {
    DERROR("Failed to initialize HardSync!\n");
    return false;
  }

  if(!initFlightController())
  {
    DERROR("Failed to initialize FlightController!\n");
    return false;
  }

  if(!initFirewall())
  {
    DERROR("Failed to initialize firewall!\n");
    return false;
  }
#if defined(__linux__)
  if(!initDJIHms())
  {
    DERROR("Failed to initialize DJIHMS!\n");
    return false;
  }
#endif
  if(!initDJIBattery())
  {
    DERROR("Failed to initialize DJIBattery!\n");
    return false;
  }

#ifdef ADVANCED_SENSING
  /*! If M300 here will use a new linker to do usb bulk
   * */
  if (!linker->isUSBPlugged()) {
    DSTATUS( "USB is not plugged or initialized successfully. "
             "Advacned-Sensing will not run.");
  } else {
    if (!initAdvancedSensing()) {
      DERROR("Failed to initialize AdvancedSensing!\n");
      return false;
    } else {
      DSTATUS("Start advanced sensing initalization");
    }
  }
#endif

#if defined(__linux__)
  /*! mop init should be here */
  if (!initMopServer())
  {
    DERROR("Failed to initialize MopServer!\n");
  }
#endif

  return true;
}

int
Vehicle::functionalSetUp()
{
  uint16_t tryTimes = 20;
  bool shakeHandRet = false;

  for (uint16_t i = 0; i < tryTimes; i++) {
    shakeHandRet = initVersion();
    if (shakeHandRet == true) {
      DSTATUS("Shake hand with drone successfully by getting drone version.");
      break;
    } else {
      DSTATUS("Shake hand with drone Fail ! Cannot get drone version. (%d/%d)",
              i + 1, tryTimes);
      DSTATUS("Try again after 1 second ......");
    }
    Platform::instance().taskSleepMs(1000);
  }

  if (shakeHandRet == false) {
    DERROR("Cannot connect with drone, block at here ...");
    while (true) {
      Platform::instance().taskSleepMs(1000);
    }
  }

  if (!this->isM210V2() && !this->isM300())
  {
    DERROR("Only support M210 V2 series and M300!\n");
    return false;
  }

  if(!this->init()) {
    DERROR("vehicle init fail. Exiting.");
    return false;
  }

  return false;
}

Vehicle::~Vehicle()
{
  if(sendHeartbeatToFCHandle)
  {
    OsdkOsal_TaskDestroy(sendHeartbeatToFCHandle);
  }

  if (this->subscribe)
  {
    subscribe->verify(1);
    subscribe->reset(1);
  }
  if(this->camera)
  {
    delete this->camera;
  }

  if(this->gimbal)
  {
    delete this->gimbal;
  }

  if(this->control)
  {
    delete this->control;
  }

  if (this->mfio)
  {
    delete this->mfio;
  }

  if (this->mobileDevice)
  {
    delete this->mobileDevice;
  }

  if (this->payloadDevice)
  {
    delete this->payloadDevice;
  }
  if (this->cameraManager)
  {
    delete this->cameraManager;
  }
  if (this->psdkManager)
  {
    delete this->psdkManager;
  }
  if (this->broadcast)
  {
    delete this->broadcast;
  }
  if (this->subscribe)
  {
    delete this->subscribe;
  }
  if (this->hardSync)
  {
    delete this->hardSync;
  }
  if (this->missionManager)
  {
    delete this->missionManager;
  }
#if defined(__linux__)
  if (this->waypointV2Mission)
  {
    delete this->waypointV2Mission;
  }
#endif
  if(this->flightController)
  {
    delete this->flightController;
  }
  if(this->legacyLinker)
  {
    delete this->legacyLinker;
  }

  if(this->firewall)
  {
    delete this->firewall;
  }
#if defined(__linux__)
  if(this->djiHms)
  {
    delete this->djiHms;
  }
  if(this->mopServer)
  {
    delete this->mopServer;
  }
#endif
#ifdef ADVANCED_SENSING
  if (this->advancedSensing)
    delete this->advancedSensing;
#endif

}


bool
Vehicle::initVersion()
{
#if STM32
  //! Non blocking call for STM32 as it does not support multi-thread
  getDroneVersion(2000);
  if(this->getFwVersion() > 0)
  {
    return true;
  }
#else
  ACK::DroneVersion rc = getDroneVersion(wait_timeout);
  if (!ACK::getError(rc.ack))
  {
    return true;
  }
#endif
  return false;
}

bool
Vehicle::initLegacyLinker(){
  if(this->legacyLinker)
  {
    DDEBUG("vehicle->legacyLinker already initalized!");
    return true;
  }

  this->legacyLinker = new (std::nothrow) LegacyLinker(this);
  if (this->legacyLinker == 0)
  {
    DERROR("Failed to allocate memory for LegacyLinker!\n");
    return false;
  }

  return true;
}

bool
Vehicle::initControl()
{
  if(this->control)
  {
    DDEBUG("vehicle->control already initalized!");
    return true;
  }

  if (isCmdSetSupported(OpenProtocolCMD::CMDSet::control))
  {
    this->control = new (std::nothrow) Control(this);
    if (this->control == 0)
    {
      DERROR("Failed to allocate memory for Control!\n");
      return false;
    }
  }
  else
  {
    DSTATUS("Control functionalities are not supported on this platform!\n");
  }

  return true;
}


bool
Vehicle::initSubscriber()
{
  if(this->subscribe)
  {
    DDEBUG("vehicle->subscribe already initalized!");
    return true;
  }

  if (isCmdSetSupported(OpenProtocolCMD::CMDSet::subscribe))
  {
    this->subscribe = new DataSubscription(this);
    if (this->subscribe == 0)
    {
      DERROR("Failed to allocate memory for Subscriber!\n");
      return false;
    }

    bool ret = this->legacyLinker->registerCMDCallback(
        OpenProtocolCMD::CMDSet::Broadcast::subscribe[0],
        OpenProtocolCMD::CMDSet::Broadcast::subscribe[1],
        this->subscribe->subscriptionDataDecodeHandler.callback,
        this->subscribe->subscriptionDataDecodeHandler.userData);
    /*
     * Wait for 1.2 seconds, so we can detect all leftover
     * packages from unclean quit, and remove them properly
     */
    if (!ret) {
      DERROR("Register broadcast callback fail.");
      return ret;
    }
    Platform::instance().taskSleepMs(1200);
    this->subscribe->removeLeftOverPackages();
  }
  else
  {
    DSTATUS("Telemetry subscription mechanism is not supported on this platform!\n");
  }

  return true;
}


bool
Vehicle::initBroadcast()
{
  if(this->broadcast)
  {
    DDEBUG("vehicle->broadcast already initalized!");
    return true;
  }

  if (isCmdSetSupported(OpenProtocolCMD::CMDSet::broadcast))
  {
    this->broadcast = new (std::nothrow) DataBroadcast(this);
    if (this->broadcast == 0)
    {
      DERROR("Failed to allocate memory for Broadcast!\n");
      return false;
    }
    bool ret = this->legacyLinker->registerCMDCallback(
        OpenProtocolCMD::CMDSet::Broadcast::broadcast[0],
        OpenProtocolCMD::CMDSet::Broadcast::broadcast[1],
        this->broadcast->unpackHandler.callback,
        this->broadcast->unpackHandler.userData);
    if (!ret) DERROR("Register broadcast callback fail.");
    return ret;
  }
  else
  {
    DSTATUS("Telemetry broadcast is not supported on this platform!\n");
  }

  return true;
}

bool
Vehicle::initCamera()
{
  if(this->camera)
  {
    DDEBUG("vehicle->camera already initalized!");
    return true;
  }

  this->camera = new (std::nothrow) Camera(this);

  if (this->camera == 0)
  {
    DERROR("Failed to allocate memory for Camera!\n");
    return false;
  }
  return true;
}

bool
Vehicle::initGimbal()
{
  if(this->gimbal)
  {
    DDEBUG("vehicle->gimbal already initalized!");
    return true;
  }

  this->gimbal = new (std::nothrow) Gimbal(this);

  if (this->gimbal == 0)
  {
    DERROR("Failed to allocate memory for Gimbal!\n");
    return false;
  }

  return true;
}


bool
Vehicle::initFlightController()
{
  if(this->flightController)
  {
    DDEBUG("flightController already initalized!");
    return true;
  }
  this->flightController = new (std::nothrow) FlightController(this);
  if (this->flightController == 0)
  {
    DERROR("Failed to allocate memory for flightController!\n");
    return false;
  }
  return true;
}


bool
Vehicle::initMFIO()
{
  if(this->mfio)
  {
    DDEBUG("vehicle->mfio already initalized!");
    return true;
  }

  if (isCmdSetSupported(OpenProtocolCMD::CMDSet::mfio))
  {
    mfio = new (std::nothrow) MFIO(this);
    if (this->mfio == 0)
    {
      DERROR("Failed to allocate memory for MFIO!\n");
      return false;
    }
  }
  else
  {
    DSTATUS("MFIO is not supported on this platform!\n");
  }

  return true;
}

bool
Vehicle::initMobileDevice()
{
  if(this->mobileDevice)
  {
    DDEBUG("Mobile Device already initalized!");
    return true;
  }
  this->mobileDevice = new (std::nothrow) MobileDevice(this);
  if (this->mobileDevice == 0)
  {
    DERROR("Failed to allocate memory for MobileDevice!\n");
    return false;
  }
  return true;
}
bool Vehicle::initPayloadDevice()
{
  if(this->payloadDevice)
  {
    DDEBUG("Payload Device already initalized!");
    return true;
  }
  this->payloadDevice = new (std::nothrow) PayloadDevice(this);
  if (this->payloadDevice == 0)
  {
    DERROR("Failed to allocate memory for payload Device!\n");
    return false;
  }
  return true;

}


bool Vehicle::initCameraManager()
{
  if(this->cameraManager)
  {
    DDEBUG("cameraManager already initalized!");
    return true;
  }
  this->cameraManager = new (std::nothrow) CameraManager(this);
  if (this->cameraManager == 0)
  {
    DERROR("Failed to allocate memory for pm!\n");
    return false;
  }
  return true;
}

bool Vehicle::initPSDKManager()
{
  if(this->psdkManager)
  {
    DDEBUG("psdkManager already initalized!");
    return true;
  }
  this->psdkManager = new (std::nothrow) PSDKManager(this);
  if (this->psdkManager == 0)
  {
    DERROR("Failed to allocate memory for PSDKManager!\n");
    return false;
  }
  return true;
}

bool Vehicle::initGimbalManager()
{
  if(this->gimbalManager)
  {
    DDEBUG("gimbalManager already initalized!");
    return true;
  }
  this->gimbalManager = new (std::nothrow) GimbalManager(this);
  if (this->gimbalManager == 0)
  {
    DERROR("Failed to allocate memory for gimbalManager!\n");
    return false;
  }
  return true;
}

#if defined(__linux__)

bool Vehicle::initMopServer()
{
  if (!this->isM300()) return true;
  if(this->mopServer)
  {
    DDEBUG("mopServer already initalized!");
    return true;
  }
  this->mopServer = new (std::nothrow) MopServer();
  if (this->mopServer == 0)
  {
    DERROR("Failed to allocate memory for mopServer!\n");
    return false;
  }
  return true;
}
#endif

bool
Vehicle::initMissionManager()
{
  if(this->missionManager)
  {
    DDEBUG("vehicle->missionManager already initalized!");
    return true;
  }

  this->missionManager = new (std::nothrow) MissionManager(this);
  if (this->missionManager == 0)
  {
    return false;
  }
  return true;
}
#if defined(__linux__)
bool
Vehicle::initWaypointV2Mission() {
  if(this->waypointV2Mission)
  {
    DDEBUG("vehicle->waypointV2Mission already initalized!");
    return true;
  }

  this->waypointV2Mission = new (std::nothrow) WaypointV2MissionOperator(this);
  if (this->waypointV2Mission == 0)
  {
    return false;
  }
  return true;
}
#endif

bool
Vehicle::initHardSync()
{
  if(this->hardSync)
  {
    DDEBUG("vehicle->hardSync already initalized!");
    return true;
  }

  if (isCmdSetSupported(OpenProtocolCMD::CMDSet::hardwareSync))
  {
    hardSync = new (std::nothrow) HardwareSync(this);
    if (this->hardSync == 0)
    {
      return false;
    }
  }
  else
  {
    DSTATUS("Hardware Sync is not supported on this platform!\n");
  }

  return true;
}

bool Vehicle::initFirewall() {
  /*! not support then return true */
  if (!this->isM300()) return true;
  /*! initialized then return true */
  if (this->firewall) return true;
  firewall = new (std::nothrow) Firewall(this->linker);
  if (this->firewall == 0)
  {
    DERROR("Error initialize firewall!");
    return false;
  }
  return true;
}

#ifdef ADVANCED_SENSING
bool
Vehicle::initAdvancedSensing()
{
  if(this->advancedSensing)
  {
    DDEBUG("vehicle->advancedSensing already initalized!");
    return true;
  }

  this->advancedSensing = new (std::nothrow) AdvancedSensing(this);
  if (this->advancedSensing == 0)
  {
    DERROR("Failed to allocate memory for AdvancedSensing!\n");
    return false;
  }
  this->advancedSensing->init();

  return true;
}
#endif


#ifdef ADVANCED_SENSING
void
Vehicle::processAdvancedSensingImgs(RecvContainer* receivedFrame)
{
  if (receivedFrame->recvInfo.cmd_id == AdvancedSensingProtocol::PROCESS_IMG_CMD_ID)
  {
    if (this->advancedSensing->stereoHandler.callback)
    {
      this->advancedSensing->stereoHandler.callback(
          this, *receivedFrame, this->advancedSensing->stereoHandler.userData);
    }
    else
    {
      this->advancedSensing->unsubscribeStereoImages();
      if(!advSensingErrorPrintOnce){
        DERROR("No callback registered for 240p stereo images.\n"
               "This usually happens when user subscribed to images and restart "
               "the program without unsubscribing them.\n"
               "Vehicle unsubscribed 240p stereo images automatically.\n");
        advSensingErrorPrintOnce = true;
      }
    }
  }
  else if (receivedFrame->recvInfo.cmd_id ==
           AdvancedSensingProtocol::PROCESS_VGA_CMD_ID)
  {
    if (this->advancedSensing->vgaHandler.callback)
    {
      this->advancedSensing->vgaHandler.callback(this, *receivedFrame,
                                                 this->advancedSensing->vgaHandler.userData);
    }
    else
    {
      this->advancedSensing->unsubscribeVGAImages();
      if(!advSensingErrorPrintOnce){
        DERROR("No callback registered for VGA stereo images.\n"
               "This usually happens when user subscribed to images and restart "
               "the program without unsubscribing them.\n"
               "Vehicle unsubscribed VGA stereo images automatically.\n");
        advSensingErrorPrintOnce = true;
      }
    }
  }
}
#endif
#if defined(__linux__)
bool
Vehicle::initDJIHms() {
    if(this->djiHms)
    {
        DDEBUG("vehicle->djiHms already initalized!");
        return true;
    }

    if(this->isM300())
    {
        djiHms = new (std::nothrow) DJIHMS(this);
        if (this->djiHms == 0)
        {
            DERROR("Error creating DJI HMS!");
            return false;
        }
    }
    else
    {
        DSTATUS("DJI HMS is not supported on this platform!\n");
    }

    return true;
}
#endif
bool
Vehicle::initOSDKHeartBeatThread() {
    /*! create task for OSDK heart beat */
    if(!sendHeartbeatToFCHandle) {
      E_OsdkStat osdkStat = OsdkOsal_TaskCreate(&sendHeartbeatToFCHandle,
                                                (void *(*)(
                                                    void *)) (sendHeartbeatToFCTask),
                                                OSDK_TASK_STACK_SIZE_DEFAULT,
                                                this->linker);
      if (osdkStat != OSDK_STAT_OK) {
        DERROR("osdk heart beat task create error:%d", osdkStat);
        return false;
      }
    }

    return true;
}

bool
Vehicle::initDJIBattery()
{
    if(this->djiBattery)
    {
        DDEBUG("vehicle->djiBattery already initalized!");
        return true;
    }

    this->djiBattery= new (std::nothrow) DJIBattery(this);

    if (this->djiBattery == 0)
    {
        DERROR("Failed to allocate memory for DJI Battery!\n");
        return false;
    }

    return true;
}
bool
Vehicle::parseDroneVersionInfo(Version::VersionData& versionData,
                               uint8_t*              ackPtr)
{

  Version::VersionData versionStruct;

  //! Note down our starting point as a sanity check
  uint8_t* startPtr = ackPtr;
  //! 2b ACK.
  versionStruct.version_ack = ackPtr[0] + (ackPtr[1] << 8);
  ackPtr += 2;

  //! Next, we might have CRC or ID; Put them into a variable that we will parse
  //! later. Find next \0
  uint8_t crc_id[16] = {};
  int     i          = 0;
  while (*ackPtr != '\0')
  {
    crc_id[i] = *ackPtr;
    i++;
    ackPtr++;
    if (ackPtr - startPtr > 18)
    {
      return false;
    }
  }
  //! Fill in the termination character
  crc_id[i] = *ackPtr;
  ackPtr++;

  //! Now we're at the name. First, let's fill up the name field.
  memcpy(versionStruct.version_name, ackPtr, 32);

  //! Now, we start parsing the name. Let's find the second space character.
  while (*ackPtr != ' ')
  {
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      return false;
    }
  } //! Found first space ("SDK-v1.x")
  ackPtr++;

  while (*ackPtr != ' ')
  {
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      return false;
    }
  } //! Found second space ("BETA")
  ackPtr++;

  //! Next is the HW version
  int j = 0;
  while (*ackPtr != '-')
  {
    versionStruct.hwVersion[j] = *ackPtr;
    ackPtr++;
    j++;
    if (ackPtr - startPtr > 64)
    {
      return false;
    }
  }
  //! Fill in the termination character
  versionStruct.hwVersion[j] = '\0';
  ackPtr++;

  //! Finally, we come to the FW version. We don't know if each clause is 2 or 3
  //! digits long.
  int ver1 = 0, ver2 = 0, ver3 = 0, ver4 = 0;

  while (*ackPtr != '.')
  {
    ver1 = (*ackPtr - 48) + 10 * ver1;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      return false;
    }
  }
  ackPtr++;
  while (*ackPtr != '.')
  {
    ver2 = (*ackPtr - 48) + 10 * ver2;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      return false;
    }
  }
  ackPtr++;
  while (*ackPtr != '.')
  {
    ver3 = (*ackPtr - 48) + 10 * ver3;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      return false;
    }
  }
  ackPtr++;
  while (*ackPtr != '\0')
  {
    ver4 = (*ackPtr - 48) + 10 * ver4;
    ackPtr++;
    if (ackPtr - startPtr > 64)
    {
      return false;
    }
  }

  versionStruct.fwVersion = Version::FW(ver1, ver2, ver3, ver4);

  //! Special cases
  //! M100:
  if (strcmp(versionStruct.hwVersion, Version::M100) == 0)
  {
    //! Bug in M100 does not report the right FW.
    ver3                    = 10 * ver3;
    versionStruct.fwVersion = Version::FW(ver1, ver2, ver3, ver4);
  }
  //! M600/A3 FW 3.2.10
  if (versionStruct.fwVersion == Version::FW(3, 2, 10, 0))
  {
    //! Bug in M600 does not report the right FW.
    ver3                    = 10 * ver3;
    versionStruct.fwVersion = Version::FW(ver1, ver2, ver3, ver4);
  }

  //! Now, we can parse the CRC and ID based on FW version. If it's older than
  //! 3.2 then it'll have a CRC, else not.
  if (versionStruct.fwVersion < Version::FW(3, 2, 0, 0))
  {
    versionStruct.version_crc =
      crc_id[0] + (crc_id[1] << 8) + (crc_id[2] << 16) + (crc_id[3] << 24);
    uint8_t* id_ptr = &crc_id[4];

    int i = 0;
    while (*id_ptr != '\0')
    {
      versionStruct.hw_serial_num[i] = *id_ptr;
      i++;
      id_ptr++;
      if (id_ptr - &crc_id[4] > 12)
      {
        return false; //! Not catastrophic error
      }
    }
    //! Fill in the termination character
    versionStruct.hw_serial_num[i] = *id_ptr;
  }
  else
  {
    versionStruct.version_crc = 0;
    uint8_t* id_ptr           = &crc_id[0];

    int i = 0;
    while (*id_ptr != '\0')
    {
      versionStruct.hw_serial_num[i] = *id_ptr;
      i++;
      id_ptr++;
      if (id_ptr - &crc_id[0] > 16)
      {
        return false; //! Not catastrophic error
      }
    }
    //! Fill in the termination character
    versionStruct.hw_serial_num[i] = *id_ptr;
  }

  //! Finally, we print stuff out.

  if (versionStruct.fwVersion > Version::FW(3, 1, 0, 0))
  {
    DSTATUS("Device Serial No. = %.16s", versionStruct.hw_serial_num);
  }
  DSTATUS("Firmware = %d.%d.%d.%d", ver1, ver2, ver3, ver4);
  if (versionStruct.fwVersion < Version::FW(3, 2, 0, 0))
  {
    DSTATUS("Version CRC = 0x%X", versionStruct.version_crc);
  }

  versionData = versionStruct;
  return true;
}


bool Vehicle::setUSBFlightOn(bool en) {
  if (!linker->isUSBPlugged()) {
    return true;
  }
#pragma pack(1)
  typedef struct USBCtrlData
  {
    uint16_t version;
    uint8_t  cmd;
  } USBCtrlData;
#pragma pack()
  uint8_t retryTimes = 4;
  uint16_t l = 1;
  uint16_t h = 1;
  uint16_t c = (h << 8) | (l & 0xff);
  uint8_t lBit = c & 1;
  uint8_t hBit = (uint16_t)c >> 15;
  c |= 1 << 8;
  c |= 1 << 15;

  USBCtrlData data;
  memset(&data, 0, sizeof(data));
  data.version = c;
  data.cmd = (en) ? 1 : 0;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t cbData[1024] = {0};

  cmdInfo.cmdSet = V1ProtocolCMD::fc::usbFlightMode[0];
  cmdInfo.cmdId = V1ProtocolCMD::fc::usbFlightMode[1];
  cmdInfo.dataLen = sizeof(data);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_FC_DEVICE_ID;
  cmdInfo.sender = linker->getLocalSenderId();
retryUSBFlight:
  DSTATUS("Trying to set usb-connected-flight as [%s]", en ? "enable" : "disable");
  E_OsdkStat ret =
      linker->sendSync(&cmdInfo, (uint8_t*)&data, &ackInfo, cbData, 1000, 3);
  if ((ret != OSDK_STAT_OK) || (cbData[0] != 0x00)) {
    retryTimes--;
    if (retryTimes > 0) {
      OsdkOsal_TaskSleepMs(1000);
      goto retryUSBFlight;
    } else {
      DERROR("Configure usb-connected-flight failed! Cannot take off !");
      return false;
    }
  } else {
    DSTATUS("Configure usb-connected-flight successfully.");
    return true;
  }
}

bool Vehicle::setSimulationOn(bool en, float64_t latitude, float64_t longitude) {
#pragma pack(1)
  typedef struct SimulationData
  {
    uint8_t	cmd;
    uint8_t	rc : 1;
    uint8_t	model : 1;
    uint8_t	resv : 6;
    uint8_t	freq;
    uint8_t	gps;
    double	lon;
    double	lat;
    double	height;
    uint8_t	roll : 1;
    uint8_t	pitch : 1;
    uint8_t	yaw : 1;
    uint8_t	x : 1;
    uint8_t	y : 1;
    uint8_t	z : 1;
    uint8_t	lati : 1;
    uint8_t	longti : 1;
    uint8_t	speed_x : 1;
    uint8_t	speed_y : 1;
    uint8_t	speed_z : 1;
    uint8_t	acc_x : 1;
    uint8_t	acc_y : 1;
    uint8_t	acc_z : 1;
    uint8_t	p : 1;
    uint8_t	q : 1;
    uint8_t	r : 1;
    uint8_t	rpm1 : 1;
    uint8_t	rpm2 : 1;
    uint8_t	rpm3 : 1;
    uint8_t	rpm4 : 1;
    uint8_t	rpm5 : 1;
    uint8_t	rpm6 : 1;
    uint8_t	rpm7 : 1;
    uint8_t	rpm8 : 1;
    uint8_t	duration : 1;
    uint8_t	led_color : 1;
    uint8_t	transform_state : 1;
    uint32_t	resv1 : 4;
    uint32_t	reserve;
  } SimulationData; 
#pragma pack()
#define M_PI 3.14159265358979323846
  SimulationData data = {0};

  data.cmd = en ? 0:2;
  data.rc = 1;
  data.freq = 20;
  data.gps = 20;

  data.lat = latitude * M_PI / 180;
  data.lon = longitude * M_PI / 180;

  data.height = 0.1;
  data.roll = data.pitch = data.yaw = data.x = data.y = data.z = 1;
  data.lati = data.longti = 1;
  data.speed_x = data.speed_y = data.speed_z = 1;
  data.duration = 1;
  data.led_color = data.transform_state = 1;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t cbData[1024] = {0};

  cmdInfo.cmdSet = 0x0B;
  cmdInfo.cmdId = 0x04;
  cmdInfo.dataLen = sizeof(data);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_FC_DEVICE_ID;
  cmdInfo.sender = linker->getLocalSenderId();
  E_OsdkStat ret =
      linker->sendSync(&cmdInfo, (uint8_t*)&data, &ackInfo, cbData, 1000, 3);
  if (ret != OSDK_STAT_OK) {
    DERROR("Caution !!! Simulation setting error!");
    return false;
  } else {
    if (en && (cbData[0] == 3)) {  // 3 means agree to start simulation
      DSTATUS("Start simulation successfully.");
      return true;
    } else if (!en && (cbData[0] == 5)) {  // 5 means agree to stop simulation
      DSTATUS("Stop simulation successfully.");
      return true;
    } else {
       DERROR("%s simulation failed.", en ? "start" : "stop");
      return false;
    }
  }
}

ACK::ErrorCode
Vehicle::activate(ActivateData* data, uint32_t timeoutMs)
{
  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t cbData[1024];
  ACK::ErrorCode ack;

  if(this->functionalSetUp() != 0)
  {
    DERROR("Unable to initialize some vehicle components!");
    ack.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
    return ack;
  }

#if 0
  /*! M300 drone do the firewall logic */
  uint8_t retryTimes = 0;
  if (this->isM300()
      && this->linker->isUSBPlugged()
      && !firewall->isPolicyUpdated()) {
    firewall->setAppKey((uint8_t *) data->encKey, strlen(data->encKey) - 1);
    do {
      retryTimes ++;
      DSTATUS("osdk policy file updating(1) ......");
      OsdkOsal_TaskSleepMs(1000);
    } while ((!firewall->RequestUpdatePolicy()) && (retryTimes < 15));

    /*! pending for firewall logic finished */
    retryTimes = 0;
    do {
      retryTimes++;
      DSTATUS("osdk policy file updating(2) ......");
      OsdkOsal_TaskSleepMs(1000);
    } while ((!firewall->isPolicyUpdated()) && (retryTimes < 15));
  }
#endif

  data->version        = versionData.fwVersion;
  accountData          = *data;
  accountData.reserved = 2;

  for (int i             = 0; i < 32; ++i)
    accountData.iosID[i] = '0'; //! @note for ios verification
  DSTATUS("version 0x%X\n", versionData.fwVersion);
  DDEBUG("%.32s", accountData.iosID);

  cmdInfo.cmdSet = OpenProtocolCMD::CMDSet::Activation::activate[0];
  cmdInfo.cmdId  = OpenProtocolCMD::CMDSet::Activation::activate[1];
  cmdInfo.dataLen = sizeof(accountData) - sizeof(char*);
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);
  cmdInfo.protoType = PROTOCOL_SDK;
  cmdInfo.channelId = 0;

  if(timeoutMs > 3)
  {
    timeoutMs = 3;
  }

  uint8_t activateRetryTimes = 0;
  while (1)
  {
    /*! Try several times to avoid the NEW_DEVICE_ERROR issues */
    ack = *(ACK::ErrorCode*)legacyLinker->sendSync(
      OpenProtocolCMD::CMDSet::Activation::activate,
      (uint8_t*)&accountData,
      sizeof(accountData) - sizeof(char*),
      timeoutMs * 1000 / 3,
      3);
    if ((ack.data == OpenProtocolCMD::ErrorCode::ActivationACK::SUCCESS) ||
        (activateRetryTimes >= 3))
      break;
    OsdkOsal_TaskSleepMs(1000);
    DSTATUS("Retry to activate again ..");
    activateRetryTimes++;
  }

  if (ack.data == OpenProtocolCMD::ErrorCode::ActivationACK::SUCCESS &&
      accountData.encKey)
  {
    DSTATUS("Activation successful\n");
    for (uint8_t i = 0; i < MAX_SEND_DATA_BURY_PKG_COUNT; i++)
    {
        sendBuriedDataPkgToFC();
        OsdkOsal_TaskSleepMs(200);
    }
    linker->setKey(accountData.encKey);
    setActivationStatus(true);
  }
  else
  {
    //! Let user know about other errors if any
    ACK::getErrorCodeMessage(ack, __func__);
    DERROR("Failed to activate please retry SET 0x%X ID 0x%X code 0x%X\n",
           ack.info.cmd_set, ack.info.cmd_id, ack.data);

    if(ack.info.cmd_set == OpenProtocolCMD::CMDSet::activation
        && ack.info.cmd_id == 1
        && ack.data == ErrorCode::ActivationACK::NEW_DEVICE_ERROR )
    {
      DERROR("Solutions for NEW_DEVICE_ERROR:\n"
             "\t* Double-check your app_id and app_key in UserConfig.txt. "
             "Does it match with your DJI developer account?\n"
             "\t* If this is a new device, you need to activate it through the App or DJI Assistant 2 with Internet\n"
             "\tFor different aircraft, the App and the version of DJI Assistant 2 might be different\n"
             "\tFor A3, N3, M600/Pro and M100, please use DJI GO App\n"
             "\tFor M210 V1, please use DJI GO 4 App or DJI Pilot App\n"
             "\tFor M210 V2, please use DJI Pilot App\n"
             "\tFor DJI Assistant 2, it's available on the 'Download' tab of the product page\n"
             "\t* If this device is previously activated with another app_id and app_key, "
             "you will need to re-activate it again.\n"
             "\t* A new device needs to be activated twice to fix the NEW_DEVICE_ERROR, "
             "so please try it twice.\n");
    }
  }

  return ack;
}


void
Vehicle::activate(ActivateData* data, VehicleCallBack callback,
                  UserData userData)
{
  if(this->functionalSetUp() != 0)
  {
    DERROR("Unable to initialize some vehicle components!");
    return;
  }

#if 0
  /*! M300 drone do the firewall logic */
  uint8_t retryTimes = 0;
  if (this->isM300()
      && this->linker->isUSBPlugged()
      && !firewall->isPolicyUpdated()) {
    firewall->setAppKey((uint8_t *) data->encKey, strlen(data->encKey) - 1);
    do {
      retryTimes ++;
      DSTATUS("osdk policy file updating(1) ......");
      OsdkOsal_TaskSleepMs(1000);
    } while ((!firewall->RequestUpdatePolicy()) && (retryTimes < 15));

    /*! pending for firewall logic finished */
    retryTimes = 0;
    do {
      retryTimes++;
      DSTATUS("osdk policy file updating(2) ......");
      OsdkOsal_TaskSleepMs(1000);
    } while ((!firewall->isPolicyUpdated()) && (retryTimes < 15));
  }
#endif

  data->version        = this->versionData.fwVersion;
  accountData          = *data;
  accountData.reserved = 2;

  for (int i = 0; i < 32; ++i)
  {
    accountData.iosID[i] = '0'; //! @note for ios verification
  }
  DSTATUS("version 0x%X\n", versionData.fwVersion);
  DDEBUG("%.32s", accountData.iosID);
  //! Using function prototype II of send
  VehicleCallBack cb = NULL;
  UserData udata = NULL;
  if (callback)
  {
    cb = callback;
    udata = userData;
  }
  else
  {
    cb = activateCallback;
    udata = NULL;
  }
  legacyLinker->sendAsync(OpenProtocolCMD::CMDSet::Activation::activate,
                          (uint8_t *) &accountData,
                          sizeof(accountData) - sizeof(char *), 1000, 3, cb,
                          udata);
  for (uint8_t i = 0; i < MAX_SEND_DATA_BURY_PKG_COUNT; i++)
  {
      sendBuriedDataPkgToFC();
      OsdkOsal_TaskSleepMs(200);
  }
}


void
Vehicle::activateCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                          UserData userData)
{

  uint16_t ack_data;
  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= 2)
  {
    ack_data = recvFrame.recvData.ack;

    vehiclePtr->ackErrorCode.data = ack_data;
    vehiclePtr->ackErrorCode.info = recvFrame.recvInfo;

    if (ACK::getError(vehiclePtr->ackErrorCode) &&
        ack_data ==
            OpenProtocolCMD::ErrorCode::ActivationACK::OSDK_VERSION_ERROR)
    {
      DERROR("SDK version did not match\n");
      vehiclePtr->getDroneVersion();
    }

    //! Let user know about other errors if any
    ACK::getErrorCodeMessage(vehiclePtr->ackErrorCode, __func__);
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  if (ack_data == OpenProtocolCMD::ErrorCode::ActivationACK::SUCCESS &&
      vehiclePtr->accountData.encKey)
  {
    vehiclePtr->linker->setKey(vehiclePtr->accountData.encKey);
    vehiclePtr->setActivationStatus(true);
  }
  else if(ack_data == OpenProtocolCMD::ErrorCode::ActivationACK::NEW_DEVICE_ERROR)
  {
    DERROR("Solutions for NEW_DEVICE_ERROR:\n"
           "\t* Double-check your app_id and app_key in UserConfig.txt. "
           "Does it match with your DJI developer account?\n"
           "\t* If this is a new device, you need to activate it through the App or DJI Assistant 2 with Internet\n"
           "\tFor different aircraft, the App and the version of DJI Assistant 2 might be different\n"
           "\tFor A3, N3, M600/Pro and M100, please use DJI GO App\n"
           "\tFor M210 V1, please use DJI GO 4 App or DJI Pilot App\n"
           "\tFor M210 V2, please use DJI Pilot App\n"
           "\tFor DJI Assistant 2, it's available on the 'Download' tab of the product page\n"
           "\t* If this device is previously activated with another app_id and app_key, "
           "you will need to re-activate it again.\n"
           "\t* A new device needs to be activated twice to fix the NEW_DEVICE_ERROR, "
           "so please try it twice.\n");
  }
}

void
Vehicle::fcLostConnectCallBack(void)
{
    DSTATUS("OSDK lost connection with Drone!");
}

uint8_t
Vehicle::sendHeartbeatToFCFunc(Linker *linker)
{
    if (linker) {
        uint8_t ackData[1024];
        uint8_t *data = (uint8_t *) &heartBeatPack;
        T_CmdInfo heatBeatCmdInfo    = {0};
        T_CmdInfo heatBeatAckCmdInfo = {0};
        heatBeatCmdInfo.cmdSet     = OpenProtocolCMD::CMDSet::Activation::heatBeatCmd[0];
        heatBeatCmdInfo.cmdId      = OpenProtocolCMD::CMDSet::Activation::heatBeatCmd[1];
        heatBeatCmdInfo.dataLen    = sizeof(HeartBeatPack);
        heatBeatCmdInfo.needAck    = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
        heatBeatCmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
        heatBeatCmdInfo.protoType  = PROTOCOL_SDK;
        heatBeatCmdInfo.addr       = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);
        heatBeatCmdInfo.channelId  = 0;

        linker->sendSync(&heatBeatCmdInfo, data,
                         &heatBeatAckCmdInfo, ackData, 500, 2);
        HeartBeatPack *ack = (HeartBeatPack *)ackData;
        if (ack->seqNumber == heartBeatPack.seqNumber) {
            fcLostConnectCount = 0;
            osdkConnectFCFlag = 1;
        } else {
            fcLostConnectCount++;
        }
        if (fcLostConnectCount > kMaxFCLostConnectCount) {
            osdkConnectFCFlag = 0;
            DJI::OSDK::Vehicle::fcLostConnectCallBack();
        }
        heartBeatPack.seqNumber++;

        return true;
    } else {
        return false;
    }
}

void *
Vehicle::sendHeartbeatToFCTask(void *arg) {
    OsdkOsal_TaskSleepMs(1000);
    DSTATUS("OSDK send heart beat to fc task created.");
    if(arg) {
      Linker *linker = (Linker *) arg;
      static uint32_t preHeartBeatTimeStamp = 0;
      static uint32_t curHeartBeatTimeStamp = 0;
      for (;;)
      {
          OsdkOsal_GetTimeMs(&curHeartBeatTimeStamp);
          if (curHeartBeatTimeStamp - preHeartBeatTimeStamp >=  kHeartBeatPackSendTimeInterval)
          {
            if (linker->isUartPlugged()) {
              DJI::OSDK::Vehicle::sendHeartbeatToFCFunc(linker);
            }
              preHeartBeatTimeStamp = curHeartBeatTimeStamp;
          }

          /*! TODO: with out sleep 100ms, the time will get the same as last time. */
          OsdkOsal_TaskSleepMs(100);
      }
    } else {
      DERROR("Osdk send heart beat to fc task run failed because of the invalid linker "
             "ptr. Please recheck this task params.");
    }
  return NULL;
}

void
Vehicle::getDroneVersionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                                 UserData userData)
{

  if (!parseDroneVersionInfo(vehiclePtr->versionData,
                             recvFrame.recvData.versionACK))
  {
    DERROR("Drone version not obtained! Please do not proceed.\n"
           "Possible reasons:\n"
           "\tSerial port connection:\n"
           "\t\t* SDK is not enabled, please check DJI Assistant2 -> SDK -> [v] Enable API Control.\n"
           "\t\t* Baudrate is not correct, please double-check from DJI Assistant2 -> SDK -> baudrate.\n"
           "\t\t* TX and RX pins are inverted.\n"
           "\t\t* Serial port is occupied by another program.\n"
           "\t\t* Permission required. Please do 'sudo usermod -a -G dialout $USER' "
           "(you do not need to replace $USER with your username). Then logout and login again\n");

    //! Set fwVersion to 0 so we can catch the error.
    vehiclePtr->versionData.fwVersion = 0;
  }
  else
  {
    //! Finally, we print stuff out.
    if (vehiclePtr->versionData.fwVersion > Version::FW(3, 1, 0, 0))
    {
      DSTATUS("Device Serial No. = %.16s\n",
              vehiclePtr->versionData.hw_serial_num);
    }
    DSTATUS("Hardware = %.12s\n", vehiclePtr->versionData.hwVersion);
    DSTATUS("Firmware = %X\n", vehiclePtr->versionData.fwVersion);
    if (vehiclePtr->versionData.fwVersion < Version::FW(3, 2, 0, 0))
    {
      DSTATUS("Version CRC = 0x%X\n", vehiclePtr->versionData.version_crc);
    }
    if (vehiclePtr->isM300() && vehiclePtr->linker) vehiclePtr->linker->setSenderId(OSDK_COMMAND_OSDK_DEVICE_ID);
    else vehiclePtr->linker->setSenderId(OSDK_COMMAND_PC_DEVICE_ID);
  }
}

void
Vehicle::getDroneVersion(VehicleCallBack callback, UserData userData)
{
  versionData.version_ack =
      OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  versionData.version_crc     = 0x0;
  versionData.version_name[0] = 0;
  versionData.fwVersion       = 0;

  uint32_t cmd_timeout = 100; // unit is ms
  uint32_t retry_time  = 3;
  uint8_t  cmd_data    = 0;
  VehicleCallBack cb = NULL;
  UserData udata = NULL;
  if (callback)
  {
    cb = callback;
    udata = userData;
  }
  else
  {
    cb = getDroneVersionCallback;
    udata = NULL;
  }

  // When UserData is implemented, pass the Vehicle as userData.
  legacyLinker->sendAsync(OpenProtocolCMD::CMDSet::Activation::getVersion,
                          (uint8_t *) &cmd_data, 1, cmd_timeout, retry_time, cb,
                          udata);
}

ACK::DroneVersion
Vehicle::getDroneVersion(uint32_t timeoutMs)
{
  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t data[1024];
  uint8_t  cmd_data = 0;

  droneVersionACK.ack.info.cmd_set = OpenProtocolCMD::CMDSet::Activation::getVersion[0];
  droneVersionACK.ack.info.cmd_id = OpenProtocolCMD::CMDSet::Activation::getVersion[1];

  versionData.version_ack =
    OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  versionData.version_crc     = 0x0;
  versionData.version_name[0] = 0;

  cmdInfo.cmdSet = OpenProtocolCMD::CMDSet::Activation::getVersion[0];
  cmdInfo.cmdId  = OpenProtocolCMD::CMDSet::Activation::getVersion[1];
  cmdInfo.dataLen = 1;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_SDK_COMMAND_INDEX);

  if(timeoutMs < 3)
  {
    timeoutMs = 3;
  }

  linker->sendSync(&cmdInfo, &cmd_data, &ackInfo, data, timeoutMs/3, 3);

  // Parse received data
  if (!parseDroneVersionInfo(this->versionData, data))
  {
    DERROR("Drone version not obtained! Please do not proceed.\n"
             "Possible reasons:\n"
             "\tSerial port connection:\n"
             "\t\t* SDK is not enabled, please check DJI Assistant2 -> SDK -> [v] Enable API Control.\n"
             "\t\t* Baudrate is not correct, please double-check from DJI Assistant2 -> SDK -> baudrate.\n"
             "\t\t* TX and RX pins are inverted.\n"
             "\t\t* Serial port is occupied by another program.\n"
             "\t\t* Permission required. Please do 'sudo usermod -a -G dialout $USER' "
             "(you do not need to replace $USER with your username). Then logout and login again\n");

    droneVersionACK.ack.data = this->versionData.version_ack;
    //! Set fwVersion to 0 so we can catch the error.
    this->versionData.fwVersion = 0;
  }
  else
  {
    //! Construct final ACK to return to user
    droneVersionACK.ack.data         = this->versionData.version_ack;

    // We can prompt for droneVersion without prior activation
    if(ACK::getError(droneVersionACK.ack) &&
       droneVersionACK.ack.data == OpenProtocolCMD::ErrorCode::CommonACK::NO_AUTHORIZATION_ERROR)
    {
      droneVersionACK.ack.data = ACK::SUCCESS;
    }

    droneVersionACK.data.version_ack = this->versionData.version_ack;
    droneVersionACK.data.version_crc = this->versionData.version_crc;
    droneVersionACK.data.fwVersion   = this->versionData.fwVersion;

    strncpy(droneVersionACK.data.version_name, this->versionData.version_name,
            sizeof(this->versionData.version_name));
    droneVersionACK.data.version_name[sizeof(this->versionData.version_name) - 1] =
      '\0';

    strncpy(droneVersionACK.data.hwVersion, this->versionData.hwVersion,
            sizeof(this->versionData.hwVersion));
    droneVersionACK.data.hwVersion[sizeof(this->versionData.hwVersion) - 1] = '\0';

    strncpy(droneVersionACK.data.hw_serial_num, this->versionData.hw_serial_num,
            sizeof(this->versionData.hw_serial_num));
    droneVersionACK.data.hw_serial_num[sizeof(this->versionData.hw_serial_num) - 1] =
      '\0';
    if (isM300() && linker) linker->setSenderId(OSDK_COMMAND_OSDK_DEVICE_ID);
    else linker->setSenderId(OSDK_COMMAND_PC_DEVICE_ID);
  }
  return droneVersionACK;
}

Vehicle::ActivateData
Vehicle::getAccountData() const
{
  return accountData;
}

void
Vehicle::setAccountData(const ActivateData& value)
{
  accountData = value;
}

/*****************************Set State Data**************************************/

void
Vehicle::setVersion(const Version::FirmWare& value)
{
  versionData.fwVersion = value;
}


Version::FirmWare
Vehicle::getFwVersion() const
{
  return versionData.fwVersion;
}
char*
Vehicle::getHwVersion() const
{
  return (char*)versionData.hwVersion;
}
char*
Vehicle::getHwSerialNum() const
{
  return (char*)versionData.hw_serial_num;
}

void
Vehicle::setEncryption(bool encryptSetting)
{
  this->encrypt = encryptSetting;
}

bool
Vehicle::getEncryption()
{
  return this->encrypt;
}

void Vehicle::setActivationStatus(bool is_activated)
{
  this->is_activated = is_activated;
}

bool Vehicle::getActivationStatus()
{
  return this->is_activated;
}

bool
Vehicle::isLegacyM600()
{
  //! Check for the special M600 backwards compatibility
  if (versionData.fwVersion == Version::FW(3, 2, 15, 62))
  {
    if (strncmp(versionData.hwVersion, "PM820V3", 7) == 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

bool
Vehicle::isM100()
{
  //! Check for the M100 backwards compatibility
  if (versionData.fwVersion == Version::FW(3, 1, 10, 0))
  {
    if (strncmp(versionData.hwVersion, Version::M100, 4) == 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  return false;
}

bool
Vehicle::isM210V2()
{
  if (strncmp(versionData.hwVersion, Version::M210V2, 5) == 0)
  {
    return true;
  }
  return false;
}

bool
Vehicle::isM300()
{
  if (strncmp(versionData.hwVersion, Version::M300, 5) == 0)
  {
    return true;
  }
  return false;
}


void
Vehicle::initCMD_SetSupportMatrix()
{
  cmd_setSupportMatrix[0].cmdSet    = OpenProtocolCMD::CMDSet::activation;
  cmd_setSupportMatrix[0].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[1].cmdSet    = OpenProtocolCMD::CMDSet::control;
  cmd_setSupportMatrix[1].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[2].cmdSet    = OpenProtocolCMD::CMDSet::broadcast;
  cmd_setSupportMatrix[2].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[3].cmdSet    = OpenProtocolCMD::CMDSet::mission;
  cmd_setSupportMatrix[3].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[4].cmdSet    = OpenProtocolCMD::CMDSet::hardwareSync;
  cmd_setSupportMatrix[4].fwVersion = extendedVersionBase;

  // Not supported in extendedVersionBase
  cmd_setSupportMatrix[5].cmdSet    = OpenProtocolCMD::CMDSet::virtualRC;
  cmd_setSupportMatrix[5].fwVersion = mandatoryVersionBase;

  cmd_setSupportMatrix[7].cmdSet    = OpenProtocolCMD::CMDSet::mfio;
  cmd_setSupportMatrix[7].fwVersion = extendedVersionBase;

  cmd_setSupportMatrix[8].cmdSet    = OpenProtocolCMD::CMDSet::subscribe;
  cmd_setSupportMatrix[8].fwVersion = extendedVersionBase;

  cmd_setSupportMatrix[9].cmdSet    = V1ProtocolCMD::CMDSet::hms;
  cmd_setSupportMatrix[9].fwVersion = mandatoryVersionBase;
}

bool
Vehicle::isCmdSetSupported(const uint8_t cmdSet)
{
  for (int i = 0; i < NUM_CMD_SET; i++)
  {
    if (cmd_setSupportMatrix[i].cmdSet == cmdSet)
    {
      if (cmdSet == OpenProtocolCMD::CMDSet::virtualRC &&
          versionData.fwVersion != Version::M100_31)
      {
        return false;
      }
      else if (versionData.fwVersion == Version::M100_31)
      {
        // CMDs not supported in Matrice 100
        if (cmdSet == OpenProtocolCMD::CMDSet::hardwareSync ||
            cmdSet == OpenProtocolCMD::CMDSet::mfio ||
            cmdSet == OpenProtocolCMD::CMDSet::subscribe)
        {
          return false;
        }
      }
      else if (isLegacyM600())
      {
        // CMDs not supported in Matrice 600 old firmware
        if (cmdSet == OpenProtocolCMD::CMDSet::hardwareSync ||
            cmdSet == OpenProtocolCMD::CMDSet::mfio ||
            cmdSet == OpenProtocolCMD::CMDSet::subscribe)
        {
          return false;
        }
      }
    }
  }
  return true;
}

void
Vehicle::sendBuriedDataPkgToFC(void)
{
    char sdk_version[MAX_OSDK_VERSION_SIZE];
    sprintf(sdk_version,"OSDK%d.%d.%d beta",DJIOSDK_MAJOR_VERSION, DJIOSDK_MINOR_VERSION, DJIOSDK_PATCH_VERSION);
    DataBuryPack data = {" ", DJIOSDK_IS_DEBUG, DJIOSDK_HARDWARE_TYPE, DJIOSDK_OPERATOR_TYPE};
    memcpy(data.sdk_version , sdk_version, sizeof(data.sdk_version) / sizeof(char));
#ifdef STM32
    data.hardware_type = STM32_HARDWARE_TYPE;
    data.operator_type = RTOS_OPERATOR_TYPE;
#endif
    legacyLinker->send(OpenProtocolCMD::CMDSet::Activation::dataBury, (uint8_t *) &data, sizeof(DataBuryPack));
}
