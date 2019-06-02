/** @file dji_vehicle.cpp
 *  @version 3.3
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
#include <new>

using namespace DJI;
using namespace DJI::OSDK;

Vehicle::Vehicle(const char* device,
		 uint32_t baudRate,
		 bool threadSupport,
                 bool enableAdvancedSensing)
  : protocolLayer(NULL)
  , subscribe(NULL)
  , broadcast(NULL)
  , control(NULL)
  , camera(NULL)
#ifdef ADVANCED_SENSING
  , advancedSensing(NULL)
  , advSensingErrorPrintOnce(false)
#endif
  , gimbal(NULL)
  , mfio(NULL)
  , moc(NULL)
  , mobileDevice(NULL)
  , missionManager(NULL)
  , hardSync(NULL)
  , virtualRC(NULL)
  , UARTSerialReadThread(NULL)
  , callbackThread(NULL)
  , advancedSensingEnabled(enableAdvancedSensing)
  , USBReadThread(NULL)
  , USBThreadReady(false)
  , payloadDevice(NULL)
{
  if (!device)
  {
    DERROR("Illegal serial device handle!\n");
  }

  this->threadSupported = threadSupport;
  this->device          = device;
  this->baudRate        = baudRate;
  callbackId            = 0;
  ackErrorCode.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  mandatorySetUp();
}

Vehicle::Vehicle(bool threadSupport)
  : protocolLayer(NULL)
  , subscribe(NULL)
  , broadcast(NULL)
  , control(NULL)
  , camera(NULL)
  , mfio(NULL)
  , moc(NULL)
  , mobileDevice(NULL)
  , missionManager(NULL)
  , hardSync(NULL)
  , virtualRC(NULL)
  , UARTSerialReadThread(NULL)
  , callbackThread(NULL)
  , payloadDevice(NULL)
{
  this->threadSupported = threadSupport;
  callbackId            = 0;

  mandatorySetUp();
}

void
Vehicle::mandatorySetUp()
{
  /*
   * Initialize buffers for threaded callbacks
   */
  if (this->threadSupported)
  {
    // We only need a buffer of recvContainers if we are using threads
    this->nbCallbackRecvContainer = new RecvContainer[200];
    this->circularBuffer          = new CircularBuffer();
  }

  /*
   * @note Initialize predefined callbacks
   */
  initCallbacks();

  /*
   * @note Initialize CMD_SET support matrix to identify
   * CMD_SET availability for paritcular FW version
   */
  initCMD_SetSupportMatrix();

  /*
   * @note Initialize communication layer
   */
  if (!initOpenProtocol())
  {
    DERROR("Failed to initialize Protocol Layer!\n");
  }

  /*
   * @note Initialize read thread
   */
  if (!initMainReadThread())
  {
    DERROR("Failed to initialize main read thread!\n");
  }
}

bool
Vehicle::GimbalSetUp()
{
  if (this->gimbal == 0)
  {
    initGimbal();
  }
  return ((this->gimbal == 0) ? false : true);
}

int
Vehicle::functionalSetUp()
{
  if (!initVersion())
  {
    return 1;
  }
  else if (this->getFwVersion() < extendedVersionBase &&
           this->getFwVersion() != Version::M100_31 && !(this->isLegacyM600()))
  {
    DERROR("Upgrade firmware using Assistant software!\n");
    return 1;
  }

  if(!initFullPlatformSupport())
  {
    DERROR("Failed to initialize full platform support!\n");
    return 1;
  }

  /*
   * Initialize subscriber if supported
   */
  if (!initSubscriber())
  {
    DERROR("Failed to initialize subscriber!\n");
    return 1;
   }

  /*
   * Initialize broadcast if supported
   */
  if (!initBroadcast())
  {
    DERROR("Failed to initialize Broadcast!\n");
    return 1;
  }

  /*
   * @note Initialize Movement Control
   */
  if (!initControl())
  {
    DERROR("Failed to initialize Control!\n");
    return 1;
  }

  /*
   * @note Initialize external components
   * like Camera and MFIO
   */

  if (!initCamera())
  {
    DERROR("Failed to initialize Camera!\n");
    return 1;
  }

  /*
   * Initialize MFIO if supported
   */
  if (!initMFIO())
  {
    DERROR("Failed to initialize MFIO!\n");
    return 1;
  }

  /*
   * Initialize Mobile-Onboard Communication (MobileCommunication)
   */
  if (!initMOC())
  {
    DERROR("Failed to initialize MobileCommunication!\n");
    return 1;
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


    if (!initMissionManager())
  {
    DERROR("Failed to initialize Mission Manager!\n");
    return 1;
  }

  if (!initHardSync())
  {
    DERROR("Failed to initialize HardSync!\n");
    return 1;
  }

  if (!initVirtualRC())
  {
    DERROR("Failed to initiaze VirtualRC!\n");
    return 1;
  }

#ifdef ADVANCED_SENSING
  if (advancedSensingEnabled)
  {
    if (!initAdvancedSensing())
    {
      DERROR("Failed to initialize AdvancedSensing!\n");
      return 1;
    }
    else
    {
      if (this->advancedSensing->getAdvancedSensingProtocol())
      {
        this->USBThreadReady = true;
      }
      else
      {
        this->USBThreadReady = false;
        return 1;
      }
    }
  }
#endif

  return 0;
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
}

void
Vehicle::callbackPoll()
{
  VehicleCallBackHandler cbVal;
  RecvContainer          recvCont;
  //! If Head = Tail, there is no data in the buffer, do not call cbPop.
  protocolLayer->getThreadHandle()->lockNonBlockCBAck();
  if (this->circularBuffer->head != this->circularBuffer->tail)
  {
    circularBuffer->cbPop(circularBuffer, &cbVal, &recvCont);
    protocolLayer->getThreadHandle()->freeNonBlockCBAck();
    cbVal.callback(this, recvCont, cbVal.userData);
  }
  else
  {
    protocolLayer->getThreadHandle()->freeNonBlockCBAck();
  }
}

Vehicle::~Vehicle()
{
  if (this->subscribe)
  {
    subscribe->reset(1);
  }
  if (threadSupported)
  {
    if (this->UARTSerialReadThread)
    {
      this->UARTSerialReadThread->stopThread();
      delete this->UARTSerialReadThread;
    }
    if (this->callbackThread)
    {
      this->callbackThread->stopThread();
      delete this->callbackThread;
    }
    if (this->USBReadThread)
    {
      this->USBReadThread->stopThread();
      delete this->USBReadThread;
    }
    delete[](nbCallbackRecvContainer);

    if (this->circularBuffer)
    {
      delete this->circularBuffer;
    }
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

  if (this->moc)
  {
    delete this->moc;
  }

  if (this->mobileDevice)
  {
    delete this->mobileDevice;
  }

  if (this->payloadDevice)
  {
    delete this->payloadDevice;
  }
  if (this->broadcast)
  {
    delete this->broadcast;
  }
  if (this->subscribe)
  {
    delete this->subscribe;
  }
#ifdef ADVANCED_SENSING
  if (this->advancedSensing)
    delete this->advancedSensing;
#endif
  if (this->hardSync)
  {
    delete this->hardSync;
  }
  if (this->missionManager)
  {
    delete this->missionManager;
  }
  if (this->protocolLayer)
  {
    delete this->protocolLayer;
  }
  if (this->platformManager)
  {
    delete this->platformManager;
  }

  this->USBThreadReady = false;
}

bool
Vehicle::initOpenProtocol()
{
  //Initialize platform manager before passing pointer to OpenProtocol constructor
  this->platformManager = new PlatformManager();

  this->protocolLayer = new (std::nothrow)
    OpenProtocol(this->platformManager, this->device, this->baudRate);
  if (this->protocolLayer == 0)
  {
    return false;
  }

  return true;
}

bool
Vehicle::initMainReadThread()
{
  if (threadSupported)
  {
    this->UARTSerialReadThread = platformManager->addThread(
      this, PlatformManager::UART_SERIAL_READ_THREAD);
    if (this->UARTSerialReadThread == NULL)
    {
      DERROR("Failed to initialize UART serial read thread!\n");
      return false;
    }
  }
  else
  {
    this->UARTSerialReadThread = NULL;
    return true;
  }

#if defined(STM32) || defined(__arm__) || defined(QT)
#elif defined(linux)
  DDEBUG("Set serial driver to nonblocking mode");
  dynamic_cast<DJI::OSDK::LinuxSerialDevice *>(this->protocolLayer->getDriver())->setSerialPureTimedRead();
#else
  DDEBUG("Serial driver is not supported in this platform yet!");
#endif
  return UARTSerialReadThread->createThread();
}

bool
Vehicle::initFullPlatformSupport()
{
  if(this->callbackThread)
  {
    DDEBUG("vehicle->callbackThread already initalized!");
    return true;
  }

  if (threadSupported)
  {
    this->callbackThread =
      platformManager->addThread(this, PlatformManager::CALLBACK_THREAD);
    if (this->callbackThread == NULL)
    {
      DERROR("Failed to initialize read callback thread!\n");
      return false;
    }
#ifdef ADVANCED_SENSING
    if (this->advancedSensingEnabled)
    {
      this->USBReadThread =
        platformManager->addThread(this, PlatformManager::USB_READ_THREAD);
      if (this->USBReadThread == NULL)
      {
        DERROR("Failed to initialize USB read thread!\n");
        return false;
      }
    }
#endif
  }
  else
  {
    this->callbackThread       = NULL;
    this->USBReadThread        = NULL;
    return true;
  }

  bool cbThreadStatus       = callbackThread->createThread();
  bool USBReadThreadStatus;

#ifdef ADVANCED_SENSING
  if (this->advancedSensingEnabled)
  {
    USBReadThreadStatus = USBReadThread->createThread();

    return (USBReadThreadStatus && cbThreadStatus);
  }
#endif

  return cbThreadStatus;
}

bool
Vehicle::initVersion()
{
#if STM32
  //! Non blocking call for STM32 as it does not support multi-thread
  getDroneVersion();
  this->platformManager->millisecSleep(2000);
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
    DSTATUS("Device Serial No. = %.16s\n", versionStruct.hw_serial_num);
  }
  DSTATUS("Hardware = %.12s\n", versionStruct.hwVersion);
  DSTATUS("Firmware = %d.%d.%d.%d\n", ver1, ver2, ver3, ver4);
  if (versionStruct.fwVersion < Version::FW(3, 2, 0, 0))
  {
    DSTATUS("Version CRC = 0x%X\n", versionStruct.version_crc);
  }

  versionData = versionStruct;
  return true;
}

void
Vehicle::initCallbacks()
{
  hotPointCallback.callback = 0;
  wayPointCallback.callback = 0;
  hotPointCallback.userData = 0;
  wayPointCallback.userData = 0;
  missionCallback.callback  = 0;
  missionCallback.userData  = 0;
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

    /*
     * Wait for 1.2 seconds, so we can detect all leftover
     * packages from unclean quit, and remove them properly
     */
    this->platformManager->millisecSleep(1200);
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
  }
  else
  {
    DSTATUS("Telemetry broadcast is not supported on this platform!\n");
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
  int pkgNumber = 0;
  ACK::ErrorCode ack;

  // Gimbal information via subscription
  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_STATUS>::type subscriptionGimbal;

  if (isLegacyM600())
  {
    this->gimbal = new (std::nothrow) Gimbal(this);
    if (this->gimbal == 0)
    {
      DERROR("Failed to allocate memory for Gimbal!\n");
      return false;
    }
    return true;
  }
  else if (this->getFwVersion() != Version::M100_31)
  {
    ack = this->subscribe->verify(wait_timeout);
    if (ACK::getError(ack))
    {
      DERROR("Failed to verify subscription!\n");
      return false;
    }

    Telemetry::TopicName topicList0[] = { Telemetry::TOPIC_GIMBAL_STATUS };
    int                  nTopic0 = sizeof(topicList0) / sizeof(topicList0[0]);

    bool result =
      this->subscribe->initPackageFromTopicList(pkgNumber, nTopic0, topicList0, 0, 50);
    if (result)
    {
      DSTATUS("Checking if gimbal is connected ...");
      ack = this->subscribe->startPackage(pkgNumber, wait_timeout);
      if (ACK::getError(ack))
      {
        DERROR("Failed to start subscription package!\n");
        return false;
      }
    }
    else
    {
      DERROR("Failed to initialize subscription package!\n");
      return false;
    }

    // Wait for telemetry data
    this->platformManager->millisecSleep(2000);

    subscriptionGimbal =
      this->subscribe->getValue<Telemetry::TOPIC_GIMBAL_STATUS>();

    ack = this->subscribe->removePackage(pkgNumber, wait_timeout);
    if (ACK::getError(ack))
    {
      DERROR("Failed to unsubscribe package!\n");
      return false;
    }

    this->platformManager->millisecSleep(2000);
  }

  if ((this->getFwVersion() != Version::M100_31 &&
       subscriptionGimbal.mountStatus == GIMBAL_MOUNTED) ||
      this->getFwVersion() == Version::M100_31)
  {
    this->gimbal = new (std::nothrow) Gimbal(this);
    if (this->gimbal == 0)
    {
      DERROR("Failed to allocate memory for Gimbal!\n");
      return false;
    }
    return true;
  }
  else
  {
    DSTATUS("Gimbal not mounted!\n");
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

  return true;
}
#endif

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
Vehicle::initMOC()
{
  if(this->moc)
  {
    DDEBUG("vehicle->moc already initalized!");
    return true;
  }

  moc = new (std::nothrow) MobileCommunication(this);
  if (this->moc == 0)
  {
    DERROR("Failed to allocate memory for MobileCommunication!\n");
    return false;
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

bool
Vehicle::initVirtualRC()
{
  if(this->virtualRC)
  {
    DDEBUG("vehicle->virtualRC already initalized!");
    return true;
  }

  if (isCmdSetSupported(OpenProtocolCMD::CMDSet::virtualRC))
  {
    virtualRC = new (std::nothrow) VirtualRC(this);
    if (this->virtualRC == 0)
    {
      DERROR("Error creating Virtual RC!");
      return false;
    }
  }
  else
  {
    DERROR("Virtual RC is not supported on this platform!\n");
  }

  return true;
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
Vehicle::processReceivedData(RecvContainer* receivedFrame)
{
  receivedFrame->recvInfo.version = this->getFwVersion();
  if (receivedFrame->dispatchInfo.isAck)
  {
    // TODO Fill up ACKErrorCode Container
    if (receivedFrame->dispatchInfo.isCallback)
    {
      this->nbVehicleCallBackHandler.callback =
        (VehicleCallBack) this
          ->nbCallbackFunctions[receivedFrame->dispatchInfo.callbackID];
      this->nbVehicleCallBackHandler.userData =
        this->nbUserData[receivedFrame->dispatchInfo.callbackID];

      if (nbVehicleCallBackHandler.callback)
      {
        if (threadSupported)
        {
          this->nbCallbackRecvContainer[receivedFrame->dispatchInfo.callbackID] =
            *receivedFrame;
          protocolLayer->getThreadHandle()->lockNonBlockCBAck();
          this->circularBuffer->cbPush(
            this->circularBuffer, this->nbVehicleCallBackHandler,
            this->nbCallbackRecvContainer[receivedFrame->dispatchInfo.callbackID]);
          protocolLayer->getThreadHandle()->freeNonBlockCBAck();
        }
        else
        {
          this->nbVehicleCallBackHandler.callback(
            this, *receivedFrame, this->nbVehicleCallBackHandler.userData);
        }
      }
    }
    else
    {
      DDEBUG("Dispatcher identified as blocking call\n");
      // TODO remove
      this->lastReceivedFrame = *receivedFrame;

      ACKHandler(static_cast<void*>(receivedFrame));
      protocolLayer->getThreadHandle()->notify();
    }
  }
  else
  {
    DDEBUG("Dispatcher identified as push data\n");
    PushDataHandler(static_cast<void*>(receivedFrame));
  }
}

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

int
Vehicle::callbackIdIndex()
{
  if (callbackId == 199)
  {
    callbackId = 0;
    return 0;
  }
  else
  {
    callbackId++;
    return callbackId;
  }
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
  int cbIndex = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    nbCallbackFunctions[cbIndex] = (void*)activateCallback;
    nbUserData[cbIndex]          = NULL;
  }
  protocolLayer->send(
    2, 0, OpenProtocolCMD::CMDSet::Activation::activate, (uint8_t*)&accountData,
    sizeof(accountData) - sizeof(char*), 1000, 3, true, cbIndex);
}

ACK::ErrorCode
Vehicle::activate(ActivateData* data, int timeout)
{
  ACK::ErrorCode ack;

  if(this->functionalSetUp() != 0)
  {
    DERROR("Unable to initialize some vehicle components!");
    ack.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
    return ack;
  }

  data->version        = versionData.fwVersion;
  accountData          = *data;
  accountData.reserved = 2;

  for (int i             = 0; i < 32; ++i)
    accountData.iosID[i] = '0'; //! @note for ios verification
  DSTATUS("version 0x%X\n", versionData.fwVersion);
  DDEBUG("%.32s", accountData.iosID);
  //! Using function prototype II of send
  protocolLayer->send(2, this->getEncryption(), OpenProtocolCMD::CMDSet::Activation::activate,
                      (uint8_t*)&accountData,
                      sizeof(accountData) - sizeof(char*), 1000, 3, false, 0);

  ack = *(ACK::ErrorCode*)waitForACK(
    OpenProtocolCMD::CMDSet::Activation::activate, timeout);

  if (ack.data == OpenProtocolCMD::ErrorCode::ActivationACK::SUCCESS &&
      accountData.encKey)
  {
    DSTATUS("Activation successful\n");
    protocolLayer->setKey(accountData.encKey);
    setActivationStatus(true);

    if(!this->gimbal)
    {
      initGimbal();
    }
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
  int      cbIndex     = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    nbCallbackFunctions[cbIndex] = (void*)getDroneVersionCallback;
    nbUserData[cbIndex]          = NULL;
  }

  // When UserData is implemented, pass the Vehicle as userData.
  protocolLayer->send(2, 0, OpenProtocolCMD::CMDSet::Activation::getVersion,
                      (uint8_t*)&cmd_data, 1, cmd_timeout, retry_time, true,
                      cbIndex);
}

ACK::DroneVersion
Vehicle::getDroneVersion(int timeout)
{
  versionData.version_ack =
    OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  versionData.version_crc     = 0x0;
  versionData.version_name[0] = 0;

  uint32_t cmd_timeout = 100; // unit is ms
  uint32_t retry_time  = 3;
  uint8_t  cmd_data    = 0;

  protocolLayer->send(2, 0, OpenProtocolCMD::CMDSet::Activation::getVersion,
                      (uint8_t*)&cmd_data, 1, cmd_timeout, retry_time, false,
                      0);

  // Wait for drone version data
  uint8_t* rawACK = (uint8_t*)waitForACK(
    OpenProtocolCMD::CMDSet::Activation::getVersion, timeout);

  // Parse received data
  if (!parseDroneVersionInfo(this->versionData, rawACK))
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
    vehiclePtr->protocolLayer->setKey(vehiclePtr->accountData.encKey);
    vehiclePtr->setActivationStatus(true);
  }
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
  }
}

void
Vehicle::controlAuthorityCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                                  UserData userData)
{
  ACK::ErrorCode ack;
  ack.data = OpenProtocolCMD::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  uint8_t data    = 0x1;
  int     cbIndex = vehiclePtr->callbackIdIndex();

  if (recvFrame.recvInfo.len - OpenProtocol::PackageMin <= sizeof(uint16_t))
  {
    ack.data = recvFrame.recvData.ack;
    ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  if (ack.data == OpenProtocolCMD::ErrorCode::ControlACK::SetControl::
                    OBTAIN_CONTROL_IN_PROGRESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    vehiclePtr->obtainCtrlAuthority(controlAuthorityCallback);
  }
  else if (ack.data == OpenProtocolCMD::ErrorCode::ControlACK::SetControl::
                         RELEASE_CONTROL_IN_PROGRESS)
  {
    ACK::getErrorCodeMessage(ack, __func__);
    vehiclePtr->releaseCtrlAuthority(controlAuthorityCallback);
  }
  else
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }
}

/*****************************Set State
 * Data**************************************/

void
Vehicle::setVersion(const Version::FirmWare& value)
{
  versionData.fwVersion = value;
}

void
Vehicle::ACKHandler(void* eventData)
{
  if (!eventData)
  {
    DERROR("Invalid ACK event data received!\n");
    return;
  }

  RecvContainer* ackData = (RecvContainer*)eventData;
  const uint8_t cmd[] = { ackData->recvInfo.cmd_set, ackData->recvInfo.cmd_id };

  if (ackData->recvInfo.cmd_set == OpenProtocolCMD::CMDSet::mission)
  {
    if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::waypointAddPoint,
               sizeof(cmd)) == 0)
    {
      waypointAddPointACK.ack.info = ackData->recvInfo;
      waypointAddPointACK.ack.data = ackData->recvData.wpAddPointACK.ack;
      waypointAddPointACK.index    = ackData->recvData.wpAddPointACK.index;
    }
    else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::waypointDownload,
               sizeof(cmd)) == 0)
    {
      waypointInitACK.ack.info = ackData->recvInfo;
      waypointInitACK.ack.data = ackData->recvData.wpInitACK.ack;
      waypointInitACK.data     = ackData->recvData.wpInitACK.data;
    }
    else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::waypointIndexDownload,
               sizeof(cmd)) == 0)
    {
      waypointIndexACK.ack.info = ackData->recvInfo;
      waypointIndexACK.ack.data = ackData->recvData.wpIndexACK.ack;
      waypointIndexACK.data     = ackData->recvData.wpIndexACK.data;
    }
    else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::hotpointStart,
                    sizeof(cmd)) == 0)
    {
      hotpointStartACK.ack.info  = ackData->recvInfo;
      hotpointStartACK.ack.data  = ackData->recvData.hpStartACK.ack;
      hotpointStartACK.maxRadius = ackData->recvData.hpStartACK.maxRadius;
    }
    else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::hotpointDownload,
                    sizeof(cmd)) == 0)
    {
      hotpointReadACK.ack.info = ackData->recvInfo;
      hotpointReadACK.ack.data = ackData->recvData.hpReadACK.ack;
      hotpointReadACK.data     = ackData->recvData.hpReadACK.data;
    }
    else if (cmd[0] == OpenProtocolCMD::CMDSet::mission
             && OpenProtocolCMD::CMDSet::Mission::waypointInitV2[1] <= cmd[1]
                &&  cmd[1] <= OpenProtocolCMD::CMDSet::Mission::waypointGetMinMaxActionIDV2[1])
    {
      wayPoint2CommonRspACK.info      = ackData->recvInfo;
      wayPoint2CommonRspACK.info.buf  = ackData->recvData.raw_ack_array;
      wayPoint2CommonRspACK.updated   = true;
    }
    else
    {
      ackErrorCode.info = ackData->recvInfo;
      ackErrorCode.data = ackData->recvData.missionACK;
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Activation::getVersion,
                  sizeof(cmd)) == 0)
  {
    size_t arrLength = sizeof(ackData->recvData.versionACK);
    for (int i = 0; i < arrLength; i++)
    {
      //! Interim stage: version data will be parsed before returned to user
      this->rawVersionACK[i] = ackData->recvData.versionACK[i];
    }
    droneVersionACK.ack.info = ackData->recvInfo;
  }
  else if (ackData->recvInfo.cmd_set == OpenProtocolCMD::CMDSet::subscribe)
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.subscribeACK;
  }
  else if (ackData->recvInfo.cmd_set == OpenProtocolCMD::CMDSet::control)
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.commandACK;
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::MFIO::init, sizeof(cmd)) == 0)
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.mfioACK;
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::MFIO::get, sizeof(cmd)) == 0)
  {
    mfioGetACK.ack.info = ackData->recvInfo;
    mfioGetACK.ack.data = ackData->recvData.mfioGetACK.result;
    mfioGetACK.value    = ackData->recvData.mfioGetACK.value;
  }
  else
  {
    ackErrorCode.info = ackData->recvInfo;
    ackErrorCode.data = ackData->recvData.ack;
  }
}

void
Vehicle::PushDataHandler(void* eventData)
{
  RecvContainer* pushDataEntry = (RecvContainer*)eventData;

  const uint8_t cmd[] = { pushDataEntry->recvInfo.cmd_set,
                          pushDataEntry->recvInfo.cmd_id };

  if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::broadcast, sizeof(cmd)) ==
      0)
  {
    if (broadcast)
    {
      if (broadcast->unpackHandler.callback)
      {
        broadcast->unpackHandler.callback(this, *(pushDataEntry),
                                          broadcast->unpackHandler.userData);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::subscribe,
                  sizeof(cmd)) == 0)
  {
    if (subscribe)
    {
      DDEBUG("Decode callback subscribe");
      if (subscribe->subscriptionDataDecodeHandler.callback)
      {
        subscribe->subscriptionDataDecodeHandler.callback(
          this, *(pushDataEntry),
          subscribe->subscriptionDataDecodeHandler.userData);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::HardwareSync::ppsNMEAGPSGSA,
                  sizeof(cmd)) == 0
           || memcmp(cmd, OpenProtocolCMD::CMDSet::HardwareSync::ppsNMEAGPSRMC,
                     sizeof(cmd)) == 0
           || memcmp(cmd, OpenProtocolCMD::CMDSet::HardwareSync::ppsNMEARTKGSA,
                     sizeof(cmd)) == 0
           || memcmp(cmd, OpenProtocolCMD::CMDSet::HardwareSync::ppsNMEARTKRMC,
                     sizeof(cmd)) == 0)
  {
    if(hardSync)
    {
      if(hardSync->ppsNMEAHandler.callback)
      {
        hardSync->ppsNMEAHandler.callback(
          this, *(pushDataEntry),
          hardSync->ppsNMEAHandler.userData);
      }
      else  // If user listen to it already, we don't store them.
      {
        hardSync->writeData(cmd[1], pushDataEntry);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::HardwareSync::ppsUTCTime,
                  sizeof(cmd)) == 0)
  {
    if(hardSync)
    {
      if(hardSync->ppsUTCTimeHandler.callback)
      {
        hardSync->ppsUTCTimeHandler.callback(
          this, *(pushDataEntry),
          hardSync->ppsUTCTimeHandler.userData);
      }
      else  // If user listen to it already, we don't store them.
      {
        hardSync->writeData(cmd[1], pushDataEntry);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::HardwareSync::ppsUTCFCTimeRef,
                  sizeof(cmd)) == 0)
  {
    if(hardSync)
    {
      if(hardSync->ppsUTCFCTimeHandler.callback)
      {
        hardSync->ppsUTCFCTimeHandler.callback(
          this, *(pushDataEntry),
          hardSync->ppsUTCFCTimeHandler.userData);
      }
      else  // If user listen to it already, we don't store them.
      {
        hardSync->writeData(cmd[1], pushDataEntry);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::HardwareSync::ppsSource,
                  sizeof(cmd)) == 0)
  {
    if(hardSync)
    {
      if(hardSync->ppsSourceHandler.callback)
      {
        hardSync->ppsSourceHandler.callback(
          this, *(pushDataEntry),
          hardSync->ppsSourceHandler.userData);
      }
      else  // If user listen to it already, we don't store them.
      {
        hardSync->writeData(cmd[1], pushDataEntry);
      }
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::fromMobile,
                  sizeof(cmd)) == 0)
  {
    if (moc)
    {
      if (moc->fromMSDKHandler.callback)
      {
        if(threadSupported)
        {
          DDEBUG("Received data from mobile\n");
          protocolLayer->getThreadHandle()->lockNonBlockCBAck();
          this->circularBuffer->cbPush(
              this->circularBuffer, moc->fromMSDKHandler,
              *pushDataEntry);
          protocolLayer->getThreadHandle()->freeNonBlockCBAck();
        }
        else
        {
          moc->fromMSDKHandler.callback(this, *(pushDataEntry),moc->fromMSDKHandler.userData);
        }
      }
    }

    if (mobileDevice) {
      if (mobileDevice->fromMSDKHandler.callback) {
        if (threadSupported) {
          DDEBUG("Received data from mobile\n");
          protocolLayer->getThreadHandle()->lockNonBlockCBAck();
          this->circularBuffer->cbPush(this->circularBuffer, mobileDevice->fromMSDKHandler,
                                       *pushDataEntry);
          protocolLayer->getThreadHandle()->freeNonBlockCBAck();
        } else {
          mobileDevice->fromMSDKHandler.callback(this, *(pushDataEntry), mobileDevice->fromMSDKHandler.userData);
        }
      }
    }

  }

  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::fromPayload,
                  sizeof(cmd)) == 0)
  {
    if (payloadDevice) {
      if (payloadDevice->fromPSDKHandler.callback) {
        if (threadSupported) {
          DDEBUG("Received data from payload\n");
          protocolLayer->getThreadHandle()->lockNonBlockCBAck();
          this->circularBuffer->cbPush(this->circularBuffer, payloadDevice->fromPSDKHandler,
                                       *pushDataEntry);
          protocolLayer->getThreadHandle()->freeNonBlockCBAck();
        } else {
          payloadDevice->fromPSDKHandler.callback(this, *(pushDataEntry), payloadDevice->fromPSDKHandler.userData);
        }
      }
    }

  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::mission,
                  sizeof(cmd)) == 0)
  {
    if (missionManager)
    {
      if (missionCallback.callback)
      {
        missionCallback.callback(this, *(pushDataEntry),
                                 missionCallback.userData);
      }
      else
      {
        switch (pushDataEntry->recvData.missionACK)
        {
          case MISSION_MODE_A:
            break;
          case MISSION_WAYPOINT:
            if (missionManager->wpMission)
            {
              if (wayPointData)
              {
                if (missionManager->wpMission->wayPointCallback.callback)
                  missionManager->wpMission->wayPointCallback.callback(
                    this, *(pushDataEntry),
                    missionManager->wpMission->wayPointCallback.userData);
                else
                  DDEBUG("Mode WayPoint\n");
              }
            }
            break;
          case MISSION_HOTPOINT:
            if (missionManager->hpMission)
            {
              if (hotPointData)
              {
                if (missionManager->hpMission->hotPointCallback.callback)
                  missionManager->hpMission->hotPointCallback.callback(
                    this, *(pushDataEntry),
                    missionManager->hpMission->hotPointCallback.userData);
                else
                  DDEBUG("Mode HotPoint\n");
              }
            }
            break;
          case MISSION_IOC:
            //! @todo compare IOC with other mission modes comprehensively
            DDEBUG("Mode IOC \n");
            break;
          default:
            DERROR("Unknown mission code 0x%X \n", pushDataEntry->recvData.ack);
            break;
        }
      }
    }
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Broadcast::waypoint,
                  sizeof(cmd)) == 0)
  {
    if (missionManager->wpMission)
    {
      //! @todo add waypoint session decode
      if (missionManager->wpMission->wayPointEventCallback.callback)
      {
        missionManager->wpMission->wayPointEventCallback.callback(
          this, *(pushDataEntry),
          missionManager->wpMission->wayPointEventCallback.userData);
      }
      else
      {
        DDEBUG("WayPoint DATA");
      }
    }
  }
#ifdef WAYPT2_CORE
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::waypointGetStatePushDataV2,
                  sizeof(cmd)) == 0)
  {
    if (missionManager && missionManager->wpMission)
    {
      missionManager->wpMission->updateV2PushData(cmd[1],
                                                  pushDataEntry->recvInfo.seqNumber,
                                                  pushDataEntry->recvData.raw_ack_array,
                                                  pushDataEntry->recvInfo.len-OpenProtocol::PackageMin);
    }
  }
#endif
  else
  {
    DDEBUG("Received Unknown PushData\n");
  }
}

void*
Vehicle::waitForACK(const uint8_t (&cmd)[OpenProtocolCMD::MAX_CMD_ARRAY_SIZE],
                    int timeout)
{
  void* pACK;

  protocolLayer->getThreadHandle()->lockACK();
  protocolLayer->getThreadHandle()->wait(timeout);

  if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::waypointAddPoint,
             sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->waypointAddPointACK);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::waypointDownload,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->waypointInitACK);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::waypointIndexDownload,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->waypointIndexACK);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::hotpointStart,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->hotpointStartACK);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Mission::hotpointDownload,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->hotpointReadACK);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::Activation::getVersion,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->rawVersionACK);
  }
  else if (memcmp(cmd, OpenProtocolCMD::CMDSet::MFIO::get,
                  sizeof(cmd)) == 0)
  {
    pACK = static_cast<void*>(&this->mfioGetACK);
  }
  else if (cmd[0] == OpenProtocolCMD::CMDSet::mission
           && 0x40 <= cmd[1] && cmd[1] <= 0x53)
  {
    pACK = static_cast<void*>(&this->wayPoint2CommonRspACK);
  }
  else
  {
    pACK = static_cast<void*>(&this->ackErrorCode);
  }

  protocolLayer->getThreadHandle()->freeACK();

  return pACK;
}

void
Vehicle::obtainCtrlAuthority(VehicleCallBack callback, UserData userData)
{
  uint8_t data    = 1;
  int     cbIndex = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    nbCallbackFunctions[cbIndex] = (void*)controlAuthorityCallback;
    nbUserData[cbIndex]          = NULL;
  }
  protocolLayer->send(2, this->encrypt,
                      OpenProtocolCMD::CMDSet::Control::setControl, &data, 1,
                      500, 2, true, cbIndex);
}

ACK::ErrorCode
Vehicle::obtainCtrlAuthority(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        data = 1;

  protocolLayer->send(2, this->encrypt,
                      OpenProtocolCMD::CMDSet::Control::setControl, &data, 1,
                      500, 2, false, 0);

  ack = *(ACK::ErrorCode*)waitForACK(
    OpenProtocolCMD::CMDSet::Control::setControl, timeout);

  if (ack.data == OpenProtocolCMD::ErrorCode::ControlACK::SetControl::
  OBTAIN_CONTROL_IN_PROGRESS)
  {
    ack = this->obtainCtrlAuthority(timeout);
  }

  return ack;
}

void
Vehicle::releaseCtrlAuthority(VehicleCallBack callback, UserData userData)
{
  uint8_t data    = 0;
  int     cbIndex = callbackIdIndex();
  if (callback)
  {
    nbCallbackFunctions[cbIndex] = (void*)callback;
    nbUserData[cbIndex]          = userData;
  }
  else
  {
    // nbCallbackFunctions[cbIndex] = (void*)ReleaseCtrlCallback;
    nbUserData[cbIndex] = NULL;
  }
  protocolLayer->send(2, this->encrypt,
                      OpenProtocolCMD::CMDSet::Control::setControl, &data, 1,
                      500, 2, true, cbIndex);
}

ACK::ErrorCode
Vehicle::releaseCtrlAuthority(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        data = 0;

  protocolLayer->send(2, this->encrypt,
                      OpenProtocolCMD::CMDSet::Control::setControl, &data, 1,
                      500, 2, false, 1);

  ack = *(ACK::ErrorCode*)waitForACK(
    OpenProtocolCMD::CMDSet::Control::setControl, timeout);

  if (ack.data == OpenProtocolCMD::ErrorCode::ControlACK::SetControl::
                    RELEASE_CONTROL_IN_PROGRESS)
  {
    ack = this->releaseCtrlAuthority(timeout);
  }

  return ack;
}

void
Vehicle::setLastReceivedFrame(RecvContainer recvFrame)
{
  protocolLayer->getThreadHandle()->lockFrame();
  this->lastReceivedFrame = recvFrame;
  protocolLayer->getThreadHandle()->freeFrame();
}

RecvContainer
Vehicle::getLastReceivedFrame()
{
  RecvContainer recvFrame;
  protocolLayer->getThreadHandle()->lockFrame();
  recvFrame = this->lastReceivedFrame;
  protocolLayer->getThreadHandle()->freeFrame();
  return recvFrame;
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

PlatformManager*
Vehicle::getPlatformManager() const
{
  return this->platformManager;
}

Thread*
Vehicle::getSerialReadThread() const
{
  return this->UARTSerialReadThread;
}

Thread*
Vehicle::getCallbackThread() const
{
  return this->callbackThread;
}

Thread*
Vehicle::getUSBReadThread() const
{
  return this->USBReadThread;
}

bool
Vehicle::isUSBThreadReady()
{
  return this->USBThreadReady;
}

uint8_t*
Vehicle::getRawVersionAck()
{
  return this->rawVersionACK;
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