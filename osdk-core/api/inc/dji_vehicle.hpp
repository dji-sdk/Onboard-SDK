/** @file dji_vehicle.hpp
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

#ifndef OSDK_CORE_INC_DJI_VEHICLE_H_
#define OSDK_CORE_INC_DJI_VEHICLE_H_

#include <cstdint>

#include "dji_broadcast.hpp"
#include "dji_camera.hpp"
#include "dji_circular_buffer.hpp"
#include "dji_command.hpp"
#include "dji_control.hpp"
#include "dji_gimbal.hpp"
#include "dji_hard_driver.hpp"
#include "dji_hardware_sync.hpp"
#include "dji_mfio.hpp"
#include "dji_mission_manager.hpp"
#include "dji_mobile_communication.hpp"
#include "dji_open_protocol.hpp"
#include "dji_platform_manager.hpp"
#include "dji_status.hpp"
#include "dji_subscription.hpp"
#include "dji_thread_manager.hpp"
#include "dji_type.hpp"
#include "dji_vehicle_callback.hpp"
#include "dji_version.hpp"
#include "dji_virtual_rc.hpp"
#ifdef ADVANCED_SENSING
#include "dji_advanced_sensing.hpp"
#endif

/*! Platform includes:
 *  This set of macros figures out which files to include based on your
 * platform.
 */

#ifdef QT
#include <qt_thread.hpp>
#elif STM32
#include <STM32F4DataGuard.h>
#elif defined(__linux__)
#include "posix_thread.hpp"
#endif

namespace DJI
{
namespace OSDK
{

static int callbackId;

/*! @brief A top-level encapsulation of a DJI drone/FC connected to your OES.
 *
 * @details This class instantiates objects for all features your drone/FC
 * supports.
 * Create a Vechile object in your code and you will have access to the entire
 * DJI OSDK API.
 *
 */
class Vehicle
{
public:
#pragma pack(1)
  typedef struct ActivateData
  {
    uint32_t ID;
    uint32_t reserved;
    uint32_t version;
    uint8_t  iosID[32]; //! @note useless
    char*    encKey;
  } ActivateData; // pack(1)
#pragma pack()

public:
  Vehicle(const char* device,
	  uint32_t baudRate,
          bool threadSupport,
          bool enableAdvancedSensing = false);
  Vehicle(bool threadSupport);
  ~Vehicle();

  OpenProtocol*        protocolLayer;
  DataSubscription*    subscribe;
  DataBroadcast*       broadcast;
  Control*             control;
  Camera*              camera;
  Gimbal*              gimbal;
  MFIO*                mfio;
  MobileCommunication* moc;
  MissionManager*      missionManager;
  HardwareSync*        hardSync;
  // Supported only on Matrice 100
  VirtualRC* virtualRC;
#ifdef ADVANCED_SENSING
  AdvancedSensing* advancedSensing;
#endif

  ////// Control authorities //////

  /*! @brief
  *
  *  Obtain the control authority of the api (non-blocking call)
  *
  *  @param callback callback function
  *  @param userData user data (void ptr)
  */
  void obtainCtrlAuthority(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
  *
  *  Obtain the control authority of the api (blocking call)
  *
  *  @param timeout time to wait for ACK
  */
  ACK::ErrorCode obtainCtrlAuthority(int timeout);
  /*! @brief
  *
  *  Release the control authority of the api (non-blocking call)
  *
  *  @param callback callback function
  *  @param userData user data (void ptr)
  */
  void releaseCtrlAuthority(VehicleCallBack callback = 0,
                            UserData        userData = 0);
  /*! @brief
  *
  *  Release the control authority of the api (blocking call)
  *
  *  @param timeout time to wait for ACK
  */
  ACK::ErrorCode releaseCtrlAuthority(int timeout);

  ////// Callback Handler setters //////

  ////////// Blocking calls ///////////

  /**
  * @remark
  * Blocks until ACK frame arrives or timeout occurs
  *
  * @brief
  * Send activation control to your flight controller to check if: \n a)
  * your application registered in your developer
  * account \n b) API Control enabled in the Assistant software\n\n
  * Proceed to programming if activation successful.
  *
  * @return ACK from flight controller
  *
  * @todo
  * Implement high resolution timer to catch ACK timeout
  */
  ACK::ErrorCode activate(ActivateData* data, int timeout);
  /**
   * @brief
   * Send get version control to the vehicle.
   *
   * @return type ACK::DroneVersion containing:
   * ACKErrorCode: data (ack value)
   * VersionData:  hardware version
   * VersionData:  firmware version
   * VersionData:  hardware serial number
   * VersionData:  CRC
   * VersionData:  version name
   */
  ACK::DroneVersion getDroneVersion(int timeout);

  ////////// Callback calls //////////

  /**
   * @brief
   * Send activation request to your flight controller
   * to check if: \n a) your application registered in your developer
   * account \n b) API Control enabled in the Assistant software\n\n
   * Proceed to programming if activation successful.
   */
  void activate(ActivateData* data, VehicleCallBack callback = 0,
                UserData userData = 0);

  //@{
  /**
   * Get aircraft version.
   *
   * @note
   * You can query your flight controller prior to activation.
   */
  void getDroneVersion(VehicleCallBack callback = 0, UserData userData = 0);

  //////////// Getters/Setters //////////

  /**
   * Get Activation information
   */
  ActivateData getAccountData() const;

  /*
   * Activation Control
   */
  void setAccountData(const ActivateData& value);

  /**
   * Set SDK version.
   */
  void setVersion(const Version::FirmWare& value);

  Version::FirmWare getFwVersion() const;
  char*             getHwVersion() const;
  char*             getHwSerialNum() const;
  bool              isLegacyM600();
  bool              isM100();

  void setKey(const char* key);
  CircularBuffer* circularBuffer; //! @note not used yet

  /**
   * Storage for last received packet: accessors
   */
  void setLastReceivedFrame(RecvContainer recvFrame);
  RecvContainer getLastReceivedFrame();
  //! @brief Wait for ACK frame to arrive
  void* waitForACK(const uint8_t (&cmd)[OpenProtocolCMD::MAX_CMD_ARRAY_SIZE],
                   int timeout);

  ///////////// Interact with Protocol ///////////

  /*! @brief This function takes a frame and calls the right handlers/functions
   * based
   *         on the nature of the frame (ack, blocking, etc.)
   * @param receivedFrame: RecvContainer populated by the protocolLayer
   * @return NULL
   */
  void processReceivedData(RecvContainer* receivedFrame);

#ifdef ADVANCED_SENSING
  /*! @brief This function takes a frame and calls the right handlers/functions
   *         based on the nature of the frame (ack, blocking, etc.)
   * @param receivedFrame: RecvContainer populated by the AdvancedSensing Protocol
   * @return NULL
   */
  void processAdvancedSensingImgs(RecvContainer* receivedFrame);

  bool advSensingErrorPrintOnce;
#endif

  //! User sets this to true in order to enable Callback thread with Non
  //! blocking calls.
  void     callbackPoll();
  int      callbackIdIndex();
  void*    nbCallbackFunctions[200]; //! @todo magic number
  UserData nbUserData[200];          //! @todo magic number

private:
  Version::VersionData versionData;
  ActivateData         accountData;

  //! Thread management
public:
  Thread* getSerialReadThread() const;
  Thread* getCallbackThread() const;
  Thread* getUSBReadThread() const;
  bool    isUSBThreadReady();

private:
  bool encrypt = false;
  Thread* UARTSerialReadThread;
  Thread* callbackThread;
  Thread* USBReadThread;
  bool    stopCond;
  bool    USBThreadReady;

  //! Initialization data
  bool        threadSupported;
  bool        advancedSensingEnabled;
  const char* device;
  uint32_t    baudRate;

  //! ACK management
  // Internal space
  uint8_t rawVersionACK[MAX_ACK_SIZE];

  // User space ACK types
  ACK::ErrorCode     ackErrorCode;
  ACK::DroneVersion  droneVersionACK;
  ACK::HotPointStart hotpointStartACK;
  ACK::HotPointRead  hotpointReadACK;
  /*!WayPoint download command
   * @note Download mission setting*/
  ACK::WayPointInit waypointInitACK;
  /*!WayPoint index download command ACK
   * @note Download index settings*/
  ACK::WayPointIndex waypointIndexACK;
  /*!WayPoint add point command ACK*/
  ACK::WayPointAddPoint waypointAddPointACK;
  ACK::MFIOGet          mfioGetACK;

public:
  uint8_t* getRawVersionAck();

private:
  //! This array will be populated by Non blocking calls depending on
  //! availability of array elements.
  //! Elements may be equal to NULL if Callback function execution has been
  //! completed and array element of
  //! callbackFunction is available to be populated.
  RecvContainer* nbCallbackRecvContainer;

  VehicleCallBackHandler nbVehicleCallBackHandler;

  //! Added for connecting protocolLayer to Vehicle
  RecvContainer lastReceivedFrame;

  /*
   * @brief Vehicle initialization components
   */
public:
  /*! @brief Initialize all functional Vehicle components
*  like, Subscription, Broadcast, Control, Gimbal, ect
*/
  int functionalSetUp();

private:
  /*! @brief Initialize minimal Vehicle components
*/
  void mandatorySetUp();

  /*! @brief Initialize the right platform-specific implementations
   *  @details
   *  @return false if error, true if success
   */
  bool initFullPlatformSupport();
  /*!
   * @brief Initialize main read thread to support UART communication
   * @return fasle if error, true if success
   */
  bool initMainReadThread();
  bool initOpenProtocol();
  void initCallbacks();
  void initCMD_SetSupportMatrix();
  bool initSubscriber();
  bool initBroadcast();
  bool initControl();
  bool initCamera();
  bool initGimbal();
  bool initMFIO();
  bool initMOC();
  bool initMissionManager();
  bool initHardSync();
  bool initVirtualRC();
#ifdef ADVANCED_SENSING
  bool initAdvancedSensing();
#endif

  //* Set of callback handler for various things
  VehicleCallBackHandler subscriberDecodeHandler;

  /*! @brief Check if given CMD_SET supported on your flight controller
   *  @return false if not supported, true if supported
   */
  bool isCmdSetSupported(const uint8_t cmdSet);

  bool initVersion();

  /*! @brief A callback function for activate non-blocking calls
   *  @param receivedFrame: RecvContainer populated by the protocolLayer
   *  @return NULL
   */
  static void activateCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                               UserData userData = 0);
  /*! @brief A callback function for get drone version non-blocking calls
   *  @param receivedFrame: RecvContainer populated by the protocolLayer
   *  @return NULL
   */
  static void getDroneVersionCallback(Vehicle*      vehiclePtr,
                                      RecvContainer recvFrame,
                                      UserData      userData = 0);
  /*! @brief A callback function for control authority non-blocking calls
   *  @param receivedFrame: RecvContainer populated by the protocolLayer
   *  @return NULL
   */
  static void controlAuthorityCallback(Vehicle*      vehiclePtr,
                                       RecvContainer recvFrame,
                                       UserData      userData);

  void ACKHandler(void* eventData);
  void PushDataHandler(void* eventData);

  /*
   * Used in PushData event handling
   */
public:
  bool hotPointData;
  bool wayPointData;

private:
  VehicleCallBackHandler hotPointCallback;
  VehicleCallBackHandler wayPointCallback;
  VehicleCallBackHandler missionCallback;

public:
  static bool parseDroneVersionInfo(Version::VersionData& versionData,
                                    uint8_t*              ackPtr);

private:
  const int            wait_timeout   = 5;
  const int            GIMBAL_MOUNTED = 1;
  static const uint8_t NUM_CMD_SET    = 9;
  CMD_SETSupportMatrix cmd_setSupportMatrix[NUM_CMD_SET];

  /*
   * Platform management
   */
public:
  PlatformManager* getPlatformManager() const;
  void setEncryption(bool encryptSetting);
  bool getEncryption();

private:
  PlatformManager* platformManager;
};
}
}
#endif /* OSDK_CORE_INC_DJI_VEHICLE_H_ */
