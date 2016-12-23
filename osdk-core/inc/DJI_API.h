/** @file DJI_API.h
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Core API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

// The comment block below is made for doxygen.

/*! @mainpage
 * This is the officially maintained DJI Onboard SDK library. The library provides a set of APIs for implementing the various functionality available through the [open protocol](https://developer.dji.com/onboard-sdk/documentation/introduction/index.html). 
 *
 * @section intro_sec Introduction
 *
 * API class documentation is available here. Click on the Files/Classes/Namespaces tabs above to see more information about the library. \n 
 * Documentation for the SDK has moved to the [DJI Developer Website](https://developer.dji.com/onboard-sdk/documentation/).
 * Please refer to the [Programming Guide](https://developer.dji.com/onboard-sdk/documentation/application-development-guides/programming-guide.html) 
 * for more information.
 *
 */

#ifndef DJI_API_H
#define DJI_API_H

#include "DJI_Type.h"
#include "DJI_HardDriver.h"
#include "DJI_App.h"

namespace DJI
{
namespace onboardSDK
{
class CoreAPI;
class Flight;
class Camera;
class VirtualRC;
class HotPoint;

//! @todo sort enum and move to a new file

enum ACK_ERROR_CODE
{
  ACK_SUCCESS = 0x0000,
  ACK_PARAM_ERROR = 0x0001
};

enum ACK_COMMON_CODE
{
  ACK_COMMON_SUCCESS = 0x0000,
  ACK_COMMON_KEYERROR = 0xFF00,
  ACK_COMMON_NO_AUTHORIZATION = 0xFF01,
  ACK_COMMON_NO_RIGHTS = 0xFF02,
  ACK_COMMON_NO_RESPONSE = 0xFFFF
};

enum ACK_ACTIVE_CODE
{
  ACK_ACTIVE_SUCCESS = 0x0000,
  ACK_ACTIVE_PARAMETER_ERROR = 0x0001,
  ACK_ACTIVE_ENCODE_ERROR = 0x0002,
  ACK_ACTIVE_NEW_DEVICE = 0x0003,
  ACK_ACTIVE_APP_NOT_CONNECTED = 0x0004,
  ACK_ACTIVE_NO_INTERNET = 0x0005,
  ACK_ACTIVE_SERVER_REFUSED = 0x0006,
  ACK_ACTIVE_ACCESS_LEVEL_ERROR = 0x0007,
  ACK_ACTIVE_VERSION_ERROR = 0x0008
};

enum ACK_SETCONTROL_CODE
{
  ACK_SETCONTROL_ERROR_MODE = 0x0000,
  ACK_SETCONTROL_RELEASE_SUCCESS = 0x0001,
  ACK_SETCONTROL_OBTAIN_SUCCESS = 0x0002,
  ACK_SETCONTROL_OBTAIN_RUNNING = 0x0003,
  ACK_SETCONTROL_RELEASE_RUNNING = 0x0004,
  ACK_SETCONTROL_NEED_MODE_F = 0x0006,
  ACK_SETCONTROL_NEED_MODE_P = 0x0005,
  ACK_SETCONTROL_IOC = 0x00C9,

};

enum ACK_ARM_CODE
{
  ACK_ARM_SUCCESS = 0x0000,
  ACK_ARM_NEED_CONTROL = 0x0001,
  ACK_ARM_ALREADY_ARMED = 0x0002,
  ACK_ARM_IN_AIR = 0x0003,
};

enum TASK_ACK_CODE
{ 
  TASK_FAILURE = 0x01,
  TASK_SUCCESS = 0x02
};


//! @note end of ACKs

enum CMD_SET
{
  SET_ACTIVATION = 0x00,
  SET_CONTROL = 0x01,
  SET_BROADCAST = 0x02,
  SET_MISSION = 0x03,
  SET_SYNC = 0x04,
  SET_VIRTUALRC = 0x05
};

enum SYNC_CODE
{
  CODE_SYNC_BROADCAST = 0x00
};

enum HOTPOINT_CODE
{
  CODE_HOTPOINT_START = 0x20,
  CODE_HOTPOINT_STOP = 0x21,
  CODE_HOTPOINT_SETPAUSE = 0x22,
  CODE_HOTPOINT_YAWRATE = 0x23,
  CODE_HOTPOINT_RADIUS = 0x24,
  CODE_HOTPOINT_SETYAW = 0x25,
  CODE_HOTPOINT_LOAD = 0x26
};

enum FOLLOW_CODE
{
  CODE_FOLLOW_START = 0x30,
  CODE_FOLLOW_STOP = 0x31,
  CODE_FOLLOW_SETPAUSE = 0X32,
  CODE_FOLLOW_TARGET = 0X33
};

enum WAYPOINT_CODE
{
  CODE_WAYPOINT_INIT = 0x10,
  CODE_WAYPOINT_ADDPOINT = 0x11,
  CODE_WAYPOINT_SETSTART = 0x12,
  CODE_WAYPOINT_SETPAUSE = 0x13,
  CODE_WAYPOINT_INFO_READ = 0x14,
  CODE_WAYPOINT_INDEX_READ = 0x15,
  CODE_WAYPOINT_SETVELOCITY = 0x16,
  CODE_WAYPOINT_GETVELOCITY = 0x17,
};

enum ACTIVATION_CODE
{
  CODE_GETVERSION = 0,
  CODE_ACTIVATE = 1,
  CODE_FREQUENCY = 0x10,
  CODE_TOMOBILE = 0xFE
};

enum CONTROL_CODE
{
  CODE_SETCONTROL = 0,
  CODE_TASK = 1,
  CODE_STATUS = 2,
  CODE_CONTROL = 3,
  CODE_SETARM = 5,
};

enum BROADCAST_CODE
{
  CODE_BROADCAST = 0x00,
  CODE_LOSTCTRL = 0x01,
  CODE_FROMMOBILE = 0x02,
  CODE_MISSION = 0x03,
  CODE_WAYPOINT = 0x04
};

enum VIRTUALRC_CODE
{
  CODE_VIRTUALRC_SETTINGS,
  CODE_VIRTUALRC_DATA
};

enum MISSION_TYPE
{
  MISSION_MODE_A,
  MISSION_WAYPOINT,
  MISSION_HOTPOINT,
  MISSION_FOLLOW,
  MISSION_IOC
};

enum BROADCAST_FREQ
{
  BROADCAST_FREQ_0HZ = 0,
  BROADCAST_FREQ_1HZ = 1,
  BROADCAST_FREQ_10HZ = 2,
  BROADCAST_FREQ_50HZ = 3,
  BROADCAST_FREQ_100HZ = 4,
  BROADCAST_FREQ_HOLD = 5,
};

//! CoreAPI implements core Open Protocol communication between M100/M600/A3 and your onboard embedded platform.
/*!\remark
 *  API is running on two poll threads:\n
 *  - sendPoll();\n
 *  - readPoll();\n
 *  Please make sure both threads operate correctly.\n
 *
 * @note
 * if you can read data in a interrupt, try to pass data through
 * byteHandler() or byteStreamHandler()
 *
 */
class CoreAPI
{
  public:
  CoreAPI(HardDriver *Driver = 0, Version SDKVersion = 0, bool userCallbackThread = false,
        CallBack userRecvCallback = 0, UserData userData = 0);
  CoreAPI(HardDriver *Driver, Version SDKVersion, CallBackHandler userRecvCallback,
        bool userCallbackThread = false);
  void sendPoll(void);
  void readPoll(void);
  //! @todo Implement callback poll handler
  void callbackPoll(CoreAPI *api);

  //! @todo Pipeline refactoring
  void byteHandler(const uint8_t in_data);
  //! @todo Implement stream handler
  void byteStreamHandler(uint8_t *buffer, size_t size);

  void ack(req_id_t req_id, unsigned char *ackdata, int len);

  //! Notify caller ACK frame arrived
  void allocateACK(Header *protocolHeader);
  void notifyCaller(Header *protocolHeader);
  void notifyNonBlockingCaller(Header *protocolHeader);

  //@{
  /**
   * @remark
   * void send() - core overloaded function which can be invoked in three different ways.\n\n
   * void send(CallbackCommand *parameter) - main interface\n
   * (other two overloaded functions are builded on the base of this function)\n\n
   * Please be careful when passing in UserData, there might have memory leak problems.
   *
   */

  void send(unsigned char session_mode, unsigned char is_enc, CMD_SET cmd_set,
      unsigned char cmd_id, void *pdata, int len, CallBack ack_callback,
      /**@note Compatible for DJI_APP_Pro_send*/
      int timeout = 0, int retry_time = 1);
  void send(unsigned char session_mode, bool is_enc, CMD_SET cmd_set, unsigned char cmd_id,
      void *pdata, size_t len, int timeout = 0, int retry_time = 1,
      CallBack ack_handler = 0,
      /**@note Better interface entrance*/
      UserData userData = 0);

  /**@note Main interface*/
  void send(Command *parameter);
  //@}

  /// Activation Control
  /**
   *
   * @drief
   * Send activation request to your flight controller
   * to check if: \n a) your application registered in your developer
   * account \n b) API Control enabled in the Assistant software\n\n
   * Proceed to programming if activation successful.
   */
  void activate(ActivateData *data, CallBack callback = 0, UserData userData = 0);

  /// Blocking API Control
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
  unsigned short activate(ActivateData *data, int timeout);

  void setControl(bool enable, CallBack callback = 0, UserData userData = 0);

  /// Blocking API Control
  /**
  * @remark
  * Blocks until ACK frame arrives or timeout occurs
  *
  * @brief
  * Obtain control
  *
  * @return ACK from flight controller
  *
  * @todo
  * Implement high resolution timer to catch ACK timeout
  */
  unsigned short setControl(bool enable, int timeout);

  /// Activation Control
  /**
   * @brief
   * Is your aircraft already activated ?
   */
  void setActivation(bool isActivated);

  /// Activation Control
  /**
   * Get Activation information
   */
  ActivateData getAccountData() const;

  /// Activation Control
  void setAccountData(const ActivateData &value);

  void sendToMobile(uint8_t *data, uint8_t len, CallBack callback = 0, UserData userData = 0);

  /**
   * @drief
   * Set broadcast frequency.
   *
   * @remark
   * We offer 12 frequency channels to customize:\n\n
   * 0 - Timestamp\n
   * 1 - Attitude Quaterniouns\n
   * 2 - Acceleration\n
   * 3 - Velocity (Ground Frame)\n
   * 4 - Angular Velocity (Body Frame)\n
   * 5 - Position\n
   * 6 - Magnetometer\n
   * 7 - RC Channels Data\n
   * 8 - Gimbal Data\n
   * 9 - Flight Status\n
   * 10 - Battery Level\n
   * 11 - Control Information\n
   */
  void setBroadcastFreq(uint8_t *dataLenIs16, CallBack callback = 0, UserData userData = 0);
  unsigned short setBroadcastFreq(uint8_t *dataLenIs16, int timeout);

  /**
   * Reset all broadcast frequencies to their default values
   */
  void setBroadcastFreqDefaults();

  /**
   * Blocking API Control
   *
   * @brief
   * Set broadcast frequencies to their default values and block until
   * ACK arrives from flight controller
   *
   * @return ACK from flight controller
   *
   * @todo
   * Implement high resolution timer to catch ACK timeout
   */
  unsigned short setBroadcastFreqDefaults(int timeout);
   
  /*
   * Set all broadcast frequencies to zero. Only ACK data will stay on the line.
   */
  void setBroadcastFreqToZero();

  /**
   * Let user know when ACK and Broadcast messages processed
   */
  void setACKFrameStatus(uint32_t usageFlag);
  uint32_t getACKFrameStatus();
  void setBroadcastFrameStatus(bool isFrame);
  bool getBroadcastFrameStatus();

  void setSyncFreq(uint32_t freqInHz);
  void setKey(const char *key);

  //@{
  /**
   * Get aircraft version.
   *
   * @note
   * You can query your flight controller prior to activation.
   */
  void getDroneVersion(CallBack callback = 0, UserData userData = 0);

  /**
   * Blocking API Control
   *
   * @brief
   * Get drone version from flight controller block until
   * ACK arrives from flight controller
   *
   * @return VersionData containing ACK value, CRC of the
   * protocol version and protocol version itself
   *
   * @todo
   * Implement high resolution timer to catch ACK timeout
   */
  VersionData getDroneVersion(int timeout);

  /**Get broadcasted data values from flight controller.*/
  BroadcastData getBroadcastData() const;

  bool nonBlockingCBThreadEnable;

  /**
   * Get timestamp from flight controller.
   *
   * @note
   * Make sure you are using appropriate timestamp broadcast frequency. See setBroadcastFreq\n
   * function for more details.
   */
  TimeStampData getTime() const;

  /**
   * Get flight status at any time during a flight mission.
   */
  FlightStatus getFlightStatus() const;
  CtrlInfoData getCtrlInfo() const;

  /**
   * Get battery capacity.
   *
   * @note
   * Flight missions will not perform if battery capacity is under %50. If battery capacity
   * drops below %50 during a flight mission, aircraft will automatically "go home".
   *
   */
  BatteryData getBatteryCapacity() const;
  //@}


  /**
   * Get serial device handler.
   */
  HardDriver *getDriver() const;

  SimpleACK getSimpleACK() const;
  /**
   * Get SDK version
   */
  Version getSDKVersion() const;
  void setBroadcastCallback(CallBackHandler callback) { broadcastCallback = callback; }
  void setFromMobileCallback(CallBackHandler FromMobileEntrance);

  void setBroadcastCallback(CallBack handler, UserData userData = 0);
  void setFromMobileCallback(CallBack handler, UserData userData = 0);

  void setMisssionCallback(CallBackHandler callback) { missionCallback = callback; }
  void setHotPointCallback(CallBackHandler callback) { hotPointCallback = callback; }
  void setWayPointCallback(CallBackHandler callback) { wayPointCallback = callback; }
  void setFollowCallback(CallBackHandler callback) { followCallback = callback; }
  void setWayPointEventCallback(CallBackHandler callback);

  void setMisssionCallback(CallBack handler, UserData userData = 0);
  void setHotPointCallback(CallBack handler, UserData userData = 0);
  void setWayPointCallback(CallBack handler, UserData userData = 0);
  void setFollowCallback(CallBack handler, UserData userData = 0);
  void setWayPointEventCallback(CallBack handler, UserData userData = 0);


  static void activateCallback(CoreAPI *api, Header *protocolHeader, UserData userData = 0);
  static void getDroneVersionCallback(CoreAPI *api, Header *protocolHeader, UserData userData = 0);
  static void setControlCallback(CoreAPI *api, Header *protocolHeader, UserData userData = 0);
  static void sendToMobileCallback(CoreAPI *api, Header *protocolHeader, UserData userData = 0);
  static void setFrequencyCallback(CoreAPI *api, Header *protocolHeader, UserData userData = 0);

  /** 
   * MOS Protocol parsing lirbary functions. 
   */

  /**
   * Default MOS Protocol Parser. Calls other callback functions based on data
   */
  void parseFromMobileCallback(CoreAPI *api, Header *protocolHeader, UserData userData = 0);
  
  /**
   * Mobile Callback handler functions
   */
  void setObtainControlMobileCallback(CallBackHandler callback) {obtainControlMobileCallback = callback;}
  void setReleaseControlMobileCallback(CallBackHandler callback) {releaseControlMobileCallback = callback;}
  void setActivateMobileCallback(CallBackHandler callback) {activateMobileCallback = callback;}
  void setArmMobileCallback(CallBackHandler callback) {armMobileCallback = callback;}
  void setDisArmMobileCallback(CallBackHandler callback) {disArmMobileCallback = callback;}
  void setTakeOffMobileCallback(CallBackHandler callback) {takeOffMobileCallback = callback;}
  void setLandingMobileCallback(CallBackHandler callback) {landingMobileCallback = callback;}
  void setGoHomeMobileCallback(CallBackHandler callback) {goHomeMobileCallback = callback;}
  void setTakePhotoMobileCallback(CallBackHandler callback) {takePhotoMobileCallback = callback;}
  void setStartVideoMobileCallback(CallBackHandler callback) {startVideoMobileCallback = callback;}
  void setStopVideoMobileCallback(CallBackHandler callback) {stopVideoMobileCallback = callback;}

  /**
   * ACK decoder.
   */
  bool decodeACKStatus(unsigned short ack);
  void setBroadcastActivation(uint32_t ack);

  /**
   * Flight mission decoder.
   */
  bool decodeMissionStatus(uint8_t ack);

  /**
   *@note  Thread data
   */
  bool stopCond;

  /**
   *@note  Thread data
   */

  uint32_t ack_data;
  HotPointReadACK hotpointReadACK;
  WayPointInitACK waypointInitACK;
  MissionACKUnion missionACKUnion;

  /**
   *@note Activation status to override BroadcastData activation flag
   *
   */
  uint32_t ack_activation;

  /// Open Protocol Control
  /**
   * Get Open Protocol packet information.
   */
  SDKFilter getFilter() const;

  /// HotPoint Mission Control
  bool getHotPointData() const;

  /// WayPoint Mission Control
  bool getWayPointData() const;

  // FollowMe mission Control
  bool getFollowData() const;

  /// HotPoint Mission Control
  void setHotPointData(bool value);

  /// WayPoint Mission Control
  void setWayPointData(bool value);

  /// Follow Me Mission Control
  void setFollowData(bool value);

  /**
   * Initialize serial device
   */
  void setDriver(HardDriver *value);

  /**
   * Set SDK version.
   */
  void setVersion(const Version &value);

  /**
   * Setters and getters for Mobile CMD variables - these are used 
   * when interacting with a Data Transparent Transmission App 
   */

  /** Core functions - getters */
  bool getObtainControlMobileCMD() {return obtainControlMobileCMD;}
  bool getReleaseControlMobileCMD() {return releaseControlMobileCMD;}
  bool getActivateMobileCMD() {return activateMobileCMD;}
  bool getArmMobileCMD() {return armMobileCMD;}
  bool getDisArmMobileCMD() {return disArmMobileCMD;}
  bool getTakeOffMobileCMD() {return takeOffMobileCMD;}
  bool getLandingMobileCMD() {return landingMobileCMD;}
  bool getGoHomeMobileCMD() {return goHomeMobileCMD;}
  bool getTakePhotoMobileCMD() {return takePhotoMobileCMD;}
  bool getStartVideoMobileCMD() {return startVideoMobileCMD;}
  bool getStopVideoMobileCMD() {return stopVideoMobileCMD;}

  /** Custom missions - getters */
  bool getDrawCirMobileCMD() {return drawCirMobileCMD;}
  bool getDrawSqrMobileCMD() {return drawSqrMobileCMD;}
  bool getAttiCtrlMobileCMD() {return attiCtrlMobileCMD;}
  bool getGimbalCtrlMobileCMD() {return gimbalCtrlMobileCMD;}
  bool getWayPointTestMobileCMD() {return wayPointTestMobileCMD;}
  bool getLocalNavTestMobileCMD() {return localNavTestMobileCMD;}
  bool getGlobalNavTestMobileCMD() {return globalNavTestMobileCMD;}
  bool getVRCTestMobileCMD() {return VRCTestMobileCMD;}


  /** Advanced features: LiDAR Mapping, Collision Avoidance, Precision Missions */
  bool getStartLASMapLoggingCMD() {return startLASMapLoggingCMD;}
  bool getStopLASMapLoggingCMD() {return stopLASMapLoggingCMD;}
  bool getPrecisionMissionsCMD() {return precisionMissionCMD;}
  bool getPrecisionMissionsCollisionAvoidanceCMD() {return precisionMissionsCollisionAvoidanceCMD;}
  bool getPrecisionMissionsLidarMappingCMD() {return precisionMissionsLidarMappingCMD;}
  bool getPrecisionMissionsCollisionAvoidanceLidarMappingCMD() {return precisionMissionsCollisionAvoidanceLidarMappingCMD;}

  /** Core functions - setters */
  void setObtainControlMobileCMD(bool userInput) {obtainControlMobileCMD = userInput;}
  void setReleaseControlMobileCMD(bool userInput) {releaseControlMobileCMD= userInput;}
  void setActivateMobileCMD(bool userInput) {activateMobileCMD= userInput;}
  void setArmMobileCMD(bool userInput) {armMobileCMD= userInput;}
  void setDisArmMobileCMD(bool userInput) {disArmMobileCMD= userInput;}
  void setTakeOffMobileCMD(bool userInput) {takeOffMobileCMD= userInput;}
  void setLandingMobileCMD(bool userInput) {landingMobileCMD= userInput;}
  void setGoHomeMobileCMD(bool userInput) {goHomeMobileCMD= userInput;}
  void setTakePhotoMobileCMD(bool userInput) {takePhotoMobileCMD= userInput;}
  void setStartVideoMobileCMD(bool userInput) {startVideoMobileCMD= userInput;}
  void setStopVideoMobileCMD(bool userInput) {stopVideoMobileCMD= userInput;}

  /** Advanced features: LiDAR Mapping, Collision Avoidance, Precision Missions */
  void setStartLASMapLoggingCMD(bool userInput) {startLASMapLoggingCMD = userInput;}
  void setStopLASMapLoggingCMD(bool userInput) {stopLASMapLoggingCMD = userInput;}
  void setPrecisionMissionsCMD(bool userInput) {precisionMissionCMD = userInput;}
  void setPrecisionMissionsCollisionAvoidanceCMD(bool userInput) {precisionMissionsCollisionAvoidanceCMD = userInput;}
  void setPrecisionMissionsLidarMappingCMD(bool userInput) {precisionMissionsLidarMappingCMD = userInput;}
  void setPrecisionMissionsCollisionAvoidanceLidarMappingCMD(bool userInput) {precisionMissionsCollisionAvoidanceLidarMappingCMD = userInput;}


  /** Custom missions - setters */
  void setDrawCirMobileCMD(bool userInput) {drawCirMobileCMD = userInput;}
  void setDrawSqrMobileCMD(bool userInput) {drawSqrMobileCMD = userInput;}
  void setAttiCtrlMobileCMD(bool userInput) {attiCtrlMobileCMD = userInput;}
  void setGimbalCtrlMobileCMD(bool userInput) {gimbalCtrlMobileCMD = userInput;}
  void setWayPointTestMobileCMD(bool userInput) {wayPointTestMobileCMD = userInput;}
  void setLocalNavTestMobileCMD(bool userInput) {localNavTestMobileCMD = userInput;}
  void setGlobalNavTestMobileCMD(bool userInput) {globalNavTestMobileCMD = userInput;}
  void setVRCTestMobileCMD(bool userInput) {VRCTestMobileCMD = userInput;}



  private:
  BroadcastData broadcastData;
  uint32_t ackFrameStatus;
  bool broadcastFrameStatus;
  unsigned char encodeSendData[BUFFER_SIZE];
  unsigned char encodeACK[ACK_SIZE];

  //! Mobile Data Transparent Transmission - callbacks
  CallBackHandler fromMobileCallback;
  CallBackHandler broadcastCallback;
  CallBackHandler hotPointCallback;
  CallBackHandler wayPointCallback;
  CallBackHandler wayPointEventCallback;
  CallBackHandler followCallback;
  CallBackHandler missionCallback;
  CallBackHandler recvCallback;

  CallBackHandler obtainControlMobileCallback;
  CallBackHandler releaseControlMobileCallback;
  CallBackHandler activateMobileCallback;
  CallBackHandler armMobileCallback;
  CallBackHandler disArmMobileCallback;
  CallBackHandler takeOffMobileCallback;
  CallBackHandler landingMobileCallback;
  CallBackHandler goHomeMobileCallback;
  CallBackHandler takePhotoMobileCallback;
  CallBackHandler startVideoMobileCallback;
  CallBackHandler stopVideoMobileCallback;

  //! Mobile Data Transparent Transmission - flags

  //! Core functions
  bool obtainControlMobileCMD;
  bool releaseControlMobileCMD;
  bool activateMobileCMD;
  bool armMobileCMD;
  bool disArmMobileCMD;
  bool takeOffMobileCMD;
  bool landingMobileCMD;
  bool goHomeMobileCMD;
  bool takePhotoMobileCMD;
  bool startVideoMobileCMD;
  bool stopVideoMobileCMD;

  //! Custom Mission examples
  bool drawCirMobileCMD;
  bool drawSqrMobileCMD;
  bool attiCtrlMobileCMD;
  bool gimbalCtrlMobileCMD;
  bool wayPointTestMobileCMD;
  bool localNavTestMobileCMD;
  bool globalNavTestMobileCMD;
  bool VRCTestMobileCMD;


  //! Advanced features: LiDAR Mapping, Collision Avoidance, Precision Missions
  bool startLASMapLoggingCMD;
  bool stopLASMapLoggingCMD;
  //! Various flavors of precision missions
  bool precisionMissionCMD;
  bool precisionMissionsCollisionAvoidanceCMD;
  bool precisionMissionsLidarMappingCMD;
  bool precisionMissionsCollisionAvoidanceLidarMappingCMD;


  VersionData versionData;
  ActivateData accountData;

  unsigned short seq_num;

  SDKFilter filter;

  /// Serial Device Initialization
  void init(HardDriver *Driver, CallBackHandler userRecvCallback, bool userCallbackThread,
      Version SDKVersion);
  void recvReqData(Header *protocolHeader);
  void appHandler(Header *protocolHeader);
  void broadcast(Header *protocolHeader);

  int sendInterface(Command *parameter);
  int ackInterface(Ack *parameter);
  void sendData(unsigned char *buf);
  void setup(void);
  void setupMMU(void);
  void setupSession(void);

  MMU_Tab *allocMemory(unsigned short size);

  void freeSession(CMDSession *session);
  CMDSession *allocSession(unsigned short session_id, unsigned short size);

  void freeACK(ACKSession *session);
  ACKSession *allocACK(unsigned short session_id, unsigned short size);
  MMU_Tab MMU[MMU_TABLE_NUM];
  CMDSession CMDSessionTab[SESSION_TABLE_NUM];
  ACKSession ACKSessionTab[SESSION_TABLE_NUM - 1];
  unsigned char memory[MEMORY_SIZE];
  unsigned short encrypt(unsigned char *pdest, const unsigned char *psrc,
      unsigned short w_len, unsigned char is_ack, unsigned char is_enc,
      unsigned char session_id, unsigned short seq_num);

  void streamHandler(SDKFilter *p_filter, unsigned char in_data);
  void checkStream(SDKFilter *p_filter);
  void verifyHead(SDKFilter *p_filter);
  void verifyData(SDKFilter *p_filter);
  void callApp(SDKFilter *p_filter);
  void storeData(SDKFilter *p_filter, unsigned char in_data);
public:
  HardDriver *serialDevice;
private:
  bool callbackThread;
  bool hotPointData;
  bool wayPointData;
  bool followData;

#ifdef API_BUFFER_DATA
  public:
  void setTotalRead(const size_t &value) { totalRead = value; }
  void setOnceRead(const size_t &value) { onceRead = value; }

  size_t getTotalRead() const { return totalRead; }
  size_t getOnceRead() const { return onceRead; }

  private:
  size_t onceRead;
  size_t totalRead;
#endif // API_BUFFER_DATA
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_API_H
