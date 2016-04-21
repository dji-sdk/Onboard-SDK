/*! @brief
 *  @file DJI_API.h
 *  @version 3.0
 *  @date Nov 15, 2015
 *
 *  @abstract
 *  Core API for DJI onboardSDK library
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for Windows,QT,STM32,ROS,Cmake
 *  -* @date Nov 15, 2015
 *  -* @author william.wu
 *
 * */

#ifndef DJI_API_H
#define DJI_API_H
#include "DJI_Type.h"
//#include "DJI_Mission.h"

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
    ACK_SETCONTROL_NEED_MODE_F = 0x0000,
    ACK_SETCONTROL_RELEASE_SUCCESS = 0x0001,
    ACK_SETCONTROL_OBTAIN_SUCCESS = 0x0002,
    ACK_SETCONTROL_OBTAIN_RUNNING = 0x0003,
    ACK_SETCONTROL_RELEASE_RUNNING = 0x0004,
    ACK_SETCONTROL_IOC = 0x00C9,

};

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
    CODE_HOTPOINT_PALSTANCE = 0x23,
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
    CODE_WAYPOINT_DOWNLOAD = 0x14,
    CODE_WAYPOINT_INDEX = 0x15,
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

class CoreAPI
{
    /*! @brief
     *  API is running on two poll thead
     *  -sendPoll();
     *  -readPoll();
     * please make sure both thead is operating correctly
     *
     * @note
     * if you can read data in a interrupt, try to pass data through
     * byteHandler() or byteStreamHandler()
     * */
  public:
    void sendPoll(void);
    void readPoll(void);
    void callbackPoll(void); //! @todo implement, not available yet

    void byteHandler(const uint8_t in_data);
    //! @todo modify to a new algorithm
    void byteStreamHandler(uint8_t *buffer, size_t size);

  public:
    /*! @code CoreAPI*/
    //! @note init API
    CoreAPI(HardDriver *Driver = 0, Version SDKVersion = 0, bool userCallbackThread = false,
            CallBack userRecvCallback = 0, UserData userData = 0);
    CoreAPI(HardDriver *Driver, Version SDKVersion, CallBackHandler userRecvCallback,
            bool userCallbackThread = false);

    /*! @note Core Control API
     *  void send(); is a core overloaded funtion. which has three ways invoked.
     *
     *  void send(CallbackCommand *parameter) is a main interface, other two
     *  overloaded functions are builded on the base of this function.
     *
     *  Please be careful when passing in UserData, there might have memory leak
     *  problems.
     * */
    void ack(req_id_t req_id, unsigned char *ackdata, int len);
    void send(unsigned char session_mode, unsigned char is_enc, CMD_SET cmd_set,
              unsigned char cmd_id, void *pdata, int len, CallBack ack_callback,
              int timeout = 0, int retry_time = 1); //! @note compatible for DJI_APP_Pro_send
    void send(unsigned char session_mode, bool is_enc, CMD_SET cmd_set, unsigned char cmd_id,
              void *pdata, size_t len, int timeout = 0, int retry_time = 1,
              CallBack ack_handler = 0,
              UserData userData = 0); //! @note better interface entrance
    void send(Command *parameter);	//! @note main interface

    //! @note control API
    void activate(ActivateData *data, CallBack callback = 0, UserData userData = 0);
    void setControl(bool enable, CallBack callback = 0, UserData userData = 0);
    void getDroneVersion(CallBack callback = 0, UserData userData = 0);
    void sendToMobile(uint8_t *data, uint8_t len, CallBack callback = 0, UserData userData = 0);
    void setBroadcastFreq(uint8_t *dataLenIs16, CallBack callback = 0, UserData userData = 0);
    void setActivation(bool isActivated);
    void setSyncFreq(uint32_t freqInHz);
    void setKey(const char *key);

    //! @note Core read API
    BroadcastData getBroadcastData() const;
    TimeStampData getTime() const;
    FlightStatus getFlightStatus() const;
    CtrlInfoData getCtrlInfo() const;
    BatteryData getBatteryCapacity() const;

    //! @note call back functions
  public:
    //! @note Recevie data callback enterance
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

    /*! @note user callback sample
     *  @attention We can also use none-static function as a callback function.
     *  Due to saftey consideration, we strongly suggest you to define a class callback
     *  function as a static function. And passing in this pointer in a static way.
     * */
    static void activateCallback(CoreAPI *This, Header *header, UserData userData = 0);
    static void getDroneVersionCallback(CoreAPI *This, Header *header, UserData userData = 0);
    static void setControlCallback(CoreAPI *This, Header *header, UserData userData = 0);
    static void sendToMobileCallback(CoreAPI *This, Header *header, UserData userData = 0);
    static void setFrequencyCallback(CoreAPI *This, Header *header, UserData userData = 0);

  private:
    //! @todo init function
    BroadcastData broadcastData;

  private:
    unsigned char encodeSendData[BUFFER_SIZE];
    unsigned char encodeACK[ACK_SIZE];

//    uint8_t cblistTail;
//    CallBackHandler cbList[CALLBACK_LIST_NUM];
    CallBackHandler fromMobileCallback;
    CallBackHandler broadcastCallback;
    CallBackHandler hotPointCallback;
    CallBackHandler wayPointCallback;
    CallBackHandler wayPointEventCallback;
    CallBackHandler followCallback;
    CallBackHandler missionCallback;
    CallBackHandler recvCallback;

    VersionData versionData;
    ActivateData accountData;

    unsigned short seq_num;

    SDKFilter filter;

  private:
    void init(HardDriver *Driver, CallBackHandler userRecvCallback, bool userCallbackThread,
              Version SDKVersion);
    void recvReqData(Header *header);
    void appHandler(Header *header);
    void broadcast(Header *header);

    int sendInterface(Command *parameter);
    int ackInterface(Ack *parameter);
    void sendData(unsigned char *buf);

  private:
    void setup(void);
    void setupMMU(void);
    void setupSession(void);

    MMU_Tab *allocMemory(unsigned short size);

    void freeSession(CMDSession *session);
    CMDSession *allocSession(unsigned short session_id, unsigned short size);

    void freeACK(ACKSession *session);
    ACKSession *allocACK(unsigned short session_id, unsigned short size);

  private:
    //! @note memory alloc variables
    MMU_Tab MMU[MMU_TABLE_NUM];
    CMDSession CMDSessionTab[SESSION_TABLE_NUM];
    //! @note session 0 is a nak session id
    ACKSession ACKSessionTab[SESSION_TABLE_NUM - 1];
    unsigned char memory[MEMORY_SIZE];

  private:
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
    //! @note ack decoder
    bool decodeACKStatus(unsigned short ack);
    bool decodeMissionStatus(uint8_t ack);

  public:
    //! @note private variables access functions
    ActivateData getAccountData() const;
    HardDriver *getDriver() const;
    Version getSDKVersion() const;
    SDKFilter getFilter() const; //! @note evil definition
    bool getHotPointData() const;
    bool getWayPointData() const;
    bool getFollowData() const;

    void setHotPointData(bool value);
    void setWayPointData(bool value);
    void setFollowData(bool value);
    void setDriver(HardDriver *value);
    void setAccountData(const ActivateData &value);
    void setVersion(const Version &value);

  private:
    HardDriver *driver;
    bool callbackThread;
    bool hotPointData;
    bool wayPointData;
    bool followData;

//! @note debug functions:
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
