/*! @brief
 *  @file DJI_Pro_API.h
 *  @version 1.0
 *  @date Nov 15, 2015
 *
 *  @abstract
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

#ifndef DJI_PRO_API_H
#define DJI_PRO_API_H
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
class Mission;
class VirtualRC;
class Swarm;

enum TASK
{
    TASK_GOHOME = 1,
    TASK_TAKEOFF = 4,
    TASK_LANDING = 6
};

enum CMD_SET
{
    SET_ACTIVATION = 0x00,
    SET_CONTROL = 0x01,
    SET_BROADCAST = 0x02,
    SET_VIRTUALRC = 0x05
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
    CODE_CONTORL = 3,
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

enum CAMERA_CODE
{
    CODE_GIMBAL_SPEED = 0x1A,
    CODE_GIMBAL_ANGLE = 0x1B,
    CODE_CAMERA_SHOT = 0x20,
    CODE_CAMERA_VIDEO_START = 0x21,
    CODE_CAMERA_VIDEO_STOP = 0x22
};

enum VIRTUALRC_CODE
{
    CODE_VIRTUALRC_SETTINGS,
    CODE_VIRTUALRC_DATA
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
     *byteHandler();
     * */
  public:
    void sendPoll(void);
    void readPoll(void);

    void byteHandler(const uint8_t in_data);

  public:
    /*! @code CoreAPI*/
    CoreAPI(HardDriver *Driver, ReceiveHandler user_cmd_handler_entrance = 0);
    void ack(req_id_t req_id, unsigned char *ackdata, int len);
    void send(unsigned char session_mode, unsigned char is_enc, CMD_SET cmd_set,
              unsigned char cmd_id, void *pdata, int len,
              CallBack ack_callback = 0, int timeout = 0, int retry_time = 1);
    void send(Command *parameter);

    void activate(ActivateData *data, CallBack callback = 0);
    void setControl(bool enable, CallBack callback = 0);
    void getVersion(CallBack callback = 0);
    void sendToMobile(uint8_t *data, uint8_t len, CallBack callback = 0);
    void setBroadcastFeq(uint8_t *data, CallBack callback = 0);

    /*! @code Flight contorl
     *  @note These functions is based on API functions above.
     *  @todo move to a new class
     */

    void setGimbalAngle(GimbalAngleData *p_user_data);
    void setGimbalSpeed(GimbalSpeedData *p_user_data);
    void setCamera(CAMERA_CODE camera_cmd);

    QuaternionData getQuaternion() const;
    BroadcastData getBroadcastData() const;
    VelocityData getGroundSpeed() const;
    CtrlInfoData getCtrlInfo() const;
    BatteryData getBatteryCapacity() const;
    CommonData getGroundAcc() const;

    /*! @code user functions entrance*/
    void setTransparentTransmissionCallback(
        TransparentHandler transparentHandlerEntrance);
    void setBroadcastCallback(BroadcastHandler broadcastHandlerEntrance);

  private:
    BroadcastData broadcastData;

  private:
    //! @note reconstructed from DJI_Pro_App.cpp
    unsigned char encodeSendData[BUFFER_SIZE];
    unsigned char encodeACK[ACK_SIZE];

    ReceiveHandler recvHandler;
    BroadcastHandler broadcastHandler;
    TransparentHandler transparentHandler;

    VersionData versionData;
    ActivateData accountData;

    unsigned short seq_num;

    SDKFilter filter;

  public:
    static void activateCallback(CoreAPI *This, Header *header);
    static void getVersionCallback(CoreAPI *This, Header *header);
    static void setControlCallback(CoreAPI *This, Header *header);
    static void sendToMobileCallback(CoreAPI *This, Header *header);
    static void setFrequencyCallback(CoreAPI *This, Header *header);

  private:
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
                           unsigned short w_len, unsigned char is_ack,
                           unsigned char is_enc, unsigned char session_id,
                           unsigned short seq_num);

    void streamHandler(SDKFilter *p_filter, unsigned char in_data);
    void checkStream(SDKFilter *p_filter);
    void verifyHead(SDKFilter *p_filter);
    void verifyData(SDKFilter *p_filter);
    void callApp(SDKFilter *p_filter);
    void setKey(const char *key);

  public:
    HardDriver *getDriver() const;
    void setDriver(HardDriver *value);

  private:
    HardDriver *driver;
};

class Flight
{
  public:
    Flight(CoreAPI *ContorlAPI = 0);

    void task(TASK taskname, CallBack TaskCallback = 0);
    void setArm(bool enable, CallBack ArmCallback = 0);
    void setFlight(FlightData *data);

  public: //! @note callbacks
    static void armCallback(CoreAPI *This, Header *header);
    static void taskCallback(CoreAPI *This, Header *header);

  public: //! @note Access method
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
    TaskData taskData;
};

class Camera
{
  public:
    Camera(CoreAPI *ContorlAPI = 0);
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
};

class Mission
{
  public:
    Mission(CoreAPI *ContorlAPI = 0);
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
};

class Swarm
{
  public:
    Swarm(CoreAPI *ContorlAPI = 0);
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
};

class VirtualRC
{
  public:
    enum CutOff
    {
        CutOff_ToLogic = 0,
        CutOff_ToRealRC = 1
    };

  public:
    VirtualRC(CoreAPI *ContorlAPI = 0);

    void sentContorl(bool enable, CutOff cutoffType);
    void sendData(VirtualRCData Data);
    void sendData();
    void resetData();

  public:
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
    VirtualRCData data;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_PRO_API_H
