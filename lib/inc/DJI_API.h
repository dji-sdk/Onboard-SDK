/*! @brief
 *  @file DJI_API.h
 *  @version 3.0
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
class Swarm;
class WayPoint;
class HotPoint;
class FollowMe;

//! @todo sort enum and move to a new file
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

    //! @todo modify to a new algorithm
    void byteHandler(const uint8_t in_data);
    void byteStreamHandler(uint8_t *buffer, size_t size);

  public:
    /*! @code CoreAPI*/
    //! @note init API
    CoreAPI(HardDriver *Driver, ReceiveHandler user_cmd_handler_entrance = 0);

    //! @note Core Control API
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
    void setActivation(bool isActivated);
    void setKey(const char *key);

    //! @note Core read API
    BroadcastData getBroadcastData() const;
    TimeStampData getTime() const;
    FlightStatus getFlightStatus() const;
    CtrlInfoData getCtrlInfo() const;
    BatteryData getBatteryCapacity() const;

    /*! @code Flight contorl
     *  @note These functions is based on API functions above.
     *  @todo move to a new class
     */
    VelocityData getGroundSpeed() const;
    QuaternionData getQuaternion() const;
    /*! @code user functions entrance*/
    void setTransparentTransmissionCallback(
        TransparentHandler transparentHandlerEntrance);
    void setBroadcastCallback(CallBack callback);

  private:
    BroadcastData broadcastData;

  private:
    unsigned char encodeSendData[BUFFER_SIZE];
    unsigned char encodeACK[ACK_SIZE];

    CallBack broadcastCallback;
    ReceiveHandler recvHandler;
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

  public:
    HardDriver *getDriver() const;
    void setDriver(HardDriver *value);

    ActivateData getAccountData() const;
    void setAccountData(const ActivateData &value);

  private:
    HardDriver *driver;
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

    RadioData getRCdata() const;

  public:
    CoreAPI *getApi() const;
    void setApi(CoreAPI *value);

  private:
    CoreAPI *api;
    VirtualRCData data;
};

} // namespace onboardSDK
} // namespace DJI

#endif // DJI_API_H
