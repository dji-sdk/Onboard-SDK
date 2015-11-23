/*! @brief
 *  @file DJI_Pro_API.h
 *  @version 1.0
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
enum TASK
{
    TASK_GOHOME     = 1,
    TASK_TAKEOFF    = 4,
    TASK_LANDING    = 6
};

enum CMD_SET
{
    SET_ACTIVATION  = 0x00,
    SET_CONTROL     = 0x01,
    SET_BROADCAST   = 0x02
};

enum ACTIVATION_CODE
{
    CODE_GETVERSION = 0,
    CODE_ACTIVATE   = 1,
    CODE_TOMOBILE   = 0xFE
};

enum CONTROL_CODE
{
    CODE_SETCONTROL = 0,
    CODE_TASK       = 1,
    CODE_STATUS     = 2,
    CODE_CONTORL    = 3,
    CODE_GIMBAL_SPEED = 0x1A,
    CODE_GIMBAL_ANGLE = 0x1B,
    CODE_CAMERA_SHOT  = 0x20,
    CODE_CAMERA_VIDEO_START = 0x21,
    CODE_CAMERA_VIDEO_STOP  = 0x22
};

enum BROADCAST_CODE
{
    CODE_BROADCAST  = 0x00,
    CODE_LOSTCTRL   = 0x01,
    CODE_FROMMOBILE = 0x02
};

/*! @note overdefined in CONTROL_CODE
 * Just using for test
 * */
enum CAMERA
{
    CAMERA_SHOT         = 0x20,
    CAMERA_VIDEO_START  = 0x21,
    CAMERA_VIDEO_STOP   = 0x22
};

class API
{
    /*! @brief
     *  API is running on two poll thead
     *  -sendPoll();
     *  -readPoll();
     * please make sure both thead is operating correctly
     *
     * @note
     * if you can read data in a interrupt, try to pass data through byteHandler();
     * */
public:
    void sendPoll(void);
    void readPoll(void);

    void byteHandler(const uint8_t in_data);
public:
    /*! @code API*/
    API(HardDriver *Driver, ReceiveHandler user_cmd_handler_entrance = 0);
    void send(unsigned char session_mode, unsigned char is_enc, CMD_SET cmd_set, unsigned char cmd_id,
              unsigned char *pdata,int len,CallBack ack_callback, int timeout, int retry_time);
    void send(Command *parameter);
    void ack(req_id_t req_id, unsigned char *ackdata, int len);
    int getVersion(Get_API_Version_Notify user_notice_entrance);
    int task(TASK taskname, CommandResult user_notice_entrance);
    int activate(ActivateData_t *p_user_data,CommandResult user_notice_entrance);
    int sendToMobile(unsigned char *data,unsigned char len,CommandResult user_notice_entrance);
    int setControl(unsigned char cmd,CommandResult user_notice_entrance);

    /*! @code flight contorl*/
    int setAttitude(AttitudeData_t *p_user_data);
    int setGimbalAngle(GimbalAngleData_t *p_user_data);
    int setGimbalSpeed(GimbalSpeedData_t *p_user_data);
    int setCamera(CAMERA camera_cmd);

    void getBroadcastData(BroadcastData_t *p_user_buf) const;
    void getBatteryCapacity(unsigned char *data) const;
    void getQuaternion(QuaternionData_t *p_user_buf) const;
    void getGroundAcc(CommonData_t *p_user_buf) const;
    void getGroundVo(api_vel_data_t *p_user_buf) const ;
    void getCtrlInfo(CtrlInfoData_t *p_user_buf) const;

    /*! @code user functions entrance*/
    void setTransparentTransmissionCallback(TransparentHandler transparentHandlerEntrance);
    void setBroadcastCallback(BroadcastHandler broadcastHandlerEntrance);
private:
    BroadcastData_t broadcastData;
private:
    //! @note reconstructed from DJI_Pro_App.cpp
    unsigned char Pro_Encode_Data[1024];
    unsigned char Pro_Encode_ACK[10];

    CommandResult taskResult;
    CommandResult activateResult;
    CommandResult toMobileResult;
    CommandResult setControlResult;
    ReceiveHandler recvHandler;
    BroadcastHandler broadcastHandler;
    TransparentHandler transparentHandler;

    cmd_agency_data_t status_ctrl_cmd_data;
    version_query_data_t to_user_version_data;
    ActivateData_t accountData;

    unsigned short seq_num;

    SDKFilter filter ;
private:
    static void taskCallback(API* This,Header *header);
    static void getVersionCallback(API* This,Header *header);
    static void activateCallback(API* This,Header *header);
    static void sendToMobileCallback(API* This,Header *header);
    static void setControlCallback(API* This,Header *header);

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

    MMU_Tab * allocMemory(unsigned short size);

    void freeSession(CMDSession * session);
    CMDSession* allocSession(unsigned short session_id,unsigned short size);

    void freeACK(ACKSession * session);
    ACKSession * allocACK(unsigned short session_id,unsigned short size);

private:
    //!@note memory alloc variables
    MMU_Tab MMU[MMU_TABLE_NUM];
    unsigned char Static_Memory[MEMORY_SIZE];
    CMDSession CMDSessionTab[SESSION_TABLE_NUM];
    ACKSession ACKSessionTab[SESSION_TABLE_NUM - 1]; //session 0 is a nak session id


private:
    unsigned short encrypt(unsigned char *pdest, const unsigned char *psrc,
                           unsigned short w_len,unsigned char is_ack,unsigned char is_enc,unsigned char session_id,
                           unsigned short seq_num);

    void streamHandler(SDKFilter* p_filter, unsigned char in_data);
    void checkStream(SDKFilter* p_filter);
    void verifyHead(SDKFilter* p_filter);
    void verifyData(SDKFilter* p_filter);
    void callApp(SDKFilter* p_filter);
    void setKey(const char *key);

public:
    HardDriver *getDriver() const;
    void setDriver(HardDriver *value);
private:
    HardDriver *driver;

};
}//namespace onboardSDK
}//namespace DJI


#endif // DJI_PRO_API_H
