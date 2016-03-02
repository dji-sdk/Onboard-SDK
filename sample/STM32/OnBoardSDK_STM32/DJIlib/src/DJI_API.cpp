#include "DJI_API.h"
#include <string.h>
#include <stdio.h>

using namespace DJI::onboardSDK;

CoreAPI::CoreAPI(HardDriver *Driver, bool userCallbackThread, CallBack userRecvCallback,
                 UserData UserData)
{
    CallBackHandler handler;
    handler.callback = userRecvCallback;
    handler.userData = UserData;
    init(Driver, handler, userCallbackThread);
}

void CoreAPI::init(HardDriver *Driver, CallBackHandler userRecvCallback,
                   bool userCallbackThread)
{
    driver = Driver;
    // driver->init();

    seq_num = 0;
    filter.recv_index = 0;
    filter.reuse_count = 0;
    filter.reuse_index = 0;
    filter.enc_enabled = 0;
    broadcastCallback.callback = 0;
    broadcastCallback.userData = 0;
    fromMobileCallback.callback = 0;
    fromMobileCallback.userData = 0;
    recvCallback.callback = userRecvCallback.callback;
    recvCallback.userData = userRecvCallback.userData;

#ifndef SDK_VERSION_2_3
    broadcastData.timeStamp.time = 0;
    broadcastData.timeStamp.asr_ts = 0;
    broadcastData.timeStamp.sync_flag = 0;
#else
    broadcastData.timeStamp = 0;
#endif // SDK_VERSION_2_3

    hotPointData = true;
    followData = true;
    callbackThread = userCallbackThread;

    setup();
}

CoreAPI::CoreAPI(HardDriver *Driver, CallBackHandler userRecvCallback, bool userCallbackThread)
{
    init(Driver, userRecvCallback, userCallbackThread);
    // getVersion();
}

void CoreAPI::send(unsigned char session_mode, unsigned char is_enc, CMD_SET cmd_set,
                   unsigned char cmd_id, void *pdata, int len, CallBack ack_callback,
                   int timeout, int retry_time)
{
    Command param;
    unsigned char *ptemp = (unsigned char *)encodeSendData;
    *ptemp++ = cmd_set;
    *ptemp++ = cmd_id;

    memcpy(encodeSendData + SET_CMD_SIZE, pdata, len);

    param.handler = ack_callback;
    param.session_mode = session_mode;
    param.length = len + SET_CMD_SIZE;
    param.buf = encodeSendData;
    param.retry_time = retry_time;

    param.timeout = timeout;
    param.need_encrypt = is_enc;

    param.userData = 0;

    sendInterface(&param);
}

void CoreAPI::send(unsigned char session_mode, bool is_enc, CMD_SET cmd_set,
                   unsigned char cmd_id, void *pdata, size_t len, int timeout, int retry_time,
                   CallBack ack_handler, UserData userData)
{
    Command param;
    unsigned char *ptemp = (unsigned char *)encodeSendData;
    *ptemp++ = cmd_set;
    *ptemp++ = cmd_id;

    memcpy(encodeSendData + SET_CMD_SIZE, pdata, len);

    param.handler = ack_handler;
    param.session_mode = session_mode;
    param.length = len + SET_CMD_SIZE;
    param.buf = encodeSendData;
    param.retry_time = retry_time;

    param.timeout = timeout;
    param.need_encrypt = is_enc ? 1 : 0;

    param.userData = userData;

    sendInterface(&param);
}

void CoreAPI::send(Command *parameter) { sendInterface(parameter); }

void CoreAPI::ack(req_id_t req_id, unsigned char *ackdata, int len)
{
    Ack param;

    memcpy(encodeACK, ackdata, len);

    param.session_id = req_id.session_id;
    param.seq_num = req_id.sequence_number;
    param.need_encrypt = req_id.need_encrypt;
    param.buf = encodeACK;
    param.length = len;

    this->ackInterface(&param);
}

void CoreAPI::getVersion(CallBack callback, UserData userData)
{
    versionData.version_ack = AC_COMMON_NO_RESPONSE;
    versionData.version_crc = 0x0;
    versionData.version_name[0] = 0;

    unsigned cmd_timeout = 100; // unit is ms
    unsigned retry_time = 3;
    unsigned char cmd_data = 0;

    send(2, 0, SET_ACTIVATION, CODE_GETVERSION, (unsigned char *)&cmd_data, 1, cmd_timeout,
         retry_time, callback ? callback : CoreAPI::getVersionCallback, userData);
}

void CoreAPI::activate(ActivateData *data, CallBack callback, UserData userData)
{
    accountData = *data;
    for (int i = 0; i < 32; ++i) data->app_bundle_id[i] = '9'; //! @note for ios verification
    send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&accountData,
         sizeof(accountData) - sizeof(char *), 1000, 3,
         callback ? callback : CoreAPI::activateCallback, userData);
}

void CoreAPI::sendToMobile(uint8_t *data, uint8_t len, CallBack callback, UserData userData)
{
    if (len > 100)
    {
        API_LOG(driver, ERROR_LOG, "Too much data to send");
        return;
    }
    send(2, 0, SET_ACTIVATION, CODE_TOMOBILE, data, len, 500, 2,
         callback ? callback : CoreAPI::sendToMobileCallback, userData);
}

void CoreAPI::setBroadcastFreq(uint8_t *dataLenIs16, CallBack callback,
                               UserData userData)
{
    for (int i = 0; i < 16; ++i)
    {
        if (dataLenIs16[i] < 12)
            dataLenIs16[i] = (dataLenIs16[i] > 5 ? 5 : 0);
        else
            dataLenIs16[i] = 0;
    }
    send(2, 0, SET_ACTIVATION, CODE_FREQUENCY, dataLenIs16, 16, 100, 1,
         callback ? callback : CoreAPI::setFrequencyCallback, userData);
}

TimeStampData CoreAPI::getTime() const { return broadcastData.timeStamp; }

FlightStatus CoreAPI::getFlightStatus() const { return broadcastData.status; }
ActivateData CoreAPI::getAccountData() const { return accountData; }

void CoreAPI::setAccountData(const ActivateData &value) { accountData = value; }
void CoreAPI::setHotPointData(bool value) { hotPointData = value; }
void CoreAPI::setWayPointData(bool value) { wayPointData = value; }
void CoreAPI::setFollowData(bool value) { followData = value; }
bool CoreAPI::getHotPointData() const { return hotPointData; }
bool CoreAPI::getWayPointData() const { return wayPointData; }
bool CoreAPI::getFollowData() const { return followData; }

void CoreAPI::setControl(bool enable, CallBack callback, UserData userData)
{
    unsigned char data = enable ? 1 : 0;
    send(2, 1, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2,
         callback ? callback : CoreAPI::setControlCallback, userData);
}

HardDriver *CoreAPI::getDriver() const { return driver; }

void CoreAPI::setDriver(HardDriver *value) { driver = value; }

void CoreAPI::getVersionCallback(CoreAPI *This, Header *header, UserData userData __UNUSED)
{
    unsigned char *ptemp = ((unsigned char *)header) + sizeof(Header);

    This->versionData.version_ack = ptemp[0] + (ptemp[1] << 8);
    ptemp += 2;
    This->versionData.version_crc =
        ptemp[0] + (ptemp[1] << 8) + (ptemp[2] << 16) + (ptemp[3] << 24);
    ptemp += 4;
#ifdef SDK_VERSION_3_1
    memcpy(This->versionData.version_ID, ptemp, 11);
    ptemp += 11;
#endif
    memcpy(This->versionData.version_name, ptemp, 32);

    API_LOG(This->driver, STATUS_LOG, "version ack = %d\n", This->versionData.version_ack);
    API_LOG(This->driver, STATUS_LOG, "version crc = 0x%X\n", This->versionData.version_crc);
#ifdef SDK_VERSION_3_1
    API_LOG(This->driver, STATUS_LOG, "version ID = %s\n", This->versionData.version_ID);
#endif
    API_LOG(This->driver, STATUS_LOG, "version name = %s\n", This->versionData.version_name);
}

void CoreAPI::activateCallback(CoreAPI *This, Header *header, UserData userData __UNUSED)
{

    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        switch (ack_data)
        {
            case ACK_ACTIVE_SUCCESS:
                API_LOG(This->driver, STATUS_LOG, "Activation Successfully\n");

                This->getDriver()->lockMSG();
                This->broadcastData.activation = 1;
                This->getDriver()->freeMSG();

                if (This->accountData.app_key)
                    This->setKey(This->accountData.app_key);
                return;
            case ACK_ACTIVE_NEW_DEVICE:
                API_LOG(This->driver, STATUS_LOG, "new device try again\n");
                break;
            case ACK_ACTIVE_PARAMETER_ERROR:
                API_LOG(This->driver, ERROR_LOG, "activate Wrong pahameter\n");
                break;
            case ACK_ACTIVE_ENCODE_ERROR:
                API_LOG(This->driver, ERROR_LOG, "activate encode error\n");
                break;
            case ACK_ACTIVE_APP_NOT_CONNECTED:
                API_LOG(This->driver, ERROR_LOG, "activate DJIGO not connected\n");
                break;
            case ACK_ACTIVE_NO_INTERNET:
                API_LOG(This->driver, ERROR_LOG,
                        "activate DJIGO not connected to the internet\n");
                break;
            case ACK_ACTIVE_SERVER_REFUSED:
                API_LOG(This->driver, ERROR_LOG, "activate DJI server reject "
                                                 "your request, please use an "
                                                 "available SDK ID\n");
                break;
            case ACK_ACTIVE_ACCESS_LEVEL_ERROR:
                API_LOG(This->driver, ERROR_LOG, "activate Wrong SDK permission\n");
                break;
            case ACK_ACTIVE_VERSION_ERROR:
                API_LOG(This->driver, ERROR_LOG, "activate SDK version did not match\n");
                This->getVersion();
                break;
            default:
                if (!This->decodeACKStatus(ack_data))
                {
                    API_LOG(This->driver, ERROR_LOG, "While calling this function");
                }
                break;
        }
        This->getDriver()->lockMSG();
        This->broadcastData.activation = 0;
        This->getDriver()->freeMSG();
    }
    else
    {
        API_LOG(This->driver, ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequence_number);
    }
}

void CoreAPI::sendToMobileCallback(CoreAPI *This, Header *header, UserData userData __UNUSED)
{
    unsigned short ack_data = AC_COMMON_NO_RESPONSE;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (!This->decodeACKStatus(ack_data))
        {
            API_LOG(This->driver, ERROR_LOG, "While calling this function");
        }
    }
    else
    {
        API_LOG(This->driver, ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequence_number);
    }
}

void CoreAPI::setFrequencyCallback(CoreAPI *This __UNUSED, Header *header,
                                   UserData userData __UNUSED)
{
    unsigned short ack_data = AC_COMMON_NO_RESPONSE;

    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    }
    switch (ack_data)
    {
        case 0x0000:
            API_LOG(This->driver, STATUS_LOG, "Frequency set success");
            break;
        case 0x0001:
            API_LOG(This->driver, ERROR_LOG, "Frequency parameter error");
            break;
        default:
            if (!This->decodeACKStatus(ack_data))
            {
                API_LOG(This->driver, ERROR_LOG, "While calling this function");
            }
            break;
    }
}

void CoreAPI::setControlCallback(CoreAPI *This, Header *header, UserData userData __UNUSED)
{
    unsigned short ack_data = AC_COMMON_NO_RESPONSE;
    unsigned char data = 0x1;

    if (header->length - EXC_DATA_SIZE <= sizeof(ack_data))
    {
        memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    }
    else
    {
        API_LOG(This->driver, ERROR_LOG, "ACK is exception,seesion id %d,sequence %d\n",
                header->sessionID, header->sequence_number);
    }

    switch (ack_data)
    {
        case ACK_SETCONTROL_NEED_MODE_F:
            API_LOG(This->driver, STATUS_LOG, "Obtain control failed, Conditions did not "
                                              "satisfied");
            break;
        case ACK_SETCONTROL_RELEASE_SUCCESS:
            API_LOG(This->driver, STATUS_LOG, "release control successfully\n");
            break;
        case ACK_SETCONTROL_OBTAIN_SUCCESS:
            API_LOG(This->driver, STATUS_LOG, "obtain control successfully\n");
            break;
        case ACK_SETCONTROL_OBTAIN_RUNNING:
            API_LOG(This->driver, STATUS_LOG, "obtain control running\n");
            This->send(2, 1, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2,
                       CoreAPI::setControlCallback);
            break;
        case ACK_SETCONTROL_RELEASE_RUNNING:
            API_LOG(This->driver, STATUS_LOG, "release control running\n");
            data = 0;
            This->send(2, 1, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2,
                       CoreAPI::setControlCallback);
            break;
        case ACK_SETCONTROL_IOC:
            API_LOG(This->driver, STATUS_LOG, "IOC mode opening can not obtain control\n");
            break;
        default:
            if (!This->decodeACKStatus(ack_data))
            {
                API_LOG(This->driver, ERROR_LOG, "While calling this function");
            }
            break;
    }
}
