#include "DJI_API.h"
#include <string.h>
#include <stdio.h>

using namespace DJI::onboardSDK;

CoreAPI::CoreAPI(HardDriver *Driver, ReceiveHandler user_cmd_handler_entrance)
{
    driver = Driver;
    seq_num = 0;
    filter.recv_index = 0;
    filter.reuse_count = 0;
    filter.reuse_index = 0;
    filter.enc_enabled = 0;
    broadcastCallback = 0;
    transparentHandler = 0;
#ifdef SDK_VERSION_3_0
    broadcastData.timeStamp.time = 0;
    broadcastData.timeStamp.asr_ts = 0;
    broadcastData.timeStamp.sync_flag = 0;
#endif

    recvHandler = user_cmd_handler_entrance ? user_cmd_handler_entrance : 0;

    setup();
}

void CoreAPI::send(unsigned char session_mode, unsigned char is_enc,
                   CMD_SET cmd_set, unsigned char cmd_id, void *pdata, int len,
                   CallBack ack_callback, int timeout, int retry_time)
{
    Command param;
    unsigned char *ptemp = (unsigned char *)encodeSendData;
    *ptemp++ = cmd_set;
    *ptemp++ = cmd_id;

    memcpy(encodeSendData + SET_CMD_SIZE, pdata, len);

    param.callback = ack_callback;
    param.session_mode = session_mode;
    param.length = len + SET_CMD_SIZE;
    param.buf = encodeSendData;
    param.retry_time = retry_time;

    param.timeout = timeout;
    param.need_encrypt = is_enc;

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

void CoreAPI::getVersion(CallBack callback)
{
    versionData.version_ack = 0xFFFF;
    versionData.version_crc = 0x0;
    versionData.version_name[0] = 0;

    unsigned cmd_timeout = 100; // unit is ms
    unsigned retry_time = 3;
    unsigned char cmd_data = 0;

    send(2, 0, SET_ACTIVATION, CODE_GETVERSION, (unsigned char *)&cmd_data, 1,
         callback ? callback : CoreAPI::getVersionCallback, cmd_timeout,
         retry_time);
}

void CoreAPI::activate(ActivateData *data, CallBack callback)
{
    accountData = *data;

    send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&accountData,
         sizeof(accountData) - sizeof(char *),
         callback ? callback : CoreAPI::activateCallback, 1000, 3);
}

void CoreAPI::sendToMobile(uint8_t *data, uint8_t len, CallBack callback)
{
    if (len > 100)
    {
        API_ERROR("Too much data to send");
        return;
    }
    send(2, 0, SET_ACTIVATION, CODE_TOMOBILE, data, len,
         callback ? callback : CoreAPI::sendToMobileCallback, 500, 2);
}

void CoreAPI::setBroadcastFeq(uint8_t *data, CallBack callback)
{
    send(2, 0, SET_ACTIVATION, CODE_FREQUENCY, data, 16,
         callback ? callback : CoreAPI::setFrequencyCallback, 100, 1);
}

TimeStampData CoreAPI::getTime() const
{
    return broadcastData.timeStamp;
}

FlightStatus CoreAPI::getFlightStatus() const
{
    return broadcastData.status;
}
ActivateData CoreAPI::getAccountData() const
{
    return accountData;
}

void CoreAPI::setAccountData(const ActivateData &value)
{
    accountData = value;
}


void CoreAPI::setControl(bool enable, CallBack callback)
{
    unsigned char data = enable ? 1 : 0;
    send(2, 1, SET_CONTROL, API_CTRL_MANAGEMENT, &data, 1,
         callback ? callback : CoreAPI::setControlCallback, 500, 2);
}

HardDriver *CoreAPI::getDriver() const { return driver; }

void CoreAPI::setDriver(HardDriver *value) { driver = value; }

void CoreAPI::getVersionCallback(CoreAPI *This, Header *header)
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

    API_STATUS("version ack = %d\n", This->versionData.version_ack);
    API_STATUS("version crc = 0x%X\n", This->versionData.version_crc);
#ifdef SDK_VERSION_3_1
    API_STATUS("version ID = %s\n", This->versionData.version_ID);
#endif
    API_STATUS("version name = %s\n", This->versionData.version_name);
}

void CoreAPI::activateCallback(CoreAPI *This, Header *header)
{
    volatile unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (ack_data == SDK_ACTIVATE_NEW_DEVICE)
        {
            API_STATUS("new device try again\n");
        }
        else
        {
            if (ack_data == SDK_ACTIVATE_SUCCESS)
            {
                API_STATUS("Activation Successfully\n");

                This->getDriver()->lockMSG();
                This->broadcastData.activation = 1;
                This->getDriver()->freeMSG();

                if (This->accountData.app_key)
                    This->setKey(This->accountData.app_key);
            }
            else
            {
                API_ERROR("activate code:0x%X\n", ack_data);

                This->getDriver()->lockMSG();
                This->broadcastData.activation = 0;
                This->getDriver()->freeMSG();
            }
        }
    }
    else
    {
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);
    }
}

void CoreAPI::sendToMobileCallback(CoreAPI *This, Header *header)
{
    unsigned short ack_data = 0xFFFF;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    }
    else
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);
}

void CoreAPI::setFrequencyCallback(CoreAPI *This, Header *header)
{
    unsigned short ack_data = 0xFFFF;

    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    }
    switch (ack_data)
    {
        case 0x0000:
            API_STATUS("Frequency set success");
            break;
        case 0x0001:
            API_ERROR("Frequency parameter error");
            break;
        default:
            API_ERROR("Hardware fault");
            break;
    }
}

void CoreAPI::setControlCallback(CoreAPI *This, Header *header)
{
    unsigned short ack_data = 0xFFFF;
    unsigned char data = 0x1;

    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
    }
    else
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);

    switch (ack_data)
    {
        case 0x0000:
            API_STATUS("Obtain control failed, Conditions did not "
                       "satisfied");
            break;
        case 0x0001:
            API_STATUS("release control successfully\n");
            break;
        case 0x0002:
            API_STATUS("obtain control successfully\n");
            break;
        case 0x0003:
            API_STATUS("obtain control running\n");
            This->send(2, 1, SET_CONTROL, API_CTRL_MANAGEMENT, &data, 1,
                       CoreAPI::setControlCallback, 500, 2);
            break;
        case 0x0004:
            API_STATUS("release control running\n");
            data = 0;
            This->send(2, 1, SET_CONTROL, API_CTRL_MANAGEMENT, &data, 1,
                       CoreAPI::setControlCallback, 500, 2);
            break;
        case 0x00C9:
            API_STATUS("IOC mode opening can not obtain control\n");
            break;
        default:
            API_STATUS("there is unkown error,ack=0x%X\n", ack_data);
            break;
    }
}

Camera::Camera(CoreAPI *ContorlAPI) { api = ContorlAPI; }

void Camera::setCamera(Camera::CAMERA_CODE camera_cmd)
{
    unsigned char send_data = 0;
    api->send(0, 1, SET_CONTROL, camera_cmd, &send_data, 1);
}

void Camera::setGimbalAngle(GimbalAngleData *data)
{
    api->send(0, 1, SET_CONTROL, Camera::CODE_GIMBAL_ANGLE,
              (unsigned char *)data, sizeof(GimbalAngleData));
}

void Camera::setGimbalSpeed(GimbalSpeedData *data)
{
    data->reserved = 0x80;
    api->send(0, 1, SET_CONTROL, Camera::CODE_GIMBAL_SPEED,
              (unsigned char *)data, sizeof(GimbalSpeedData));
}

CoreAPI *Camera::getApi() const { return api; }

void Camera::setApi(CoreAPI *value) { api = value; }

CoreAPI *Mission::getApi() const { return api; }

void Mission::setApi(CoreAPI *value) { api = value; }
