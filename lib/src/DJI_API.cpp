#include "DJI_API.h"
#include <string.h>
#include <stdio.h>

DJI::onboardSDK::CoreAPI::CoreAPI(DJI::onboardSDK::HardDriver *Driver,
                                  ReceiveHandler user_cmd_handler_entrance)
{
    driver = Driver;
    // taskData = { 0, 0 };
    seq_num = 0;
    filter.recv_index = 0;
    filter.reuse_count = 0;
    filter.reuse_index = 0;
    filter.enc_enabled = 0;
    taskResult = 0;
    activateResult = 0;
    toMobileResult = 0;
    setControlResult = 0;
    broadcastHandler = 0;
    transparentHandler = 0;
    broadcastData.timeStamp.time = 0;
    broadcastData.timeStamp.asr_ts = 0;
    broadcastData.timeStamp.sync_flag = 0;

    recvHandler = user_cmd_handler_entrance ? user_cmd_handler_entrance : 0;

    setup();
}

void DJI::onboardSDK::CoreAPI::send(unsigned char session_mode,
                                    unsigned char is_enc, CMD_SET cmd_set,
                                    unsigned char cmd_id, unsigned char *pdata,
                                    int len, CallBack ack_callback, int timeout,
                                    int retry_time)
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

void DJI::onboardSDK::CoreAPI::send(Command *parameter)
{
    sendInterface(parameter);
}

void DJI::onboardSDK::CoreAPI::ack(req_id_t req_id, unsigned char *ackdata,
                                   int len)
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

void DJI::onboardSDK::CoreAPI::task(TASK taskname,
                                    CommandResult TaskResult)
{
    unsigned char cur_cmd = 0;
    taskResult = TaskResult;
    cur_cmd = taskname;

    taskData.cmd_data = cur_cmd;
    taskData.cmd_sequence++;

    send(2, 1, SET_CONTROL, API_CMD_REQUEST, (unsigned char *)&taskData,
         sizeof(taskData), DJI::onboardSDK::CoreAPI::taskCallback, 100, 3);
}

void DJI::onboardSDK::CoreAPI::getVersion(
    Get_API_Version_Notify user_notice_entrance)
{
    versionData.version_ack = 0xFFFF;
    versionData.version_crc = 0x0;
    versionData.version_name[0] = 0;

    unsigned cmd_timeout = 100; // unit is ms
    unsigned retry_time = 3;
    unsigned char cmd_data = 0;

    send(2, 0, SET_ACTIVATION, CODE_GETVERSION, (unsigned char *)&cmd_data, 1,
         DJI::onboardSDK::CoreAPI::getVersionCallback, cmd_timeout, retry_time);
}

void DJI::onboardSDK::CoreAPI::activate(ActivateData *data,
                                        CommandResult ActivationResult)
{
    activateResult = ActivationResult ? ActivationResult : 0;
    accountData = *data;

    send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&accountData,
         sizeof(accountData) - sizeof(char *),
         DJI::onboardSDK::CoreAPI::activateCallback, 1000, 3);
}

void DJI::onboardSDK::CoreAPI::sendToMobile(uint8_t *data, uint8_t len,
                                            CommandResult MobileResult)
{
    if (len > 100)
    {
        API_ERROR("Too much data to send");
        return;
    }
    toMobileResult = MobileResult ? MobileResult : 0;
    send(2, 0, SET_ACTIVATION, CODE_TOMOBILE, data, len,
         DJI::onboardSDK::CoreAPI::sendToMobileCallback, 500, 2);
}

void DJI::onboardSDK::CoreAPI::setControl(unsigned char cmd,
                                          CommandResult user_notice_entrance)
{
    unsigned char data = cmd & 0x1;
    setControlResult = user_notice_entrance ? user_notice_entrance : 0;
    send(2, 1, SET_CONTROL, API_CTRL_MANAGEMENT, &data, 1,
         DJI::onboardSDK::CoreAPI::setControlCallback, 500, 1);
}

void DJI::onboardSDK::CoreAPI::setFlight(FlightData *p_user_data)
{
    send(0, 1, SET_CONTROL, API_CTRL_REQUEST, (unsigned char *)p_user_data,
         sizeof(FlightData), 0, 0, 1);
}

void DJI::onboardSDK::CoreAPI::setGimbalAngle(GimbalAngleData *p_user_data)
{
    send(0, 1, SET_CONTROL, API_GIMBAL_CTRL_ANGLE_REQUEST,
         (unsigned char *)p_user_data, sizeof(GimbalAngleData), 0, 0, 1);
}

void DJI::onboardSDK::CoreAPI::setGimbalSpeed(GimbalSpeedData *p_user_data)
{
    send(0, 1, SET_CONTROL, API_GIMBAL_CTRL_SPEED_REQUEST,
         (unsigned char *)p_user_data, sizeof(GimbalSpeedData), 0, 0, 1);
}

void DJI::onboardSDK::CoreAPI::setCamera(DJI::onboardSDK::CAMERA camera_cmd)
{
    unsigned char send_data = 0;
    send(0, 1, SET_CONTROL, camera_cmd, (unsigned char *)&send_data,
         sizeof(send_data), 0, 0, 0);
}

DJI::onboardSDK::HardDriver *DJI::onboardSDK::CoreAPI::getDriver() const
{
    return driver;
}

void DJI::onboardSDK::CoreAPI::setDriver(HardDriver *value) { driver = value; }

void DJI::onboardSDK::CoreAPI::taskCallback(DJI::onboardSDK::CoreAPI *This,
                                            Header *header)
{
    unsigned short ack_data;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        API_STATUS("task running successfully,%d\n", ack_data);
        //! @todo

        if (This->taskResult)
            This->taskResult(ack_data);
    }
    else
    {
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);
    }
}

void DJI::onboardSDK::CoreAPI::getVersionCallback(
    DJI::onboardSDK::CoreAPI *This, Header *header)
{
    unsigned char *ptemp = ((unsigned char *)header) + sizeof(Header);
    char *ptemp2;
    int count = 31;
    VersionData *p_version_data = &This->versionData;

    p_version_data->version_ack = ptemp[0] + (ptemp[1] << 8);
    ptemp += 2;
    p_version_data->version_crc =
        ptemp[0] + (ptemp[1] << 8) + (ptemp[2] << 16) + (ptemp[3] << 24);
    ptemp += 4;
    ptemp2 = p_version_data->version_name;
    while (*ptemp && count)
    {
        *ptemp2++ = (char)*ptemp++;
        count--;
    }
    *ptemp2 = 0;

    //! @todo
    API_STATUS("version ack=%d\n", p_version_data->version_ack);
    API_STATUS("version crc=0x%X\n", p_version_data->version_crc);
    API_STATUS("version name=%s\n", p_version_data->version_name);
}

void DJI::onboardSDK::CoreAPI::activateCallback(DJI::onboardSDK::CoreAPI *This,
                                                Header *header)
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

                This->driver->lockMSG();
                This->broadcastData.activation = 1;
                This->driver->freeMSG();

                if (This->accountData.app_key)
                    This->setKey(This->accountData.app_key);
            }
            else
            {
                API_ERROR("activate code:0x%X\n", ack_data);

                This->driver->lockMSG();
                This->broadcastData.activation = 0;
                This->driver->freeMSG();
            }
            if (This->activateResult)
            {
                This->activateResult(ack_data);
            }
        }
    }
    else
    {
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);
    }
}

void DJI::onboardSDK::CoreAPI::sendToMobileCallback(
    DJI::onboardSDK::CoreAPI *This, Header *header)
{
    unsigned short ack_data = 0xFFFF;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (This->toMobileResult)
            This->toMobileResult(ack_data);
    }
    else
        API_ERROR("ACK is exception,seesion id %d,sequence %d\n",
                  header->sessionID, header->sequence_number);
}

void DJI::onboardSDK::CoreAPI::setControlCallback(CoreAPI *This, Header *header)
{
    unsigned short ack_data = 0xFFFF;
    if (header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,
               ((unsigned char *)header) + sizeof(Header),
               (header->length - EXC_DATA_SIZE));
        if (This->setControlResult)
            This->setControlResult(ack_data);
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
            break;
        case 0x00C9:
            API_STATUS("IOC mode opening can not obtain control\n");
            break;
        default:
            API_STATUS("there is unkown error,ack=0x%X\n", ack_data);
            break;
    }
}

CoreAPI *Flight::getApi() const { return api; }

void Flight::setApi(CoreAPI *value) { api = value; }

CoreAPI *Camera::getApi() const { return api; }

void Camera::setApi(CoreAPI *value) { api = value; }

CoreAPI *Mission::getApi() const { return api; }

void Mission::setApi(CoreAPI *value) { api = value; }
