#include "DJI_API.h"
#include <string.h>
#include <stdio.h>

DJI::onboardSDK::API *defaultAPI;

DJI::onboardSDK::API::API(DJI::onboardSDK::HardDriver *Driver,ReceiveHandler user_cmd_handler_entrance)
{
    driver = Driver;
    status_ctrl_cmd_data = {0,0};
    seq_num = 0;
    filter = { 0 };
    taskResult = 0;
    activateResult = 0;
    toMobileResult = 0;
    setControlResult = 0;
    broadcastHandler = 0;
    transparentHandler = 0;
    recvHandler = user_cmd_handler_entrance ? user_cmd_handler_entrance : 0;

    setup();
}

void DJI::onboardSDK::API::send(unsigned char session_mode, unsigned char is_enc, CMD_SET cmd_set, unsigned char cmd_id,
                                unsigned char *pdata, int len, CallBack ack_callback, int timeout, int retry_time)
{
    Command param;
    unsigned char *ptemp = (unsigned char *)Pro_Encode_Data;
    *ptemp++ = cmd_set;
    *ptemp++ = cmd_id;

    memcpy(Pro_Encode_Data + SET_CMD_SIZE,pdata,len);

    param.ack_callback = ack_callback;
    param.session_mode = session_mode;
    param.length = len + SET_CMD_SIZE;
    param.buf = Pro_Encode_Data;
    param.retry_time = retry_time;

    param.ack_timeout = timeout;
    param.need_encrypt = is_enc;

    sendInterface(&param);
}

void DJI::onboardSDK::API::send(Command *parameter)
{
    sendInterface(parameter);
}

void DJI::onboardSDK::API::ack(req_id_t req_id, unsigned char *ackdata, int len)
{
    Ack param;

    memcpy(Pro_Encode_ACK,ackdata,len);

    param.session_id = req_id.session_id;
    param.seq_num = req_id.sequence_number;
    param.need_encrypt = req_id.need_encrypt;
    param.buf = Pro_Encode_ACK;
    param.length = len;

    this->ackInterface(&param);
}

int DJI::onboardSDK::API::task(TASK taskname, CommandResult user_notice_entrance)
{
    unsigned char cur_cmd = 0;
    if(taskname != 1 && taskname != 4 && taskname != 6)
    {
        return -1;
    }
    taskResult = user_notice_entrance;
    cur_cmd = taskname;

    status_ctrl_cmd_data.cmd_data = cur_cmd;
    status_ctrl_cmd_data.cmd_sequence ++;

    send(2,1,SET_CONTROL, API_CMD_REQUEST,(unsigned char*)&status_ctrl_cmd_data,
         sizeof(status_ctrl_cmd_data),DJI::onboardSDK::API::taskCallback,100, 3);

    return 0;
}

int DJI::onboardSDK::API::getVersion(Get_API_Version_Notify user_notice_entrance)
{
    to_user_version_data.version_ack = 0xFFFF;
    to_user_version_data.version_crc = 0x0;
    to_user_version_data.version_name[0] = 0;

    unsigned cmd_timeout = 100;    //unit is ms
    unsigned retry_time = 3;
    unsigned char cmd_data = 0;

    send(2,1,SET_ACTIVATION, API_VER_QUERY,(unsigned char*)&cmd_data,
         1,DJI::onboardSDK::API::getVersionCallback,cmd_timeout, retry_time);

    return 0;
}

int DJI::onboardSDK::API::activate(ActivateData_t *p_user_data, CommandResult user_notice_entrance)
{
    activateResult = user_notice_entrance ? user_notice_entrance : 0;
    accountData = *p_user_data;

    send( 2, 0, SET_ACTIVATION, API_USER_ACTIVATION,
          (unsigned char*)&accountData,
          sizeof(accountData) - sizeof(char *),
          DJI::onboardSDK::API::activateCallback,1000,3);
    return 0;
}

int DJI::onboardSDK::API::sendToMobile(unsigned char *data, unsigned char len, CommandResult user_notice_entrance)
{
    if(len > 100)
    {
        return -1;
    }
    toMobileResult = user_notice_entrance ? user_notice_entrance : 0;
    send(2, 0, SET_ACTIVATION, API_TRANSPARENT_DATA_TO_MOBILE,
         data,len,DJI::onboardSDK::API::sendToMobileCallback,500,2);

    return 0;
}

int DJI::onboardSDK::API::setControl(unsigned char cmd, CommandResult user_notice_entrance)
{
    unsigned char data = cmd & 0x1;
    setControlResult = user_notice_entrance ? user_notice_entrance : 0;
    send(2,1, SET_CONTROL, API_CTRL_MANAGEMENT,
         &data,1,DJI::onboardSDK::API::setControlCallback,500,1);
    return 0;
}

int DJI::onboardSDK::API::setAttitude(AttitudeData_t *p_user_data)
{
    send(0,1, SET_CONTROL, API_CTRL_REQUEST,
         (unsigned char *)p_user_data,sizeof(AttitudeData_t),
         0,0,1);
    return 0;
}

int DJI::onboardSDK::API::setGimbalAngle(GimbalAngleData_t *p_user_data)
{
    send(0,1, SET_CONTROL, API_GIMBAL_CTRL_ANGLE_REQUEST,
         (unsigned char *)p_user_data,sizeof(GimbalAngleData_t),
         0,0,1);
    return 0;
}

int DJI::onboardSDK::API::setGimbalSpeed(GimbalSpeedData_t *p_user_data)
{
    send(0,1, SET_CONTROL, API_GIMBAL_CTRL_SPEED_REQUEST,
         (unsigned char *)p_user_data,sizeof(GimbalSpeedData_t),
         0,0,1);
    return 0;
}

int DJI::onboardSDK::API::setCamera(DJI::onboardSDK::CAMERA camera_cmd)
{
    unsigned char send_data = 0;

    if(camera_cmd != API_CAMERA_SHOT && camera_cmd != API_CAMERA_VIDEO_START
            && camera_cmd != API_CAMERA_VIDEO_STOP)
    {
        API_ERROR("%s,line %d,Param ERROR\n",__func__,__LINE__);
        return -1;
    }

    send(0,1, SET_CONTROL, camera_cmd,
         (unsigned char*)&send_data,sizeof(send_data),0,0,0);

    return 0;
}

DJI::onboardSDK::HardDriver *DJI::onboardSDK::API::getDriver() const
{
    return driver;
}

void DJI::onboardSDK::API::setDriver(HardDriver *value)
{
    driver = value;
}


void DJI::onboardSDK::API::taskCallback(DJI::onboardSDK::API *This, Header *header)
{
    unsigned short ack_data;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,((unsigned char *)header)+sizeof(Header), (header->length - EXC_DATA_SIZE));
        API_STATUS("%s,task running successfully,%d\n",__func__,ack_data);
        //!@ todo
    }
    else
    {
        API_ERROR("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

void DJI::onboardSDK::API::getVersionCallback(DJI::onboardSDK::API *This, Header *header)
{
    unsigned char *ptemp =((unsigned char *)header)+sizeof(Header);
    char *ptemp2;
    int count = 31;
    version_query_data_t *p_version_data = &This->to_user_version_data;

    p_version_data->version_ack = ptemp[0] + (ptemp[1] << 8);
    ptemp += 2;
    p_version_data->version_crc = ptemp[0] + (ptemp[1] << 8) +
            (ptemp[2] << 16) + (ptemp[3] << 24);
    ptemp += 4;
    ptemp2 = p_version_data->version_name;
    while(*ptemp && count)
    {
        *ptemp2 ++ = (char)*ptemp ++;
        count --;
    }
    *ptemp2 = 0;

    API_STATUS("%s,version ack=%d\n",__func__,p_version_data->version_ack);
    API_STATUS("%s,version crc=0x%X\n",__func__,p_version_data->version_crc);
    API_STATUS("%s,version name=%s\n",__func__,p_version_data->version_name);
}

void DJI::onboardSDK::API::activateCallback(DJI::onboardSDK::API *This, Header *header)
{
    //printf("%s: Entery",__func__);
    volatile unsigned short ack_data;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,((unsigned char *)header)+sizeof(Header), (header->length - EXC_DATA_SIZE));
        if(ack_data == SDK_ACTIVATE_NEW_DEVICE)
        {
            API_STATUS("new device try again \n");
        }
        else
        {
            if(ack_data == SDK_ACTIVATE_SUCCESS)
            {
                API_STATUS("Activation Successfully\n");

                This->driver->lockMSG();
                This->broadcastData.activation= 1;
                This->driver->freeMSG();

                if(This->accountData.app_key)
                    This->setKey(This->accountData.app_key);
            }
            else
            {
                API_STATUS("%s,line %d,activate ERR code:0x%X\n",__func__,__LINE__,ack_data);

                This->driver->lockMSG();
                This->broadcastData.activation= 0;
                This->driver->freeMSG();

            }
            if(This->activateResult)
            {
                This->activateResult(ack_data);
            }
        }

    }
    else
    {
        API_ERROR("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

void DJI::onboardSDK::API::sendToMobileCallback(DJI::onboardSDK::API *This, Header *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,((unsigned char *)header)+sizeof(Header), (header->length - EXC_DATA_SIZE));
        if(This->toMobileResult)
            This->toMobileResult(ack_data);
    }
    else
    {
        API_ERROR("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }
}

void DJI::onboardSDK::API::setControlCallback(API* This,Header *header)
{
    unsigned short ack_data = 0xFFFF;
    if(header->length - EXC_DATA_SIZE <= 2)
    {
        memcpy((unsigned char *)&ack_data,((unsigned char *)header)+sizeof(Header), (header->length - EXC_DATA_SIZE));
        if(This->setControlResult)
            This->setControlResult(ack_data);

    }
    else
    {
        API_ERROR("%s,line %d:ERROR,ACK is exception,seesion id %d,sequence %d\n",
               __func__,__LINE__,header->session_id,header->sequence_number);
    }

    switch(ack_data)
    {
    case 0x0000:
        API_STATUS("%s,line %d, obtain control failed, Conditions did not satisfied",__func__,__LINE__);
    case 0x0001:
        API_STATUS("%s,line %d, release control successfully\n",__func__,__LINE__);
        break;
    case 0x0002:
        API_STATUS("%s,line %d, obtain control successfully\n",__func__,__LINE__);
        break;
    case 0x0003:
        API_STATUS("%s,line %d, obtain control running\n",__func__,__LINE__);
        break;
    case 0x0004:
        API_STATUS("control request already done");
        break;
    default:
        API_STATUS("%s,line %d, there is unkown error,ack=0x%X\n",__func__,__LINE__,ack_data);
        break;
    }
}

