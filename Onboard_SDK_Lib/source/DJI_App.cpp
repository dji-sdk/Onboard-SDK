/*! @brief
 *  @file DJI_Pro_APP.cpp
 *  @version 1.0
 *
 *  @abstract
 *
 *  @attention
 *  Project configuration:
 *
 *  @version features:
 *  -* @version V3.0
 *  -* DJI-onboard-SDK for QT\STM32\ROS\Cmake
 *  -* @date Nov 11, 2015
 *  -* @author william.wu
 *
 *  -* @version V2.0
 *  -* DJI-onboard-SDK for QT-windows
 *  -* @date Sep 8, 2015
 *  -* @author wuyuwei
 * */

#include <string.h>
#include <stdio.h>
#ifdef __GNUC__
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#endif
#include "DJI_App.h"
#include "DJI_API.h"

using namespace DJI::onboardSDK;

unsigned char getCmdSet(Header *header)
{
    unsigned char *ptemp =((unsigned char *)header)+sizeof(Header);
    return *ptemp;
}

unsigned char getCmdCode(Header *header)
{
    unsigned char *ptemp = ((unsigned char *)header)+sizeof(Header);
    ptemp ++;
    return *ptemp;
}

void DJI::onboardSDK::API::getBroadcastData(BroadcastData_t *p_user_buf) const
{
    driver->lockMSG();
    *p_user_buf = broadcastData;
    driver->freeMSG();
}

void DJI::onboardSDK::API::getBatteryCapacity(unsigned char *data) const
{
    driver->lockMSG();
    *data = broadcastData.battery_remaining_capacity;
    driver->freeMSG();
}

void DJI::onboardSDK::API::getQuaternion(QuaternionData_t *p_user_buf) const
{
    driver->lockMSG();
    *p_user_buf = broadcastData.q;
    driver->freeMSG();
}

void DJI::onboardSDK::API::getGroundAcc(CommonData_t *p_user_buf) const
{
    driver->lockMSG();
    *p_user_buf = broadcastData.a;
    driver->freeMSG();
}

void DJI::onboardSDK::API::getGroundVo(api_vel_data_t *p_user_buf) const
{
    driver->lockMSG();
    *p_user_buf = broadcastData.v;
    driver->freeMSG();
}

void DJI::onboardSDK::API::getCtrlInfo(CtrlInfoData_t *p_user_buf) const
{
    driver->lockMSG();
    *p_user_buf = broadcastData.ctrl_info;
    driver->freeMSG();
}

void DJI::onboardSDK::API::broadcast(Header *header)
{
    unsigned char *pdata = ((unsigned char *)header)+sizeof(Header);
    unsigned short *msg_enable_flag;
    unsigned short data_len = MSG_ENABLE_FLAG_LEN;
    driver->lockMSG();
    pdata += 2;
    msg_enable_flag = (unsigned short *)pdata;
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_TIME,       broadcastData.time_stamp                   , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_Q,          broadcastData.q                            , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_A,          broadcastData.a                            , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_V,          broadcastData.v                            , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_W,          broadcastData.w                            , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_POS,        broadcastData.pos                          , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_MAG,        broadcastData.mag                          , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_RC,         broadcastData.rc                           , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_GIMBAL,     broadcastData.gimbal                       , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_STATUS,     broadcastData.status                       , pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_BATTERY,    broadcastData.battery_remaining_capacity	, pdata, data_len);
    PARSE_STD_MSG( *msg_enable_flag, ENABLE_MSG_DEVICE,     broadcastData.ctrl_info                    , pdata, data_len);
    driver->freeMSG();

    if (broadcastHandler)
        broadcastHandler();

    if(broadcastData.time_stamp%500 == 0)
        API_DEBUG("time: %d\n",broadcastData.time_stamp);
}

void DJI::onboardSDK::API::recvReqData(Header *header)
{
    unsigned char buf[100] = {0,0};
    unsigned char len = 0;
    switch(header->session_id)
    {
    case 0:
        if(getCmdSet(header) == SET_BROADCAST)
        {
            switch(getCmdCode(header))
            {
            case CODE_BROADCAST:
                broadcast(header);
                break;
            case CODE_FROMMOBILE:
                if(transparentHandler)
                {
                    len = (header->length - EXC_DATA_SIZE -2) > 100 ? 100 : (header->length - EXC_DATA_SIZE -2);
                    memcpy(buf,((unsigned char *)header)+sizeof(Header) + 2,len);
                    transparentHandler(buf,len);
                }
                break;
            case CODE_LOSTCTRL:
                API_STATUS("%s:onboardSDK lost contrl\n",__func__);
                break;
            default:
                API_STATUS("error, unknown BROADCAST command code 0x%X\n",getCmdCode(header));
                break;
            }
        }
        else
        {
            API_DEBUG("%s:receive unknown command\n",__func__);
            if(recvHandler)
            {
                recvHandler(header);
            }
        }
        break;
    case 1:
    case 2:
        API_DEBUG("%s:Recv request,session id=%d,seq_num=%d\n",
               __func__,header->session_id,header->sequence_number);
        if(recvHandler)
        {
            recvHandler(header);
        }
        else
        {
            Ack param;
            if(header->session_id > 0)
            {
                buf[0] = buf[1] = 0;
                param.session_id = header->session_id;
                param.seq_num = header->sequence_number;
                param.need_encrypt = header->enc_type;
                param.buf = buf;
                param.length = 2;
                ackInterface(&param);
            }
        }
        break;
    }
}

void DJI::onboardSDK::API::setTransparentTransmissionCallback(TransparentHandler transparentHandlerEntrance)
{
    transparentHandler = transparentHandlerEntrance;
}

void DJI::onboardSDK::API::setBroadcastCallback(BroadcastHandler broadcastHandlerEntrance)
{
    broadcastHandler = broadcastHandlerEntrance;
}
