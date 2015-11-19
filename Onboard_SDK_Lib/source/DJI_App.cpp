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
#include "DJI_App.h"
#include "DJI_API.h"

using namespace DJI::onboardSDK;

inline void passData(uint16_t _flag, uint16_t _enable, void *_data,
                     unsigned char *_buf, size_t _datalen)
{
    if ((_flag & _enable))
    {
        memcpy((unsigned char *)&(_data), (unsigned char *)(_buf) + (_datalen),
               sizeof(_data));
        _datalen += sizeof(_data);
    }
}

unsigned char getCmdSet(Header *header)
{
    unsigned char *ptemp = ((unsigned char *)header) + sizeof(Header);
    return *ptemp;
}

unsigned char getCmdCode(Header *header)
{
    unsigned char *ptemp = ((unsigned char *)header) + sizeof(Header);
    ptemp++;
    return *ptemp;
}

BroadcastData DJI::onboardSDK::API::getBroadcastData() const
{
    return broadcastData;
}

BatteryData DJI::onboardSDK::API::getBatteryCapacity() const
{
    return broadcastData.capacity;
}

QuaternionData DJI::onboardSDK::API::getQuaternion() const
{
    return broadcastData.q;
}

CommonData DJI::onboardSDK::API::getGroundAcc() const
{
    return broadcastData.a;
}

SpeedData DJI::onboardSDK::API::getGroundSpeed() const
{
    return broadcastData.v;
}

CtrlInfoData DJI::onboardSDK::API::getCtrlInfo() const
{
    return broadcastData.ctrl_info;
}

void DJI::onboardSDK::API::broadcast(Header *header)
{
    unsigned char *pdata = ((unsigned char *)header) + sizeof(Header);
    unsigned short *enableFlag;
    unsigned short len = MSG_ENABLE_FLAG_LEN;
    driver->lockMSG();
    pdata += 2;
    enableFlag = (unsigned short *)pdata;
    passData(*enableFlag, ENABLE_MSG_TIME, &broadcastData.timeStamp, pdata,
             len);
    passData(*enableFlag, HAS_Q, &broadcastData.q, pdata, len);
    passData(*enableFlag, HAS_A, &broadcastData.a, pdata, len);
    passData(*enableFlag, HAS_V, &broadcastData.v, pdata, len);
    passData(*enableFlag, HAS_W, &broadcastData.w, pdata, len);
    passData(*enableFlag, HAS_POS, &broadcastData.pos, pdata, len);
    passData(*enableFlag, HAS_MAG, &broadcastData.mag, pdata, len);
    passData(*enableFlag, HAS_RC, &broadcastData.rc, pdata, len);
    passData(*enableFlag, HAS_GIMBAL, &broadcastData.gimbal, pdata, len);
    passData(*enableFlag, HAS_STATUS, &broadcastData.status, pdata, len);
    passData(*enableFlag, HAS_BATTERY, &broadcastData.capacity, pdata, len);
    passData(*enableFlag, HAS_DEVICE, &broadcastData.ctrl_info, pdata, len);
    driver->freeMSG();

    if (broadcastHandler)
        broadcastHandler();

    if (broadcastData.timeStamp % 500 == 0)
        API_DEBUG("time: %d\n", broadcastData.timeStamp);
}

void DJI::onboardSDK::API::recvReqData(Header *header)
{
    unsigned char buf[100] = { 0, 0 };
    unsigned char len = 0;
    switch (header->session_id)
    {
        case 0:
            if (getCmdSet(header) == SET_BROADCAST)
            {
                switch (getCmdCode(header))
                {
                    case CODE_BROADCAST:
                        broadcast(header);
                        break;
                    case CODE_FROMMOBILE:
                        if (transparentHandler)
                        {
                            len = (header->length - EXC_DATA_SIZE - 2) > 100
                                      ? 100
                                      : (header->length - EXC_DATA_SIZE - 2);
                            memcpy(buf, ((unsigned char *)header) +
                                            sizeof(Header) + 2,
                                   len);
                            transparentHandler(buf, len);
                        }
                        break;
                    case CODE_LOSTCTRL:
                        API_STATUS("%s:onboardSDK lost contrl\n", __func__);
                        break;
                    default:
                        API_STATUS(
                            "error, unknown BROADCAST command code 0x%X\n",
                            getCmdCode(header));
                        break;
                }
            }
            else
            {
                API_DEBUG("%s:receive unknown command\n", __func__);
                if (recvHandler)
                {
                    recvHandler(header);
                }
            }
            break;
        case 1:
        case 2:
            API_DEBUG("%s:Recv request,session id=%d,seq_num=%d\n", __func__,
                      header->session_id, header->sequence_number);
            if (recvHandler)
            {
                recvHandler(header);
            }
            else
            {
                Ack param;
                if (header->session_id > 0)
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

void DJI::onboardSDK::API::setTransparentTransmissionCallback(
    TransparentHandler transparentHandlerEntrance)
{
    transparentHandler = transparentHandlerEntrance;
}

void DJI::onboardSDK::API::setBroadcastCallback(
    BroadcastHandler broadcastHandlerEntrance)
{
    broadcastHandler = broadcastHandlerEntrance;
}
