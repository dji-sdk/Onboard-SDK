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

inline void passData(uint16_t flag, uint16_t enable, void *data,
                     unsigned char *buf, size_t datalen, size_t &offset)
{
    //! @todo new algorithm
    if ((flag & enable))
    {
        memcpy((unsigned char *)data, (unsigned char *)buf + offset, datalen);
        offset += datalen;
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

BroadcastData DJI::onboardSDK::CoreAPI::getBroadcastData() const
{
    return broadcastData;
}

BatteryData DJI::onboardSDK::CoreAPI::getBatteryCapacity() const
{
    return broadcastData.capacity;
}

CtrlInfoData DJI::onboardSDK::CoreAPI::getCtrlInfo() const
{
    return broadcastData.ctrl_info;
}

void DJI::onboardSDK::CoreAPI::broadcast(Header *header)
{
    unsigned char *pdata = ((unsigned char *)header) + sizeof(Header);
    unsigned short *enableFlag;
    driver->lockMSG();
    pdata += 2;
    enableFlag = (unsigned short *)pdata;
    size_t len = MSG_ENABLE_FLAG_LEN;

    passData(*enableFlag, HAS_TIME, &broadcastData.timeStamp, pdata,
             sizeof(TimeStampData), len);
    passData(*enableFlag, HAS_Q, &broadcastData.q, pdata,
             sizeof(QuaternionData), len);
    passData(*enableFlag, HAS_A, &broadcastData.a, pdata, sizeof(CommonData),
             len);
    passData(*enableFlag, HAS_V, &broadcastData.v, pdata, sizeof(VelocityData),
             len);
    passData(*enableFlag, HAS_W, &broadcastData.w, pdata, sizeof(CommonData),
             len);
    passData(*enableFlag, HAS_POS, &broadcastData.pos, pdata,
             sizeof(PossitionData), len);
    passData(*enableFlag, HAS_MAG, &broadcastData.mag, pdata,
             sizeof(MagnetData), len);
    passData(*enableFlag, HAS_RC, &broadcastData.rc, pdata, sizeof(RadioData),
             len);
    passData(*enableFlag, HAS_GIMBAL, &broadcastData.gimbal, pdata,
             sizeof(GimbalData), len);
    passData(*enableFlag, HAS_STATUS, &broadcastData.status, pdata,
             sizeof(uint8_t), len);
    passData(*enableFlag, HAS_BATTERY, &broadcastData.capacity, pdata,
             sizeof(BatteryData), len);
    passData(*enableFlag, HAS_DEVICE, &broadcastData.ctrl_info, pdata,
             sizeof(CtrlInfoData), len);

    driver->freeMSG();

    if (broadcastCallback)
        broadcastCallback(this, header);
}

void DJI::onboardSDK::CoreAPI::recvReqData(Header *header)
{
    unsigned char buf[100] = { 0, 0 };
    unsigned char len = 0;

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
                    memcpy(buf, ((unsigned char *)header) + sizeof(Header) + 2,
                           len);
                    transparentHandler(buf, len);
                }
                break;
            case CODE_LOSTCTRL:
                API_LOG(driver, STATUS_LOG, "onboardSDK lost contrl\n");
                Ack param;
                if (header->sessionID > 0)
                {
                    buf[0] = buf[1] = 0;
                    param.session_id = header->sessionID;
                    param.seq_num = header->sequence_number;
                    param.need_encrypt = header->enc_type;
                    param.buf = buf;
                    param.length = 2;
                    ackInterface(&param);
                }
                break;
            case CODE_MISSION:
                //! @todo add mission session decode

                API_LOG(driver, DEBUG_LOG, "%x",
                        *((unsigned char *)header + sizeof(Header) + 2));
                break;
            case CODE_WAYPOINT:
                //! @todo add waypoint session decode
                break;
            default:
                API_LOG(driver, STATUS_LOG,
                        "error, unknown BROADCAST command code\n");
                break;
        }
    }
    else
        API_LOG(driver, DEBUG_LOG, "receive unknown command\n");
    if (recvCallback)
        recvCallback(this, header);
}

void CoreAPI::setTransparentTransmissionCallback(
    TransparentHandler transparentHandlerEntrance)
{
    transparentHandler = transparentHandlerEntrance;
}

void CoreAPI::setBroadcastCallback(CallBack callback)
{
    broadcastCallback = callback;
}
