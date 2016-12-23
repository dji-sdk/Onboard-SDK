/*! @file DJI_App.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Application layer support functionality for DJI onboardSDK library
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include <string.h>
#include <stdio.h>
#include "DJI_App.h"
#include "DJI_API.h"

using namespace DJI;
using namespace DJI::onboardSDK;

inline void passData(uint16_t flag, uint16_t &enable, void *data, unsigned char *buf,
    size_t datalen, size_t &offset)
{
  //! @todo new algorithm
  if ((flag & enable))
  {
    memcpy((unsigned char *)data, (unsigned char *)buf + offset, datalen);
    offset += datalen;
  }
  enable <<= 1;
}

unsigned char getCmdSet(Header *protocolHeader)
{
  unsigned char *ptemp = ((unsigned char *)protocolHeader) + sizeof(Header);
  return *ptemp;
}

unsigned char getCmdCode(Header *protocolHeader)
{
  unsigned char *ptemp = ((unsigned char *)protocolHeader) + sizeof(Header);
  ptemp++;
  return *ptemp;
}

BroadcastData DJI::onboardSDK::CoreAPI::getBroadcastData() const {
  serialDevice->lockMSG();
  BroadcastData data = broadcastData;
  serialDevice->freeMSG();
  return data;
}

BatteryData DJI::onboardSDK::CoreAPI::getBatteryCapacity() const { return getBroadcastData().battery; }

CtrlInfoData DJI::onboardSDK::CoreAPI::getCtrlInfo() const { return getBroadcastData().ctrlInfo; }

void DJI::onboardSDK::CoreAPI::setBroadcastFrameStatus(bool isFrame)
{
  broadcastFrameStatus = isFrame;
}

bool DJI::onboardSDK::CoreAPI::getBroadcastFrameStatus()
{
  return broadcastFrameStatus;
}

#ifdef SDK_DEV
#include "devApp.cpp"
#else
void DJI::onboardSDK::CoreAPI::broadcast(Header *protocolHeader)
{
  unsigned char *pdata = ((unsigned char *)protocolHeader) + sizeof(Header);
  unsigned short *enableFlag;
  serialDevice->lockMSG();
  pdata += 2;
  enableFlag = (unsigned short *)pdata;
  broadcastData.dataFlag = *enableFlag;
  size_t len = MSG_ENABLE_FLAG_LEN;

  //! @warning Change to const (+change interface for passData) in next release
  uint16_t DATA_FLAG = 0x0001;
  //! @todo better algorithm

  /** 
   *@note Write activation status
   *
   * Default value: 0xFF
   * Activation successful: 0x00
   * Activation error codes: 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
   * (see DJI_API.h for detailed description of the error codes)
   */
  broadcastData.activation = ack_activation;

  if (versionData.version != versionM100_23)
    passData(*enableFlag, DATA_FLAG, &broadcastData.timeStamp, pdata, sizeof(TimeStampData),
        len);
  else
    passData(*enableFlag, DATA_FLAG, &broadcastData.timeStamp.time, pdata, sizeof(uint32_t),
        len);

  passData(*enableFlag, DATA_FLAG, &broadcastData.q, pdata, sizeof(QuaternionData), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.a, pdata, sizeof(CommonData), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.v, pdata, sizeof(VelocityData), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.w, pdata, sizeof(CommonData), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.pos, pdata, sizeof(PositionData), len);

  if (versionData.version == versionA3_31 || versionData.version == versionA3_32)
  {
    passData(*enableFlag, DATA_FLAG, &broadcastData.gps, pdata, sizeof(GPSData), len);
    passData(*enableFlag, DATA_FLAG, &broadcastData.rtk, pdata, sizeof(RTKData), len);
    if (((*enableFlag) & 0x0040))
      API_LOG(serialDevice, RTK_LOG, "receive GPS data %llu\n", (unsigned long long)serialDevice->getTimeStamp());
    if (((*enableFlag) & 0x0080))
      API_LOG(serialDevice, RTK_LOG, "receive RTK data %llu\n", (unsigned long long)serialDevice->getTimeStamp());
  }
  passData(*enableFlag, DATA_FLAG, &broadcastData.mag, pdata, sizeof(MagnetData), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.rc, pdata, sizeof(RadioData), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.gimbal, pdata,
      sizeof(GimbalData) - ((versionData.version == versionM100_23) ? 1 : 0), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.status, pdata, sizeof(FlightStatus), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.battery, pdata, sizeof(BatteryData), len);
  passData(*enableFlag, DATA_FLAG, &broadcastData.ctrlInfo, pdata,
      sizeof(CtrlInfoData) - ((versionData.version == versionM100_23) ? 1 : 0), len);
  serialDevice->freeMSG();

  /**
   * Set broadcast frame status
   * @todo Implement proper notification mechanism
   */
  setBroadcastFrameStatus(true);

  if (broadcastCallback.callback)
    broadcastCallback.callback(this, protocolHeader, broadcastCallback.userData);
}
#endif
void DJI::onboardSDK::CoreAPI::recvReqData(Header *protocolHeader)
{
  unsigned char buf[100] = { 0, 0 };

  uint8_t ack = *((unsigned char *)protocolHeader + sizeof(Header) + 2);
  if (getCmdSet(protocolHeader) == SET_BROADCAST)
  {
    switch (getCmdCode(protocolHeader))
    {
      case CODE_BROADCAST:
        broadcast(protocolHeader);
        break;
      case CODE_FROMMOBILE:
        API_LOG(serialDevice, STATUS_LOG, "Receive data from mobile\n");
        if (fromMobileCallback.callback)
        {
          fromMobileCallback.callback(this, protocolHeader, fromMobileCallback.userData);
        }
        else
        {
          parseFromMobileCallback(this, protocolHeader);
        }
        break;
      case CODE_LOSTCTRL:
        API_LOG(serialDevice, STATUS_LOG, "onboardSDK lost control\n");
        Ack param;
        if (protocolHeader->sessionID > 0)
        {
          buf[0] = buf[1] = 0;
          param.sessionID = protocolHeader->sessionID;
          param.seqNum = protocolHeader->sequenceNumber;
          param.encrypt = protocolHeader->enc;
          param.buf = buf;
          param.length = 2;
          ackInterface(&param);
        }
        break;
      case CODE_MISSION:
        //! @todo add mission session decode
        if (missionCallback.callback)
          missionCallback.callback(this, protocolHeader, missionCallback.userData);
        else
        {
          switch (ack)
          {
            case MISSION_MODE_A:
              break;
            case MISSION_WAYPOINT:
              if (wayPointData)
              {
                if (wayPointCallback.callback)
                  wayPointCallback.callback(this, protocolHeader,
                      wayPointCallback.userData);
                else
                  API_LOG(serialDevice, STATUS_LOG, "Mode waypoint \n");
              }
              break;
            case MISSION_HOTPOINT:
              if (hotPointData)
              {
                if (hotPointCallback.callback)
                  hotPointCallback.callback(this, protocolHeader,
                      hotPointCallback.userData);
                else
                  API_LOG(serialDevice, STATUS_LOG, "Mode HP \n");
              }
              break;
            case MISSION_FOLLOW:
              if (followData)
              {
                if (followCallback.callback)
                  followCallback.callback(this, protocolHeader,
                      followCallback.userData);
                else
                  API_LOG(serialDevice, STATUS_LOG, "Mode Follow \n");
              }
              break;
            case MISSION_IOC:
              //! @todo compare IOC with other mission modes comprehensively
              API_LOG(serialDevice, STATUS_LOG, "Mode IOC \n");
              break;
            default:
              API_LOG(serialDevice, ERROR_LOG, "Unknown mission code 0x%X \n", ack);
              break;
          }
        }
        break;
      case CODE_WAYPOINT:
        //! @todo add waypoint session decode
        if (wayPointEventCallback.callback)
          wayPointEventCallback.callback(this, protocolHeader,
              wayPointEventCallback.userData);
        else
          API_LOG(serialDevice, STATUS_LOG, "WAYPOINT DATA");
        break;
      default:
        API_LOG(serialDevice, STATUS_LOG, "Unknown BROADCAST command code\n");
        break;
    }
  }
  else
    API_LOG(serialDevice, DEBUG_LOG, "Received unknown command\n");
  if (recvCallback.callback)
    recvCallback.callback(this, protocolHeader, recvCallback.userData);
}

void CoreAPI::setBroadcastCallback(CallBack userCallback, UserData userData)
{
  broadcastCallback.callback = userCallback;
  broadcastCallback.userData = userData;
}

void CoreAPI::setFromMobileCallback(CallBack userCallback, UserData userData)
{
  fromMobileCallback.callback = userCallback;
  fromMobileCallback.userData = userData;
}
