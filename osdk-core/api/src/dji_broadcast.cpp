/** @file dji_broadcast.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Broadcast Telemetry API for DJI onboardSDK library
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#include "dji_broadcast.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

void
DataBroadcast::unpackCallback(Vehicle* vehicle, RecvContainer recvFrame,
                              UserData data)
{
  DataBroadcast* broadcastPtr = (DataBroadcast*)data;
  broadcastPtr->unpackData(&recvFrame);
  if (broadcastPtr->userCbHandler.callback)
    broadcastPtr->userCbHandler.callback(vehicle, recvFrame,
                                         broadcastPtr->userCbHandler.userData);
}

DataBroadcast::DataBroadcast(Vehicle* vehiclePtr)
{
  if (vehiclePtr)
  {
    setVehicle(vehiclePtr);
  }
  unpackHandler.callback = unpackCallback;
  unpackHandler.userData = this;

  userCbHandler.callback = 0;
  userCbHandler.userData = 0;
}

DataBroadcast::~DataBroadcast()
{
  this->setUserBroadcastCallback(0, NULL);
  unpackHandler.callback = 0;
  unpackHandler.userData = 0;
}

// clang-format off
Telemetry::TimeStamp           DataBroadcast::getTimeStamp()          const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::TimeStamp           ans = timeStamp;  vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::SyncStamp           DataBroadcast::getSyncStamp()          const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::SyncStamp           ans = syncStamp;  vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Quaternion          DataBroadcast::getQuaternion()         const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Quaternion          ans = q;          vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Vector3f            DataBroadcast::getAcceleration()       const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Vector3f            ans = a;          vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Vector3f            DataBroadcast::getVelocity()           const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Vector3f            ans = v;          vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Vector3f            DataBroadcast::getAngularRate()        const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Vector3f            ans = w;          vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::VelocityInfo        DataBroadcast::getVelocityInfo()       const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::VelocityInfo        ans = vi;         vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::GlobalPosition      DataBroadcast::getGlobalPosition()     const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::GlobalPosition      ans = gp;         vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::RelativePosition    DataBroadcast::getRelativePosition()   const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::RelativePosition    ans = rp;         vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::GPSInfo             DataBroadcast::getGPSInfo()            const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::GPSInfo             ans = gps;        vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::RTK                 DataBroadcast::getRTKInfo()            const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::RTK                 ans = rtk;        vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Mag                 DataBroadcast::getMag()                const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Mag                 ans = mag;        vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::RC                  DataBroadcast::getRC()                 const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::RC                  ans = rc;         vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Gimbal              DataBroadcast::getGimbal()             const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Gimbal              ans = gimbal;     vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Status              DataBroadcast::getStatus()             const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Status              ans = status;     vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::Battery             DataBroadcast::getBatteryInfo()        const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::Battery             ans = battery;    vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
Telemetry::SDKInfo             DataBroadcast::getSDKInfo()            const {  vehicle->protocolLayer->getThreadHandle()->lockMSG(); Telemetry::SDKInfo             ans = info;       vehicle->protocolLayer->getThreadHandle()->freeMSG();  return ans;}
// clang-format on

Vehicle*
DataBroadcast::getVehicle() const
{
  return vehicle;
}

void
DataBroadcast::setVehicle(Vehicle* vehiclePtr)
{
  vehicle = vehiclePtr;
}

void
DataBroadcast::setBroadcastFreq(uint8_t* dataLenIs16, VehicleCallBack callback,
                                UserData userData)
{
  //! @note see also enum BROADCAST_FREQ in DJI_API.h
  for (int i = 0; i < 16; ++i)
  {
    dataLenIs16[i] = (dataLenIs16[i] > 7 ? 5 : dataLenIs16[i]);
  }

  uint32_t cmd_timeout = 100; // unit is ms
  uint32_t retry_time  = 1;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)setFrequencyCallback;
    vehicle->nbUserData[cbIndex]          = NULL;
  }

  vehicle->protocolLayer->send(
    2, 0, OpenProtocol::CMDSet::Activation::frequency, dataLenIs16, 16,
    cmd_timeout, retry_time, true, cbIndex);
}

ACK::ErrorCode
DataBroadcast::setBroadcastFreq(uint8_t* dataLenIs16, int timeout)
{
  ACK::ErrorCode ack;

  for (int i = 0; i < 16; ++i)
  {
    dataLenIs16[i] = (dataLenIs16[i] > 7 ? 5 : dataLenIs16[i]);
  }

  vehicle->protocolLayer->send(2, 0,
                               OpenProtocol::CMDSet::Activation::frequency,
                               dataLenIs16, 16, 100, 1, 0, 0);

  ack = *((ACK::ErrorCode*)getVehicle()->waitForACK(
    OpenProtocol::CMDSet::Activation::frequency, timeout));

  return ack;
}

void
DataBroadcast::unpackData(RecvContainer* pRecvFrame)
{
  uint8_t* pdata = pRecvFrame->recvData.raw_ack_array;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  passFlag = *(uint16_t*)pdata;
  pdata += sizeof(uint16_t);
  // clang-format off
  unpackOne(FLAG_TIME        ,&timeStamp ,pdata,sizeof(timeStamp ));
  unpackOne(FLAG_TIME        ,&syncStamp ,pdata,sizeof(syncStamp ));
  unpackOne(FLAG_QUATERNION  ,&q         ,pdata,sizeof(q         ));
  unpackOne(FLAG_ACCELERATION,&a         ,pdata,sizeof(a         ));
  unpackOne(FLAG_VELOCITY    ,&v         ,pdata,sizeof(v         ));
  unpackOne(FLAG_VELOCITY    ,&vi        ,pdata,sizeof(vi        ));
  unpackOne(FLAG_ANGULAR_RATE,&w         ,pdata,sizeof(w         ));
  unpackOne(FLAG_POSITION    ,&gp        ,pdata,sizeof(gp        ));
  unpackOne(FLAG_POSITION    ,&rp        ,pdata,sizeof(rp        ));
  unpackOne(FLAG_GPSINFO     ,&gps       ,pdata,sizeof(gps       ));
  unpackOne(FLAG_RTKINFO     ,&rtk       ,pdata,sizeof(rtk       ));
  unpackOne(FLAG_MAG         ,&mag       ,pdata,sizeof(mag       ));
  unpackOne(FLAG_RC          ,&rc        ,pdata,sizeof(rc        ));
  unpackOne(FLAG_GIMBAL      ,&gimbal    ,pdata,sizeof(gimbal    ));
  unpackOne(FLAG_STATUS      ,&status    ,pdata,sizeof(status    ));
  unpackOne(FLAG_BATTERY     ,&battery   ,pdata,sizeof(battery   ));
  unpackOne(FLAG_DEVICE      ,&info      ,pdata,sizeof(info      ));
  // clang-format on
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
}

void
DataBroadcast::unpackOne(DataBroadcast::FLAG flag, void* data, uint8_t*& buf,
                         size_t size)
{
  if (flag & passFlag)
  {
    memcpy((uint8_t*)data, (uint8_t*)buf, size);
    buf += size;
  }
}

void
DataBroadcast::setVersionDefaults(uint8_t* frequencyBuffer)
{
  if (vehicle->getFwVersion() >= Version::A3_3_2_20_test)
  {
    setFreqDefaults(frequencyBuffer);
  }
  else
  {
    setFreqDefaultsM100_31(frequencyBuffer);
  }
}

void
DataBroadcast::setBroadcastFreqDefaults()
{
  uint8_t frequencyBuffer[16];
  setVersionDefaults(frequencyBuffer);
  setBroadcastFreq(frequencyBuffer);
}

ACK::ErrorCode
DataBroadcast::setBroadcastFreqDefaults(int timeout)
{
  uint8_t frequencyBuffer[16];
  setVersionDefaults(frequencyBuffer);
  return setBroadcastFreq(frequencyBuffer, timeout);
}

void
DataBroadcast::setFreqDefaultsM100_31(uint8_t* freq)
{
  freq[0]  = FREQ_1HZ;
  freq[1]  = FREQ_10HZ;
  freq[2]  = FREQ_50HZ;
  freq[3]  = FREQ_100HZ;
  freq[4]  = FREQ_50HZ;
  freq[5]  = FREQ_10HZ;
  freq[6]  = FREQ_0HZ;
  freq[7]  = FREQ_0HZ;
  freq[8]  = FREQ_1HZ;
  freq[9]  = FREQ_10HZ;
  freq[10] = FREQ_50HZ;
  freq[11] = FREQ_100HZ;
  freq[12] = FREQ_50HZ;
  freq[13] = FREQ_10HZ;
}

void
DataBroadcast::setFreqDefaults(uint8_t* freq)
{
  /* Channels definition:
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - Remote Controller Channel Data
   * 7 - Gimbal Data
   * 8 - Flight Status
   * 9 - Battery Level
   * 10 - Control Information
   */

  freq[0] = FREQ_50HZ;
  freq[1] = FREQ_50HZ;
  freq[2] = FREQ_50HZ;
  freq[3] = FREQ_50HZ;
  freq[4] = FREQ_50HZ;
  freq[5] = FREQ_50HZ;
  /*
   * GPS: DON'T SEND
   */
  freq[6] = FREQ_0HZ;
  /*
   * RTK: DON'T SEND
   */
  freq[7] = FREQ_0HZ;
  /*
   * Magnetometer: DON'T SEND
   */
  freq[8]  = FREQ_0HZ;
  freq[9]  = FREQ_50HZ;
  freq[10] = FREQ_50HZ;
  freq[11] = FREQ_10HZ;
  freq[12] = FREQ_1HZ;
  freq[13] = FREQ_1HZ;
}

void
DataBroadcast::setBroadcastFreqToZero()
{
  uint8_t freq[16];

  freq[0]  = FREQ_0HZ;
  freq[1]  = FREQ_0HZ;
  freq[2]  = FREQ_0HZ;
  freq[3]  = FREQ_0HZ;
  freq[4]  = FREQ_0HZ;
  freq[5]  = FREQ_0HZ;
  freq[6]  = FREQ_0HZ;
  freq[7]  = FREQ_0HZ;
  freq[8]  = FREQ_0HZ;
  freq[9]  = FREQ_0HZ;
  freq[10] = FREQ_0HZ;
  freq[11] = FREQ_0HZ;
  freq[12] = FREQ_0HZ;
  freq[13] = FREQ_0HZ;
  setBroadcastFreq(freq);
}

void
DataBroadcast::setFrequencyCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                    UserData userData)
{

  ACK::ErrorCode ackErrorCode;
  ackErrorCode.data = OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  ackErrorCode.info = recvFrame.recvInfo;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= 2)
  {
    // Two-byte ACK
    ackErrorCode.data = recvFrame.recvData.ack;
  }

  if (!ACK::getError(ackErrorCode))
  {
    ACK::getErrorCodeMessage(ackErrorCode, __func__);
  }
}

void
DataBroadcast::setUserBroadcastCallback(VehicleCallBack callback,
                                        UserData        userData)
{
  userCbHandler.callback = callback;
  userCbHandler.userData = userData;
}

uint16_t
DataBroadcast::getPassFlag()
{
  return passFlag;
}
