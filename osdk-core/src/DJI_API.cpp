/** @file DJI_API.cpp
 *  @version 3.1.9
 *  @date November 10, 2016
 *
 *  @brief
 *  Core API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All right reserved.
 *
 */

#include "DJI_API.h"
#include <string.h>
#include <stdio.h>

using namespace DJI;
using namespace DJI::onboardSDK;

#ifdef USE_ENCRYPT
uint8_t DJI::onboardSDK::encrypt = 1;
#else
uint8_t DJI::onboardSDK::encrypt = 0;
#endif // USE_ENCRYPT

CoreAPI::CoreAPI(HardDriver *sDevice, Version SDKVersion, bool userCallbackThread,
    CallBack userRecvCallback, UserData userData)
{
  CallBackHandler handler;
  handler.callback = userRecvCallback;
  handler.userData = userData;
  init(sDevice, handler, userCallbackThread, SDKVersion);
}

void CoreAPI::init(HardDriver *sDevice, CallBackHandler userRecvCallback,
    bool userCallbackThread, Version SDKVersion)
{
  serialDevice = sDevice;
  // serialDevice->init();

  seq_num = 0;
  ackFrameStatus = 11;
  broadcastFrameStatus = false;

  filter.recvIndex = 0;
  filter.reuseCount = 0;
  filter.reuseIndex = 0;
  filter.encode = 0;

  broadcastCallback.callback = 0;
  broadcastCallback.userData = 0;
  fromMobileCallback.callback = 0;
  fromMobileCallback.userData = 0;
  hotPointCallback.callback = 0;
  wayPointCallback.callback = 0;
  hotPointCallback.userData = 0;
  wayPointEventCallback.callback = 0;
  wayPointEventCallback.userData = 0;
  wayPointCallback.userData = 0;
  followCallback.callback = 0;
  followCallback.userData = 0;
  missionCallback.callback = 0;
  missionCallback.userData = 0;

  recvCallback.callback = userRecvCallback.callback;
  recvCallback.userData = userRecvCallback.userData;

  callbackThread = false;
  hotPointData = false;
  followData = false;
  wayPointData = false;
  callbackThread = userCallbackThread;

  nonBlockingCBThreadEnable = false;
  ack_data = 99;
  versionData.version = SDKVersion;
  ack_activation = 0xFF;


  //! @todo simplify code above
  serialDevice->lockMSG();
  memset((unsigned char *)&broadcastData, 0, sizeof(broadcastData));
  serialDevice->freeMSG();

  setup();
}

CoreAPI::CoreAPI(HardDriver *sDevice, Version SDKVersion, CallBackHandler userRecvCallback,
    bool userCallbackThread)
{
  init(sDevice, userRecvCallback, userCallbackThread, SDKVersion);
  getSDKVersion();
}

void CoreAPI::send(unsigned char session, unsigned char is_enc, CMD_SET cmdSet,
    unsigned char cmdID, void *pdata, int len, CallBack ackCallback, int timeout,
    int retry)
{
  Command param;
  unsigned char *ptemp = (unsigned char *)encodeSendData;
  *ptemp++ = cmdSet;
  *ptemp++ = cmdID;

  memcpy(encodeSendData + SET_CMD_SIZE, pdata, len);

  param.handler = ackCallback;
  param.sessionMode = session;
  param.length = len + SET_CMD_SIZE;
  param.buf = encodeSendData;
  param.retry = retry;

  param.timeout = timeout;
  param.encrypt = is_enc;

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
  param.sessionMode = session_mode;
  param.length = len + SET_CMD_SIZE;
  param.buf = encodeSendData;
  param.retry = retry_time;

  param.timeout = timeout;
  param.encrypt = is_enc ? 1 : 0;

  param.userData = userData;

  sendInterface(&param);
}

void CoreAPI::send(Command *parameter) { sendInterface(parameter); }

void CoreAPI::ack(req_id_t req_id, unsigned char *ackdata, int len)
{
  Ack param;

  memcpy(encodeACK, ackdata, len);

  param.sessionID = req_id.session_id;
  param.seqNum = req_id.sequence_number;
  param.encrypt = req_id.need_encrypt;
  param.buf = encodeACK;
  param.length = len;

  this->ackInterface(&param);
}

void CoreAPI::getDroneVersion(CallBack callback, UserData userData)
{
  versionData.version_ack = ACK_COMMON_NO_RESPONSE;
  versionData.version_crc = 0x0;
  versionData.version_name[0] = 0;

  unsigned cmd_timeout = 100; // unit is ms
  unsigned retry_time = 3;
  unsigned char cmd_data = 0;

  send(2, 0, SET_ACTIVATION, CODE_GETVERSION, (unsigned char *)&cmd_data, 1, cmd_timeout,
    retry_time, callback ? callback : CoreAPI::getDroneVersionCallback, userData);
}

VersionData CoreAPI::getDroneVersion(int timeout)
{
  versionData.version_ack = ACK_COMMON_NO_RESPONSE;
  versionData.version_crc = 0x0;
  versionData.version_name[0] = 0;

  unsigned cmd_timeout = 100; // unit is ms
  unsigned retry_time = 3;
  unsigned char cmd_data = 0;

  send(2, 0, SET_ACTIVATION, CODE_GETVERSION, (unsigned char *)&cmd_data, 1, cmd_timeout,
    retry_time, 0, 0);

  // Wait for end of ACK frame to arrive
  serialDevice->lockACK();
  serialDevice->wait(timeout);
  serialDevice->freeACK();

  // Parse return value
  unsigned char *ptemp = &(missionACKUnion.droneVersion.ack[0]);

  if(versionData.version != versionA3_32){
    versionData.version_ack = ptemp[0] + (ptemp[1] << 8);
    ptemp += 2;
    versionData.version_crc = ptemp[0] + (ptemp[1] << 8) + (ptemp[2] << 16) + (ptemp[3] << 24);
    ptemp += 4;
    if (versionData.version != versionM100_23)
    {
      memcpy(versionData.version_ID, ptemp, 11);
      ptemp += 11;
    }
    memcpy(versionData.version_name, ptemp, 32);
  }else{
    versionData.version_ack = missionACKUnion.missionACK;
    memcpy(versionData.version_name, "versionA3_32", strlen("versionA3_32")+1);
  }

  return versionData;
}

void CoreAPI::activate(ActivateData *data, CallBack callback, UserData userData)
{
  data->version = versionData.version;
  accountData = *data;
  accountData.reserved = 2;

  for (int i = 0; i < 32; ++i) accountData.iosID[i] = '0'; //! @note for ios verification
  API_LOG(serialDevice, DEBUG_LOG, "version 0x%X/n", versionData.version);
  API_LOG(serialDevice, DEBUG_LOG, "%.32s", accountData.iosID);
  send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&accountData,
    sizeof(accountData) - sizeof(char *), 1000, 3,
    callback ? callback : CoreAPI::activateCallback, userData);

  ack_data = missionACKUnion.simpleACK;
  if(ack_data == ACK_ACTIVE_SUCCESS && accountData.encKey)
   setKey(accountData.encKey);
}

unsigned short CoreAPI::activate(ActivateData *data, int timeout)
{
  data->version = versionData.version;
  accountData = *data;
  accountData.reserved = 2;

  for (int i = 0; i < 32; ++i) accountData.iosID[i] = '0'; //! @note for ios verification
  API_LOG(serialDevice, DEBUG_LOG, "version 0x%X/n", versionData.version);
  API_LOG(serialDevice, DEBUG_LOG, "%.32s", accountData.iosID);
  send(2, 0, SET_ACTIVATION, CODE_ACTIVATE, (unsigned char *)&accountData,
    sizeof(accountData) - sizeof(char *), 1000, 3, 0, 0);

  // Wait for end of ACK frame to arrive
  serialDevice->lockACK();
  serialDevice->wait(timeout);
  serialDevice->freeACK();
  ack_data = missionACKUnion.simpleACK;
  if(ack_data == ACK_ACTIVE_SUCCESS && accountData.encKey)
    setKey(accountData.encKey);

  return ack_data;
}

void CoreAPI::sendToMobile(uint8_t *data, uint8_t len, CallBack callback, UserData userData)
{
  if (len > 100)
  {
    API_LOG(serialDevice, ERROR_LOG, "Too much data to send");
    return;
  }
  send(0, 0, SET_ACTIVATION, CODE_TOMOBILE, data, len, 500, 1,
    callback ? callback : CoreAPI::sendToMobileCallback, userData);
}

void CoreAPI::setBroadcastFreq(uint8_t *dataLenIs16, CallBack callback, UserData userData)
{
  //! @note see also enum BROADCAST_FREQ in DJI_API.h
  for (int i = 0; i < 16; ++i)
  {
    if (versionData.version == versionM100_31)
      if (i < 12)
      {
        dataLenIs16[i] = (dataLenIs16[i] > 5 ? 5 : dataLenIs16[i]);
      }
      else
        dataLenIs16[i] = 0;
    else
    {
      if (i < 14)
      {
        dataLenIs16[i] = (dataLenIs16[i] > 5 ? 5 : dataLenIs16[i]);
      }
      else
        dataLenIs16[i] = 0;
    }
  }
  send(2, 0, SET_ACTIVATION, CODE_FREQUENCY, dataLenIs16, 16, 100, 1,
     callback ? callback : CoreAPI::setFrequencyCallback, userData);
}

unsigned short CoreAPI::setBroadcastFreq(uint8_t *dataLenIs16, int timeout)
{
  //! @note see also enum BROADCAST_FREQ in DJI_API.h
  for (int i = 0; i < 16; ++i)
  {
    if (versionData.version == versionM100_31)
      if (i < 12)
      {
        dataLenIs16[i] = (dataLenIs16[i] > 5 ? 5 : dataLenIs16[i]);
      }
      else
        dataLenIs16[i] = 0;
    else
    {
      if (i < 14)
      {
        dataLenIs16[i] = (dataLenIs16[i] > 5 ? 5 : dataLenIs16[i]);
      }
      else
        dataLenIs16[i] = 0;
    }
  }
  send(2, 0, SET_ACTIVATION, CODE_FREQUENCY, dataLenIs16, 16, 100, 1, 0, 0);

  // Wait for end of ACK frame to arrive
  serialDevice->lockACK();
  serialDevice->wait(timeout);
  serialDevice->freeACK();
  return missionACKUnion.simpleACK;
}

void CoreAPI::setBroadcastFreqDefaults()
{
  uint8_t freq[16];

 /* Channels definition:
  * M100:
  * 0 - Timestamp
  * 1 - Attitude Quaterniouns
  * 2 - Acceleration
  * 3 - Velocity (Ground Frame)
  * 4 - Angular Velocity (Body Frame)
  * 5 - Position
  * 6 - Magnetometer
  * 7 - RC Channels Data
  * 8 - Gimbal Data
  * 9 - Flight Status
  * 10 - Battery Level
  * 11 - Control Information
  *
  * A3:
  * 0 - Timestamp
  * 1 - Attitude Quaterniouns
  * 2 - Acceleration
  * 3 - Velocity (Ground Frame)
  * 4 - Angular Velocity (Body Frame)
  * 5 - Position
  * 6 - GPS Detailed Information
  * 7 - RTK Detailed Information
  * 8 - Magnetometer
  * 9 - RC Channels Data
  * 10 - Gimbal Data
  * 11 - Flight Status
  * 12 - Battery Level
  * 13 - Control Information
  *
  */

  if (versionData.version == versionM100_31 || versionData.version == versionM100_23) {
    freq[0] = BROADCAST_FREQ_1HZ;
    freq[1] = BROADCAST_FREQ_10HZ;
    freq[2] = BROADCAST_FREQ_50HZ;
    freq[3] = BROADCAST_FREQ_100HZ;
    freq[4] = BROADCAST_FREQ_50HZ;
    freq[5] = BROADCAST_FREQ_10HZ;
    freq[6] = BROADCAST_FREQ_1HZ;
    freq[7] = BROADCAST_FREQ_10HZ;
    freq[8] = BROADCAST_FREQ_50HZ;
    freq[9] = BROADCAST_FREQ_100HZ;
    freq[10] = BROADCAST_FREQ_50HZ;
    freq[11] = BROADCAST_FREQ_10HZ;
  }
  else if (versionData.version == versionA3_31 || versionData.version == versionA3_32) {
    freq[0] = BROADCAST_FREQ_1HZ;
    freq[1] = BROADCAST_FREQ_10HZ;
    freq[2] = BROADCAST_FREQ_50HZ;
    freq[3] = BROADCAST_FREQ_100HZ;
    freq[4] = BROADCAST_FREQ_50HZ;
    freq[5] = BROADCAST_FREQ_10HZ;
    freq[6] = BROADCAST_FREQ_0HZ;
    freq[7] = BROADCAST_FREQ_0HZ;
    freq[8] = BROADCAST_FREQ_1HZ;
    freq[9] = BROADCAST_FREQ_10HZ;
    freq[10] = BROADCAST_FREQ_50HZ;
    freq[11] = BROADCAST_FREQ_100HZ;
    freq[12] = BROADCAST_FREQ_50HZ;
    freq[13] = BROADCAST_FREQ_10HZ;
  }
  setBroadcastFreq(freq);
}

void CoreAPI::setBroadcastFreqToZero()
{
  uint8_t freq[16];

  /* Channels definition:
   * M100:
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - Magnetometer
   * 7 - RC Channels Data
   * 8 - Gimbal Data
   * 9 - Flight Status
   * 10 - Battery Level
   * 11 - Control Information
   *
   * A3:
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   *
   */

  freq[0] = BROADCAST_FREQ_0HZ;
  freq[1] = BROADCAST_FREQ_0HZ;
  freq[2] = BROADCAST_FREQ_0HZ;
  freq[3] = BROADCAST_FREQ_0HZ;
  freq[4] = BROADCAST_FREQ_0HZ;
  freq[5] = BROADCAST_FREQ_0HZ;
  freq[6] = BROADCAST_FREQ_0HZ;
  freq[7] = BROADCAST_FREQ_0HZ;
  freq[8] = BROADCAST_FREQ_0HZ;
  freq[9] = BROADCAST_FREQ_0HZ;
  freq[10] = BROADCAST_FREQ_0HZ;
  freq[11] = BROADCAST_FREQ_0HZ;
  if(versionData.version == versionA3_31 || versionData.version == versionA3_32)
  {
    freq[12] = BROADCAST_FREQ_0HZ;
    freq[13] = BROADCAST_FREQ_0HZ;
  }
  setBroadcastFreq(freq);
}


unsigned short CoreAPI::setBroadcastFreqDefaults(int timeout)
{
  uint8_t freq[16];

  /* Channels definition:
   * M100:
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - Magnetometer
   * 7 - RC Channels Data
   * 8 - Gimbal Data
   * 9 - Flight Status
   * 10 - Battery Level
   * 11 - Control Information
   *
   * A3:
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Status
   * 12 - Battery Level
   * 13 - Control Information
   *
   */

  if (versionData.version == versionM100_31 || versionData.version == versionM100_23) {
    freq[0] = BROADCAST_FREQ_1HZ;
    freq[1] = BROADCAST_FREQ_10HZ;
    freq[2] = BROADCAST_FREQ_50HZ;
    freq[3] = BROADCAST_FREQ_100HZ;
    freq[4] = BROADCAST_FREQ_50HZ;
    freq[5] = BROADCAST_FREQ_10HZ;
    freq[6] = BROADCAST_FREQ_1HZ;
    freq[7] = BROADCAST_FREQ_10HZ;
    freq[8] = BROADCAST_FREQ_50HZ;
    freq[9] = BROADCAST_FREQ_100HZ;
    freq[10] = BROADCAST_FREQ_50HZ;
    freq[11] = BROADCAST_FREQ_10HZ;
  }
  else if (versionData.version == versionA3_31 || versionData.version == versionA3_32) {
    freq[0] = BROADCAST_FREQ_1HZ;
    freq[1] = BROADCAST_FREQ_10HZ;
    freq[2] = BROADCAST_FREQ_50HZ;
    freq[3] = BROADCAST_FREQ_100HZ;
    freq[4] = BROADCAST_FREQ_50HZ;
    freq[5] = BROADCAST_FREQ_10HZ;
    freq[6] = BROADCAST_FREQ_0HZ;
    freq[7] = BROADCAST_FREQ_0HZ;
    freq[8] = BROADCAST_FREQ_1HZ;
    freq[9] = BROADCAST_FREQ_10HZ;
    freq[10] = BROADCAST_FREQ_50HZ;
    freq[11] = BROADCAST_FREQ_100HZ;
    freq[12] = BROADCAST_FREQ_50HZ;
    freq[13] = BROADCAST_FREQ_10HZ;
  }

  return setBroadcastFreq(freq, timeout);
}

TimeStampData CoreAPI::getTime() const { return getBroadcastData().timeStamp; }

FlightStatus CoreAPI::getFlightStatus() const { return getBroadcastData().status; }

void CoreAPI::setFromMobileCallback(CallBackHandler FromMobileEntrance)
{
  fromMobileCallback = FromMobileEntrance;
}


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
  send(2, DJI::onboardSDK::encrypt, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2,
    callback ? callback : CoreAPI::setControlCallback, userData);
}

unsigned short CoreAPI::setControl(bool enable, int timeout)
{
  unsigned char data = enable ? 1 : 0;
  send(2, DJI::onboardSDK::encrypt, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500, 2, 0, 0);

  // Wait for end of ACK frame to arrive
  serialDevice->lockACK();
  serialDevice->wait(timeout);
  serialDevice->freeACK();

  if (missionACKUnion.simpleACK == ACK_SETCONTROL_ERROR_MODE)
  {
    if(versionData.version != versionA3_32)
      missionACKUnion.simpleACK = ACK_SETCONTROL_NEED_MODE_F;
    else
      missionACKUnion.simpleACK = ACK_SETCONTROL_NEED_MODE_P;
  }

  return missionACKUnion.simpleACK;
}

HardDriver *CoreAPI::getDriver() const { return serialDevice; }

SimpleACK CoreAPI::getSimpleACK () const { return missionACKUnion.simpleACK; }

void CoreAPI::setDriver(HardDriver *sDevice) { serialDevice = sDevice; }

void CoreAPI::getDroneVersionCallback(CoreAPI *api, Header *protocolHeader, UserData userData __UNUSED)
{
  unsigned char *ptemp = ((unsigned char *)protocolHeader) + sizeof(Header);
  size_t ack_length = protocolHeader->length - EXC_DATA_SIZE;

  if(ack_length > 1){
    api->versionData.version_ack = ptemp[0] + (ptemp[1] << 8);
    ptemp += 2;
    api->versionData.version_crc =
      ptemp[0] + (ptemp[1] << 8) + (ptemp[2] << 16) + (ptemp[3] << 24);
    ptemp += 4;
    if (api->versionData.version != versionM100_23)
    {
      memcpy(api->versionData.version_ID, ptemp, 11);
      ptemp += 11;
    }
    memcpy(api->versionData.version_name, ptemp, 32);
  }else{
    api->versionData.version_ack = ptemp[0];
    memcpy(api->versionData.version_name, "versionA3_32", strlen("versionA3_32")+1);
  }

  API_LOG(api->serialDevice, STATUS_LOG, "version ack = %d\n", api->versionData.version_ack);

  if(api->versionData.version != versionA3_32){
    API_LOG(api->serialDevice, STATUS_LOG, "version crc = 0x%X\n", api->versionData.version_crc);
    if (api->versionData.version != versionM100_23)
      API_LOG(api->serialDevice, STATUS_LOG, "version ID = %.11s\n", api->versionData.version_ID);
  }

  API_LOG(api->serialDevice, STATUS_LOG, "version name = %.32s\n",
      api->versionData.version_name);
}

void CoreAPI::activateCallback(CoreAPI *api, Header *protocolHeader, UserData userData __UNUSED)
{

  unsigned short ack_data;
  if (protocolHeader->length - EXC_DATA_SIZE <= 2)
  {
    memcpy((unsigned char *)&ack_data, ((unsigned char *)protocolHeader) + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
    
    // Write activation status to the broadcast data
    api->setBroadcastActivation(ack_data);

    switch (ack_data)
    {
      case ACK_ACTIVE_SUCCESS:
        API_LOG(api->serialDevice, STATUS_LOG, "Activated successfully\n");

        if (api->accountData.encKey)
          api->setKey(api->accountData.encKey);
        return;
      case ACK_ACTIVE_NEW_DEVICE:
        API_LOG(api->serialDevice, STATUS_LOG, "New device, please link DJIGO to your "
            "remote controller and try again\n");
        break;
      case ACK_ACTIVE_PARAMETER_ERROR:
        API_LOG(api->serialDevice, ERROR_LOG, "Wrong parameter\n");
        break;
      case ACK_ACTIVE_ENCODE_ERROR:
        API_LOG(api->serialDevice, ERROR_LOG, "Encode error\n");
        break;
      case ACK_ACTIVE_APP_NOT_CONNECTED:
        API_LOG(api->serialDevice, ERROR_LOG, "DJIGO not connected\n");
        break;
      case ACK_ACTIVE_NO_INTERNET:
        API_LOG(api->serialDevice, ERROR_LOG, "DJIGO not "
            "connected to the internet\n");
        break;
      case ACK_ACTIVE_SERVER_REFUSED:
        API_LOG(api->serialDevice, ERROR_LOG, "DJI server rejected "
            "your request, please use your SDK ID\n");
        break;
      case ACK_ACTIVE_ACCESS_LEVEL_ERROR:
        API_LOG(api->serialDevice, ERROR_LOG, "Wrong SDK permission\n");
        break;
      case ACK_ACTIVE_VERSION_ERROR:
        API_LOG(api->serialDevice, ERROR_LOG, "SDK version did not match\n");
        api->getDroneVersion();
        break;
      default:
        if (!api->decodeACKStatus(ack_data))
        {
          API_LOG(api->serialDevice, ERROR_LOG, "While calling this function");
        }
        break;
    }
  }
  else
  {
    API_LOG(api->serialDevice, ERROR_LOG, "ACK is exception, session id %d,sequence %d\n",
        protocolHeader->sessionID, protocolHeader->sequenceNumber);
  }
}

void CoreAPI::sendToMobileCallback(CoreAPI *api, Header *protocolHeader, UserData userData __UNUSED)
{
  unsigned short ack_data = ACK_COMMON_NO_RESPONSE;
  if (protocolHeader->length - EXC_DATA_SIZE <= 2)
  {
    memcpy((unsigned char *)&ack_data, ((unsigned char *)protocolHeader) + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
    if (!api->decodeACKStatus(ack_data))
    {
      API_LOG(api->serialDevice, ERROR_LOG, "While calling this function");
    }
  }
  else
  {
    API_LOG(api->serialDevice, ERROR_LOG, "ACK is exception, session id %d,sequence %d\n",
        protocolHeader->sessionID, protocolHeader->sequenceNumber);
  }
}

//! Mobile Data Transparent Transmission Input Servicing 
void CoreAPI::parseFromMobileCallback(CoreAPI *api, Header *protocolHeader, UserData userData __UNUSED)
{
  uint16_t mobile_data_id;
  
  if (protocolHeader->length - EXC_DATA_SIZE <= 4)
  {
    mobile_data_id = *((unsigned char*)protocolHeader + sizeof(Header) + 2);

    switch (mobile_data_id)
    {
      case 2: 
        if (obtainControlMobileCallback.callback)
        {
          obtainControlMobileCallback.callback(api, protocolHeader, obtainControlMobileCallback.userData);          
        }
        else
        {
          obtainControlMobileCMD = true; 
        }
        break;

      case 3: 
        if (releaseControlMobileCallback.callback)
        {
          releaseControlMobileCallback.callback(api, protocolHeader, releaseControlMobileCallback.userData);          
        }
        else
        {
          releaseControlMobileCMD = true;
        }
        break;

      case 4: 
        if (activateMobileCallback.callback)
        {
          activateMobileCallback.callback(api, protocolHeader, activateMobileCallback.userData);          
        }
        else
        {
          activateMobileCMD = true;
        }
        break;

      case 5: 
        if (armMobileCallback.callback)
        {
          armMobileCallback.callback(api, protocolHeader, armMobileCallback.userData);
        }
        else
        {
          armMobileCMD = true;
        }
        break;

      case 6: 
        if (disArmMobileCallback.callback)
        {
          disArmMobileCallback.callback(api, protocolHeader, disArmMobileCallback.userData);     
        }
        else
        {
          disArmMobileCMD = true;
        }
        break;

      case 7: 
        if (takeOffMobileCallback.callback)
        {
          takeOffMobileCallback.callback(api, protocolHeader, takeOffMobileCallback.userData);   
        }
        else
        {
          takeOffMobileCMD = true;
        }
        break;

      case 8: 
        if (landingMobileCallback.callback)
        {
          landingMobileCallback.callback(api, protocolHeader, landingMobileCallback.userData);  
        }
        else
        {
          landingMobileCMD = true;
        }
        break;

      case 9: 
        if (goHomeMobileCallback.callback)
        {
          goHomeMobileCallback.callback(api, protocolHeader, goHomeMobileCallback.userData); 
        }
        else
        {
          goHomeMobileCMD = true;
        }
        break;

      case 10: 
        if (takePhotoMobileCallback.callback)
        {
          takePhotoMobileCallback.callback(api, protocolHeader, takePhotoMobileCallback.userData);     
        }
        else
        {
          takePhotoMobileCMD = true;
        }
        break;

      case 11: 
        if (startVideoMobileCallback.callback)
        {
          startVideoMobileCallback.callback(api, protocolHeader, startVideoMobileCallback.userData);   
        }
        else
        {
          startVideoMobileCMD = true;
        }
        break;

      case 13: 
        if (stopVideoMobileCallback.callback)
        {
          stopVideoMobileCallback.callback(api, protocolHeader, stopVideoMobileCallback.userData); 
        }
        else
        {
          stopVideoMobileCMD = true;
        }
        break;
      //! Advanced features: LiDAR Mapping, Collision Avoidance, Precision Missions
      case 20:
          startLASMapLoggingCMD = true;
        break;
      case 21:
          stopLASMapLoggingCMD = true;
        break;
      case 24:
          precisionMissionCMD = true;
        break;
      case 25:
          precisionMissionsCollisionAvoidanceCMD = true;
        break;
      case 26:
          precisionMissionsLidarMappingCMD = true;
        break;
      case 27:
          precisionMissionsCollisionAvoidanceLidarMappingCMD = true;
        break;

      //! The next few are only polling based and do not use callbacks. See usage in Linux Sample.
      case 61:
        drawCirMobileCMD = true;
        break;
      case 62:
        drawSqrMobileCMD = true;
        break;
      case 63:
        attiCtrlMobileCMD = true;
        break;
      case 64:
        gimbalCtrlMobileCMD = true;
        break;
      case 65:
        wayPointTestMobileCMD = true;
        break;
      case 66:
        localNavTestMobileCMD = true;
        break;
      case 67:
        globalNavTestMobileCMD = true;
        break;
      case 68:
        VRCTestMobileCMD = true;
        break;
      case 69:
        precisionMissionCMD = true;
        break;
    }
  }
}

void CoreAPI::setFrequencyCallback(CoreAPI *api __UNUSED, Header *protocolHeader,
    UserData userData __UNUSED)
{
  unsigned short ack_data = ACK_COMMON_NO_RESPONSE;

  if (protocolHeader->length - EXC_DATA_SIZE <= 2)
  {
    memcpy((unsigned char *)&ack_data, ((unsigned char *)protocolHeader) + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
  }
  switch (ack_data)
  {
    case 0x0000:
      API_LOG(api->serialDevice, STATUS_LOG, "Frequency set successfully\n");
      break;
    case 0x0001:
      API_LOG(api->serialDevice, ERROR_LOG, "Frequency parameter error\n");
      break;
    default:
      if (!api->decodeACKStatus(ack_data))
      {
        API_LOG(api->serialDevice, ERROR_LOG, "While calling this function\n");
      }
      break;
  }
}

Version CoreAPI::getSDKVersion() const { return versionData.version; }

SDKFilter CoreAPI::getFilter() const { return filter; }

void CoreAPI::setVersion(const Version &value) { versionData.version = value; }

void CoreAPI::setControlCallback(CoreAPI *api, Header *protocolHeader, UserData userData __UNUSED)
{
  unsigned short ack_data = ACK_COMMON_NO_RESPONSE;
  unsigned char data = 0x1;

  if (protocolHeader->length - EXC_DATA_SIZE <= sizeof(ack_data))
  {
    memcpy((unsigned char *)&ack_data, ((unsigned char *)protocolHeader) + sizeof(Header),
        (protocolHeader->length - EXC_DATA_SIZE));
  }
  else
  {
    API_LOG(api->serialDevice, ERROR_LOG, "ACK is exception, session id %d,sequence %d\n",
        protocolHeader->sessionID, protocolHeader->sequenceNumber);
  }

  switch (ack_data)
  {
    case ACK_SETCONTROL_ERROR_MODE:
      if(api->versionData.version != versionA3_32)
      {
        API_LOG(api->serialDevice, STATUS_LOG, "Obtain control failed: switch to F mode\n");
      }
      else
      {
	API_LOG(api->serialDevice, STATUS_LOG, "Obtain control failed: switch to P mode\n");
      }
      break;
    case ACK_SETCONTROL_RELEASE_SUCCESS:
      API_LOG(api->serialDevice, STATUS_LOG, "Released control successfully\n");
      break;
    case ACK_SETCONTROL_OBTAIN_SUCCESS:
      API_LOG(api->serialDevice, STATUS_LOG, "Obtained control successfully\n");
      break;
    case ACK_SETCONTROL_OBTAIN_RUNNING:
      API_LOG(api->serialDevice, STATUS_LOG, "Obtain control running\n");
      api->send(2, DJI::onboardSDK::encrypt, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500,
          2, CoreAPI::setControlCallback);
      break;
    case ACK_SETCONTROL_RELEASE_RUNNING:
      API_LOG(api->serialDevice, STATUS_LOG, "Release control running\n");
      data = 0;
      api->send(2, DJI::onboardSDK::encrypt, SET_CONTROL, CODE_SETCONTROL, &data, 1, 500,
          2, CoreAPI::setControlCallback);
      break;
    case ACK_SETCONTROL_IOC:
      API_LOG(api->serialDevice, STATUS_LOG, "IOC mode opening can not obtain control\n");
      break;
    default:
      if (!api->decodeACKStatus(ack_data))
      {
        API_LOG(api->serialDevice, ERROR_LOG, "While calling this function");
      }
      break;
  }
}
