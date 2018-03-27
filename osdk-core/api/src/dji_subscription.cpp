/** @file dji_subscription.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Telemetry Subscription API for DJI OSDK library
 *
 *  @Copyright (c) 2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "dji_subscription.hpp"
#include "dji_vehicle.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
const uint8_t  ADD_PACKAEG_DATA_LENGTH = 200;
const uint32_t DBVersion               = 0x00000100;
//
// @note: make sure the order of entry is the same as in the enum TopicName
// definition
//
// clang-format off
TopicInfo Telemetry::TopicDataBase[] =
{  // Topic Name ,                     UID,
  {TOPIC_QUATERNION                , UID_QUATERNION               , sizeof(TypeMap<TOPIC_QUATERNION              >::type), 200 ,   0,  255,  0},
  {TOPIC_ACCELERATION_GROUND       , UID_ACCELERATION_GROUND      , sizeof(TypeMap<TOPIC_ACCELERATION_GROUND     >::type), 200 ,   0,  255,  0},
  {TOPIC_ACCELERATION_BODY         , UID_ACCELERATION_BODY        , sizeof(TypeMap<TOPIC_ACCELERATION_BODY       >::type), 200 ,   0,  255,  0},
  {TOPIC_ACCELERATION_RAW          , UID_ACCELERATION_RAW         , sizeof(TypeMap<TOPIC_ACCELERATION_RAW        >::type), 400 ,   0,  255,  0},
  {TOPIC_VELOCITY                  , UID_VELOCITY                 , sizeof(TypeMap<TOPIC_VELOCITY                >::type), 200 ,   0,  255,  0},
  {TOPIC_ANGULAR_RATE_FUSIONED     , UID_ANGULAR_RATE_FUSIONED    , sizeof(TypeMap<TOPIC_ANGULAR_RATE_FUSIONED   >::type), 200 ,   0,  255,  0},
  {TOPIC_ANGULAR_RATE_RAW          , UID_ANGULAR_RATE_RAW         , sizeof(TypeMap<TOPIC_ANGULAR_RATE_RAW        >::type), 400 ,   0,  255,  0},
  {TOPIC_ALTITUDE_FUSIONED         , UID_ALTITUDE_FUSIONED        , sizeof(TypeMap<TOPIC_ALTITUDE_FUSIONED       >::type), 200 ,   0,  255,  0},
  {TOPIC_ALTITUDE_BAROMETER        , UID_ALTITUDE_BAROMETER       , sizeof(TypeMap<TOPIC_ALTITUDE_BAROMETER      >::type), 200 ,   0,  255,  0},
  {TOPIC_HEIGHT_HOMEPOINT          , UID_HEIGHT_HOMEPOINT         , sizeof(TypeMap<TOPIC_HEIGHT_HOMEPOINT        >::type), 1   ,   0,  255,  0},
  {TOPIC_HEIGHT_FUSION             , UID_HEIGHT_FUSION            , sizeof(TypeMap<TOPIC_HEIGHT_FUSION           >::type), 100 ,   0,  255,  0},
  {TOPIC_GPS_FUSED                 , UID_GPS_FUSED                , sizeof(TypeMap<TOPIC_GPS_FUSED               >::type), 50  ,   0,  255,  0},
  {TOPIC_GPS_DATE                  , UID_GPS_DATE                 , sizeof(TypeMap<TOPIC_GPS_DATE                >::type), 50  ,   0,  255,  0},
  {TOPIC_GPS_TIME                  , UID_GPS_TIME                 , sizeof(TypeMap<TOPIC_GPS_TIME                >::type), 50  ,   0,  255,  0},
  {TOPIC_GPS_POSITION              , UID_GPS_POSITION             , sizeof(TypeMap<TOPIC_GPS_POSITION            >::type), 50  ,   0,  255,  0},
  {TOPIC_GPS_VELOCITY              , UID_GPS_VELOCITY             , sizeof(TypeMap<TOPIC_GPS_VELOCITY            >::type), 50  ,   0,  255,  0},
  {TOPIC_GPS_DETAILS               , UID_GPS_DETAILS              , sizeof(TypeMap<TOPIC_GPS_DETAILS             >::type), 50  ,   0,  255,  0},
  {TOPIC_RTK_POSITION              , UID_RTK_POSITION             , sizeof(TypeMap<TOPIC_RTK_POSITION            >::type), 50  ,   0,  255,  0},
  {TOPIC_RTK_VELOCITY              , UID_RTK_VELOCITY             , sizeof(TypeMap<TOPIC_RTK_VELOCITY            >::type), 50  ,   0,  255,  0},
  {TOPIC_RTK_YAW                   , UID_RTK_YAW                  , sizeof(TypeMap<TOPIC_RTK_YAW                 >::type), 50  ,   0,  255,  0},
  {TOPIC_RTK_POSITION_INFO         , UID_RTK_POSITION_INFO        , sizeof(TypeMap<TOPIC_RTK_POSITION_INFO       >::type), 50  ,   0,  255,  0},
  {TOPIC_RTK_YAW_INFO              , UID_RTK_YAW_INFO             , sizeof(TypeMap<TOPIC_RTK_YAW_INFO            >::type), 50  ,   0,  255,  0},
  {TOPIC_COMPASS                   , UID_COMPASS                  , sizeof(TypeMap<TOPIC_COMPASS                 >::type), 100 ,   0,  255,  0},
  {TOPIC_RC                        , UID_RC                       , sizeof(TypeMap<TOPIC_RC                      >::type), 50  ,   0,  255,  0},
  {TOPIC_GIMBAL_ANGLES             , UID_GIMBAL_ANGLES            , sizeof(TypeMap<TOPIC_GIMBAL_ANGLES           >::type), 50  ,   0,  255,  0},
  {TOPIC_GIMBAL_STATUS             , UID_GIMBAL_STATUS            , sizeof(TypeMap<TOPIC_GIMBAL_STATUS           >::type), 50  ,   0,  255,  0},
  {TOPIC_STATUS_FLIGHT             , UID_STATUS_FLIGHT            , sizeof(TypeMap<TOPIC_STATUS_FLIGHT           >::type), 50  ,   0,  255,  0},
  {TOPIC_STATUS_DISPLAYMODE        , UID_STATUS_DISPLAYMODE       , sizeof(TypeMap<TOPIC_STATUS_DISPLAYMODE      >::type), 50  ,   0,  255,  0},
  {TOPIC_STATUS_LANDINGGEAR        , UID_STATUS_LANDINGGEAR       , sizeof(TypeMap<TOPIC_STATUS_LANDINGGEAR      >::type), 50  ,   0,  255,  0},
  {TOPIC_STATUS_MOTOR_START_ERROR  , UID_STATUS_MOTOR_START_ERROR , sizeof(TypeMap<TOPIC_STATUS_MOTOR_START_ERROR>::type), 50  ,   0,  255,  0},
  {TOPIC_BATTERY_INFO              , UID_BATTERY_INFO             , sizeof(TypeMap<TOPIC_BATTERY_INFO            >::type), 50  ,   0,  255,  0},
  {TOPIC_CONTROL_DEVICE            , UID_CONTROL_DEVICE           , sizeof(TypeMap<TOPIC_CONTROL_DEVICE          >::type), 50  ,   0,  255,  0},
  {TOPIC_HARD_SYNC                 , UID_HARD_SYNC                , sizeof(TypeMap<TOPIC_HARD_SYNC               >::type), 400 ,   0,  255,  0},
  {TOPIC_GPS_SIGNAL_LEVEL          , UID_GPS_SIGNAL_LEVEL         , sizeof(TypeMap<TOPIC_GPS_SIGNAL_LEVEL        >::type), 50 ,    0,  255,  0},
  {TOPIC_GPS_CONTROL_LEVEL         , UID_GPS_CONTROL_LEVEL        , sizeof(TypeMap<TOPIC_GPS_CONTROL_LEVEL       >::type), 50 ,    0,  255,  0}
};
// clang-format on

/*!
 * @details 1. Initialize the api member
 *          2. Set each package[i] entry with packageID = i
 *          3. Set the decodeCallback function
 */
DataSubscription::DataSubscription(Vehicle* vehiclePtr)
  : vehicle(vehiclePtr)
  , protocol(vehicle->protocolLayer)
{
  for (int i = 0; i < MAX_NUMBER_OF_PACKAGE; i++)
  {
    package[i].setPackageID(i);
  }

  subscriptionDataDecodeHandler.callback = decodeCallback;
  subscriptionDataDecodeHandler.userData = this;
}

DataSubscription::~DataSubscription()
{
  subscriptionDataDecodeHandler.callback = 0;
  subscriptionDataDecodeHandler.userData = 0;
}

Vehicle*
DataSubscription::getVehicle()
{
  return vehicle;
}

/*!
 * @details:  decodeCallback is a static function and cannot access object
 * member.
 *            In order to access members, it needs a pointer to the
 * subscription.
 */
void
DataSubscription::decodeCallback(Vehicle*      vehiclePtr,
                                 RecvContainer rcvContainer, UserData subPtr)
{
  DataSubscription* subscriptionHandle = (DataSubscription*)subPtr;

  // uint8_t pkgID = *(((uint8_t *)header) + sizeof(OpenHeader) + 2);
  uint8_t pkgID = rcvContainer.recvData.subscribeACK;

  if (pkgID >= MAX_NUMBER_OF_PACKAGE)
  {
    DERROR("Unexpected package id %d received.", pkgID);
    return;
  }

  SubscriptionPackage* p = &subscriptionHandle->package[pkgID];

  /*
   *  TODO: handle the case that the FC is already sending subscription packages
   * when the program starts,
   */

  subscriptionHandle->extractOnePackage(&rcvContainer, p);

  VehicleCallBackHandler h = p->getUnpackHandler();
  if (NULL != h.callback)
  {
    (*(h.callback))(vehiclePtr, rcvContainer, h.userData);
  }
}

/*!
 * @details Setup members of package[packageID]
 *          Do basic gate keeping. No api->send call involved
 */
bool
DataSubscription::initPackageFromTopicList(int packageID, int numberOfTopics,
                                           TopicName* topicList,
                                           bool sendTimeStamp, uint16_t freq)
{
  if (package[packageID].isOccupied())
  {
    DERROR("package [%d] is being occupied.\n", packageID);
    return false;
  }

  package[packageID].setConfig(sendTimeStamp ? 1 : 0);
  return package[packageID].setTopicList(topicList, numberOfTopics, freq);
}

void
DataSubscription::registerUserPackageUnpackCallback(
  int packageID, VehicleCallBack userFunctionAfterPackageExtraction,
  UserData userData)
{
  package[packageID].setUserUnpackCallback(userFunctionAfterPackageExtraction,
                                           userData);
}

//bool
//DataSubscription::pausePackage(int packageID)
//{
//  return true;
//}
//
//bool
//DataSubscription::resumePackage(int packageID)
//{
//  return true;
//}

void
DataSubscription::verify()
{
  uint32_t data = DBVersion;

  int cbIndex                           = vehicle->callbackIdIndex();
  vehicle->nbCallbackFunctions[cbIndex] = (void*)verifyCallback;
  vehicle->nbUserData[cbIndex]          = NULL;

  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::versionMatch, &data,
                 sizeof(data), 500, 2, true, cbIndex);
}

void
DataSubscription::verifyCallback(Vehicle*      vehiclePtr,
                                 RecvContainer rcvContainer, UserData userData)
{
  ACK::ErrorCode ackErrorCode;
  ackErrorCode.info = rcvContainer.recvInfo;
  ackErrorCode.data = rcvContainer.recvData.subscribeACK;

  if (!ACK::getError(ackErrorCode))
  {
    DSTATUS("Verify subscription successful.");
//    subscribPtr->verifySuccessful = true;
  }
  else
  {
    ACK::getErrorCodeMessage(ackErrorCode, __func__);
  }
}

ACK::ErrorCode
DataSubscription::verify(int timeout)
{
  ACK::ErrorCode ack;
  uint32_t       data = DBVersion;

  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::versionMatch, &data,
                 sizeof(data), 500, 2, NULL, 0);

  ack = *((ACK::ErrorCode*)getVehicle()->waitForACK(
    OpenProtocolCMD::CMDSet::Subscribe::versionMatch, timeout));

  if (!ACK::getError(ack))
  {
    DSTATUS("Verify subscription successful.");
//    verifySuccessful = true;
  }
  else
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }
  return ack;
}

void
DataSubscription::startPackage(int packageID)
{
  // We need to prevent running startPackage multiple times
  // The reason is that allocateDataBuffer will delete and reallocating the
  // memory
  // During this period, data will be copied to illegal memory
  if (package[packageID].isOccupied())
  {
    DERROR("Cannot start package [%d] which "
           "is being occupied. Call "
           "removePackage first.",
           packageID);
    return;
  }

  uint8_t buffer[ADD_PACKAEG_DATA_LENGTH];

  int bufferLength = package[packageID].serializePackageInfo(buffer);
  package[packageID].allocateDataBuffer();

  // Register Callback
  int cbIndex = vehicle->callbackIdIndex();
  vehicle->nbCallbackFunctions[cbIndex] =
    (void*)DataSubscription::addPackageCallback;
  vehicle->nbUserData[cbIndex] = &package[packageID];

  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::addPackage, buffer,
                 bufferLength, 500, 1, true, cbIndex);
}

void
DataSubscription::addPackageCallback(Vehicle*      vehiclePtr,
                                     RecvContainer rcvContainer,
                                     UserData      pkgHandle)
{
  // First, check ACK value
  SubscriptionPackage* packageHandle = (SubscriptionPackage*)pkgHandle;

  ACK::ErrorCode ackErrorCode;
  ackErrorCode.info = rcvContainer.recvInfo;
  ackErrorCode.data = rcvContainer.recvData.subscribeACK;

  DSTATUS("Start package %d result: %d.", packageHandle->getInfo().packageID,
          ackErrorCode.data);
  DSTATUS("Package %d info: freq=%d, nTopics=%d.",
          packageHandle->getInfo().packageID, packageHandle->getInfo().freq,
          packageHandle->getInfo().numberOfTopics);
  if (!ACK::getError(ackErrorCode))
  {
    packageHandle->packageAddSuccessHandler();
  }
  else
  {
    ACK::getErrorCodeMessage(ackErrorCode, __func__);
  }
}

ACK::ErrorCode
DataSubscription::startPackage(int packageID, int timeout)
{
  ACK::ErrorCode ack;

  // We need to prevent running startPackage multiple times
  // The reason is that allocateDataBuffer will delete and reallocating the
  // memory
  // During this period, data will be copied to illegal memory
  if (package[packageID].isOccupied())
  {
    DERROR("Cannot start package [%d] which "
           "is being occupied. Call "
           "removePackage first.",
           packageID);

    ack.info.cmd_set = OpenProtocolCMD::CMDSet::subscribe;

    // @TODO: the SUBSCRIBER_MULTIPLE_SUBSCRIBE is not returned from FC, we may
    // need to distinguish between "short circuit return" from "round trip
    // return"
    ack.data = OpenProtocolCMD::ErrorCode::SubscribeACK::MULTIPLE_SUBSCRIBE;
    return ack;
  }

  uint8_t buffer[ADD_PACKAEG_DATA_LENGTH];

  int bufferLength = package[packageID].serializePackageInfo(buffer);
  package[packageID].allocateDataBuffer();

  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::addPackage, buffer,
                 bufferLength, 500, 1, NULL, 0);

  ack = *((ACK::ErrorCode*)getVehicle()->waitForACK(
    OpenProtocolCMD::CMDSet::Subscribe::addPackage, timeout));

  DSTATUS("Start package %d result: %d.",
          package[packageID].getInfo().packageID, ack.data);
  DSTATUS("Package %d info: freq=%d, nTopics=%d.\n",
          package[packageID].getInfo().packageID,
          package[packageID].getInfo().freq,
          package[packageID].getInfo().numberOfTopics);

  if (!ACK::getError(ack))
  {
    package[packageID].packageAddSuccessHandler();
  }
  else
  {
    // TODO Remove. User should do it on the application side
    ACK::getErrorCodeMessage(ack, __func__);
  }

  return ack;
}

// adapted from DataSubscribe::Package::unpack
void
DataSubscription::extractOnePackage(RecvContainer*       pRcvContainer,
                                    SubscriptionPackage* pkg)
{
  //  uint8_t *data = ((uint8_t *)header) + sizeof(OpenHeader) + 2;
  //  DDEBUG(
  //          "%d unpacking %d %d 0x%x 0x%x.", pkg->getBufferSize(),
  //          header->length - CoreAPI::PackageMin - 3, *((uint8_t *)data + 1),
  //          *((uint32_t *)data), *((uint32_t *)data + 1));
  //  data++;

  uint8_t* data = pRcvContainer->recvData.raw_ack_array;
  data++; // skip the package ID

  /*
   * TODO: Handle the time stamp field if it exists
   */

  protocol->getThreadHandle()->lockMSG();
  if (pkg->getDataBuffer())
  {
    // TODO: the length needs to come from the header, not package
    memcpy(pkg->getDataBuffer(), data, pkg->getBufferSize());
    // memcpy(pkg->getDataBuffer(), data, header->length - CoreAPI::PackageMin -
    // 3);
  }
  else
  {
    // This happens when the FC is not power-cycled
    if(!(pkg->hasLeftOverData()))
    {
      pkg->setLeftOverDataFlag(true);
      DDEBUG("Detected telemetry data in package %d before subscribing to it.",pkg->getInfo().packageID);
      DDEBUG("This was due to unclean quit of the program without restarting the drone.\n");
    }
  }
  protocol->getThreadHandle()->freeMSG();
}

void
DataSubscription::removePackage(int packageID)
{
  uint8_t data = packageID;

  int cbIndex = vehicle->callbackIdIndex();
  vehicle->nbCallbackFunctions[cbIndex] =
    (void*)DataSubscription::removePackageCallback;
  vehicle->nbUserData[cbIndex] = &package[packageID];

  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::removePackage, &data,
                 sizeof(data), 500, 1, true, cbIndex);
}

void
DataSubscription::removePackageCallback(Vehicle*      vehiclePtr,
                                        RecvContainer rcvContainer,
                                        UserData      pkgHandle)
{
  SubscriptionPackage* packageHandle = (SubscriptionPackage*)pkgHandle;

  uint8_t packageID = packageHandle->getInfo().packageID;

  ACK::ErrorCode ackErrorCode;
  ackErrorCode.info = rcvContainer.recvInfo;
  ackErrorCode.data = rcvContainer.recvData.subscribeACK;

  if (!ACK::getError(ackErrorCode))
  {
    DSTATUS("Remove package %d successful.", packageID);
    packageHandle->packageRemoveSuccessHandler();
    if(packageHandle->hasLeftOverData())
    {
      packageHandle->setLeftOverDataFlag(false);
    }
  }
  else
  {
    ACK::getErrorCodeMessage(ackErrorCode, __func__);
  }
}

ACK::ErrorCode
DataSubscription::removePackage(int packageID, int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        data = packageID;

  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::removePackage, &data,
                 sizeof(data), 500, 1, NULL, 0);

  ack = *((ACK::ErrorCode*)getVehicle()->waitForACK(
    OpenProtocolCMD::CMDSet::Subscribe::removePackage, timeout));

  if (!ACK::getError(ack))
  {
    DSTATUS("Remove package %d successful.", packageID);
    package[packageID].packageRemoveSuccessHandler();
    if(package[packageID].hasLeftOverData())
    {
      package[packageID].setLeftOverDataFlag(false);
    }
  }
  else
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }

  return ack;
}

void DataSubscription::removeLeftOverPackages()
{
  ACK::ErrorCode ack;
  for(int retry = 0; retry <=3; retry++)
  {
    for(int packageID = 0; packageID < MAX_NUMBER_OF_PACKAGE; packageID++)
    {
      if(package[packageID].hasLeftOverData())
      {
        if(retry == 3)
        {
          DERROR("Package %d was not properly removed due to unclean quit. Please power cycle the drone...", packageID);
        }
        ack = removePackage(packageID, 1);
        if(!ACK::getError(ack))
        {
          DERROR("failed to remove package %d", packageID);
        }
      }
    }
  }
}

void DataSubscription::removeAllExistingPackages()
{
  ACK::ErrorCode ack;
  for(int packageID=0; packageID<MAX_NUMBER_OF_PACKAGE; packageID++)
  {
    if(package[packageID].hasLeftOverData() || package[packageID].isOccupied())
    {
      ack = removePackage(packageID, 1);
      if(!ACK::getError(ack))
      {
        DERROR("failed to remove package %d", packageID);
      }
    }
  }
}

void DataSubscription::reset()
{
  uint8_t data = 0;
  int cbIndex = vehicle->callbackIdIndex();
  vehicle->nbCallbackFunctions[cbIndex] =
          (void*)DataSubscription::resetCallback;
  vehicle->nbUserData[cbIndex] = NULL;

  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::reset, &data,
                 sizeof(data), 500, 1, true, cbIndex);
}

void
DataSubscription::resetCallback(Vehicle*      vehiclePtr,
                                RecvContainer rcvContainer,
                                UserData      pkgHandle)
{
  ACK::ErrorCode ackErrorCode;
  ackErrorCode.info = rcvContainer.recvInfo;
  ackErrorCode.data = rcvContainer.recvData.subscribeACK;

  if (!ACK::getError(ackErrorCode))
  {
    DSTATUS("Reset Subscription Successful.");
  }
  else
  {
    ACK::getErrorCodeMessage(ackErrorCode, __func__);
  }
}
ACK::ErrorCode
DataSubscription::reset(int timeout)
{
  uint8_t data = 0;
  protocol->send(2, vehicle->getEncryption(),
                 OpenProtocolCMD::CMDSet::Subscribe::reset, &data,
                 sizeof(data), 500, 1, NULL, 0);
  ACK::ErrorCode ack;

  ack = *((ACK::ErrorCode*)getVehicle()->waitForACK(
          OpenProtocolCMD::CMDSet::Subscribe::reset, timeout));

  if (!ACK::getError(ack))
  {
    DSTATUS("Reset Subscription Successful.\n");
  }
  else
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }

  return ack;
}

//////////////////////
SubscriptionPackage::SubscriptionPackage()
  : occupied(false)
  , leftOverDataFlag(false)
  , incomingDataBuffer(NULL)
  , packageDataSize(0)
{
  userUnpackHandler.callback = NULL;
  userUnpackHandler.userData = NULL;
}

SubscriptionPackage::~SubscriptionPackage()
{
  cleanUpPackage();
}

void
SubscriptionPackage::setPackageID(uint8_t id)
{
  info.packageID = id;
}

void
SubscriptionPackage::setConfig(uint8_t config)
{
  info.config = config;
}

bool
SubscriptionPackage::isOccupied()
{
  return occupied;
}

void
SubscriptionPackage::setOccupied(bool status)
{
  occupied = status;
}

bool
SubscriptionPackage::hasLeftOverData()
{
  return leftOverDataFlag;
}

void
SubscriptionPackage::setLeftOverDataFlag(bool flag)
{
  leftOverDataFlag = flag;
}

/*
 * Fill in the necessary information for ADD_PACKAGE call
 */
bool
SubscriptionPackage::setTopicList(TopicName* topics, int numberOfTopics,
                                  uint16_t freq)
{
  if (isOccupied())
  {
    return false;
  }
  // It's not simply copy entries of topics to topicList. Two Details needs to
  // be taken care of:
  // 1. Make sure the frequency is valid, i.e., less than the max frequency for
  // each topic
  // 2. The total data payload does not exceed limit
  int totalSize = (info.config == 1) ? 8 : 0;
  for (int i = 0; i < numberOfTopics; i++)
  {
    if (TopicDataBase[topics[i]].maxFreq < freq)
    {
      DDEBUG("Requesting Frequency %d, Max Frequency %d\n", freq,
             TopicDataBase[topics[i]].maxFreq);
      return false;
    }
    totalSize += TopicDataBase[topics[i]].size;
    if (totalSize > ADD_PACKAEG_DATA_LENGTH)
    {
      DDEBUG(
        "Too many topics, data payload of the first %d topic is already %d", i,
        totalSize);
      return false;
    }
  }

  // After passing the above 2 checks, we are safe to fill the data fields
  packageDataSize     = (info.config == 1) ? 8 : 0;
  info.numberOfTopics = numberOfTopics;
  info.freq           = freq;

  for (int i = 0; i < numberOfTopics; i++)
  {
    offsetList[i] = packageDataSize;
    topicList[i]  = topics[i];
    uidList[i]    = TopicDataBase[topicList[i]].uid;
    packageDataSize += TopicDataBase[topicList[i]].size;
  }
  return true;
}

void
SubscriptionPackage::allocateDataBuffer()
{
  if (incomingDataBuffer)
  {
    delete[] incomingDataBuffer;
    incomingDataBuffer = NULL;
  }

  incomingDataBuffer = new uint8_t[packageDataSize];
}

void
SubscriptionPackage::cleanUpPackage()
{
  info.freq           = 0;
  info.config         = 0;
  info.numberOfTopics = 0;

  memset(uidList, 0xFF, sizeof(uidList));
  memset(topicList, 0xFF, sizeof(topicList));
  memset(offsetList, 0, sizeof(offsetList));

  packageDataSize            = 0;
  userUnpackHandler.callback = NULL;
  userUnpackHandler.userData = NULL;
  clearDataBuffer();
}

void
SubscriptionPackage::clearDataBuffer()
{
  if (incomingDataBuffer)
  {
    delete[] incomingDataBuffer;
    incomingDataBuffer = NULL;
  }
}

int
SubscriptionPackage::serializePackageInfo(uint8_t* buffer)
{
  // First, copy the info part of the package
  memcpy(buffer, (uint8_t*)&info, sizeof(info));

  // Next copy the uid list
  memcpy(buffer + sizeof(PackageInfo), (uint8_t*)uidList,
         sizeof(uint32_t) * info.numberOfTopics);

  return sizeof(info) + sizeof(uint32_t) * info.numberOfTopics;
}

void
SubscriptionPackage::setUserUnpackCallback(
  VehicleCallBack userFunctionAfterPackageExtraction, UserData userData)
{
  userUnpackHandler.callback = userFunctionAfterPackageExtraction;
  userUnpackHandler.userData = userData;
}

SubscriptionPackage::PackageInfo
SubscriptionPackage::getInfo()
{
  return info;
}

uint32_t*
SubscriptionPackage::getUidList()
{
  return &uidList[0];
}

TopicName*
SubscriptionPackage::getTopicList()
{
  return &topicList[0];
}

uint32_t*
SubscriptionPackage::getOffsetList()
{
  return &offsetList[0];
}

uint8_t*
SubscriptionPackage::getDataBuffer()
{
  return incomingDataBuffer;
}

uint32_t
SubscriptionPackage::getBufferSize()
{
  return packageDataSize;
}

VehicleCallBackHandler
SubscriptionPackage::getUnpackHandler()
{
  return userUnpackHandler;
}

void
SubscriptionPackage::packageAddSuccessHandler()
{
  // In the TopicDataBase, we set the freq, protocoland data pointer for each
  // subscribed topic
  for (size_t i = 0; i < info.numberOfTopics; ++i)
  {
    TopicDataBase[topicList[i]].pkgID = info.packageID;
    TopicDataBase[topicList[i]].freq  = info.freq;

    // The offset already takes time stamp into consideration
    TopicDataBase[topicList[i]].latest = incomingDataBuffer + offsetList[i];
  }

  setOccupied(true);
}

void
SubscriptionPackage::packageRemoveSuccessHandler()
{
  // Clean up
  // Step 1. Clear fields in TopicDataBase
  for (size_t i = 0; i < info.numberOfTopics; ++i)
  {
    TopicDataBase[topicList[i]].freq   = 0;
    TopicDataBase[topicList[i]].pkgID  = 255;  // Set pkgID to invalid
    TopicDataBase[topicList[i]].latest = NULL; // Clear data pointer
  }

  // Step 2. Clean up package content, except packageID
  cleanUpPackage();

  setOccupied(false);
}

// Detailed doxygen comments begin below

/*! @var DJI::OSDK::Telemetry::TopicName DJI::OSDK::Telemetry::TOPIC_QUATERNION
 * | Angle        | Unit | Accuracy   | Notes                                           |
   |--------------|------|------------|-------------------------------------------------|
   | pitch, roll  | deg  | <1         | in non-ahrs mode                                |
   | yaw          | deg  | <3         | in well-calibrated compass with fine aligned    |
   | yaw with rtk | deg  | around 1.2 | in RTK heading fixed mode with 1 meter baseline |
*/

/*! @var DJI::OSDK::Telemetry::TOPIC_VELOCITY
 * | Axis     | Unit | Accuracy                                                                                    |
   |----------|------|---------------------------------------------------------------------------------------------|
   | vgx, vgy | m/s  | Around 5cm/s for GNSS navigation. Around 3cm/s with VO at 1 meter height                    |
   | vgz      | m/s  | 10cm/s only with barometer in steady air. 3cm/s with VO at 1 meter height with 8cm baseline |
 */

/*! @var DJI::OSDK::Telemetry::TOPIC_GPS_FUSED
 *   | Axis | Unit | Position Sensor | Accuracy                                         |
     |------|------|-----------------|--------------------------------------------------|
     | x, y | m    | GPS             | <3m with open sky without multipath              |
     | z    | m    | GPS             | <5m with open sky without multipath              |
     | x, y | m    | RTK             | around 2cm with fine alignment and fix condition |
     | z    | m    | RTK             | around 3cm with fine alignment and fix condition |
 *
 */

/*! @var DJI::OSDK::Telemetry::TOPIC_GIMBAL_ANGLES
 *  Data Accuracy: 0.1 deg in all axes
 */