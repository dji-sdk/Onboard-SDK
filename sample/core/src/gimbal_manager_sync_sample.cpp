/*! @file gimbal_manager_sync_sample.cpp
 *  @version 4.0 Beta
 *  @date December 12 2019
 *
 *  @brief
 *  Demonstrate how to use the synchronous apis of gimbal manager.
 *
 *  @Copyright (c) 2019 DJI
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

#include "gimbal_manager_sync_sample.hpp"

#define GIMBA_SUB_PACKAGE_INDEX 0

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

GimbalManagerSyncSample::GimbalManagerSyncSample(Vehicle *vehiclePtr)
    : vehicle(vehiclePtr) {
  /*! verify the subscribe function */
  ACK::ErrorCode ack = vehiclePtr->subscribe->verify(1);
  if (ACK::getError(ack) != ACK::SUCCESS) {
    ACK::getErrorCodeMessage(ack, __func__);
  }

  /*! Package 0: Subscribe to gimbal data at freq 50 Hz */
  ACK::ErrorCode subscribeStatus;
  int       pkgIndex        = GIMBA_SUB_PACKAGE_INDEX;
  int       freq            = 50;
  TopicName topicList50Hz[]  = { vehiclePtr->isM300() ? TOPIC_THREE_GIMBAL_DATA : TOPIC_DUAL_GIMBAL_DATA };
  int       numTopic        = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
      pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    DERROR("init package for gimbal data failed." );
  }
  subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, 1);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    vehicle->subscribe->removePackage(pkgIndex, 1);
    DERROR("subscribe gimbal data failed." );
  }

  /*! init gimbal modules for gimbalManager */
  ErrorCode::ErrorCodeType ret;
  /*! main gimbal init */
  ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_0,
                                                 "Sample_main_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Init Camera module Sample_main_gimbal failed.");
    ErrorCode::printErrorCodeMsg(ret);
  }
  /*! vice gimbal init */
  ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_1,
                                                 "Sample_vice_gimbal");
  if (ret != ErrorCode::SysCommonErr::Success)
  {
    DERROR("Init Camera module Sample_vice_gimbal failed.");
    ErrorCode::printErrorCodeMsg(ret);
  }
  /*! top gimbal init */
  if (vehicle->isM300()) {
    ret = vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_2,
                                                   "Sample_top_gimbal");
    if (ret != ErrorCode::SysCommonErr::Success) {
      DERROR("Init Camera module Sample_top_gimbal failed.");
      ErrorCode::printErrorCodeMsg(ret);
    }
  }
}

GimbalManagerSyncSample::~GimbalManagerSyncSample() {
  ACK::ErrorCode subscribeStatus =
      vehicle->subscribe->removePackage(GIMBA_SUB_PACKAGE_INDEX, 1);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    DERROR("remove subscribe package gimbal data failed." );
  }
}

ErrorCode::ErrorCodeType GimbalManagerSyncSample::resetSyncSample(
    PayloadIndexType index) {
  if (!vehicle || !vehicle->gimbalManager) {
    DERROR("vehicle or gimbalManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  GimbalManager *p = vehicle->gimbalManager;

  DSTATUS("Start to rotate the gimbal %d", index);
  retCode = p->resetSync(index, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Reset gimbal fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

ErrorCode::ErrorCodeType GimbalManagerSyncSample::rotateSyncSample(
    PayloadIndexType index, GimbalModule::Rotation rotation) {
  if (!vehicle || !vehicle->gimbalManager) {
    DERROR("vehicle or gimbalManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  GimbalManager *p = vehicle->gimbalManager;

  DSTATUS("Start to rotate the gimbal %d, (p,r,y) = (%0.2f,%0.2f,%0.2f) mode:%d"
          " time:%0.2fs", index, rotation.pitch, rotation.roll, rotation.yaw,
          rotation.rotationMode, rotation.time);
  retCode = p->rotateSync(index, rotation, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Rotation gimbal fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

GimbalSingleData GimbalManagerSyncSample::getGimbalData(PayloadIndexType index) {
  if (vehicle->isM300() && index <= PAYLOAD_INDEX_2 && index >= PAYLOAD_INDEX_0)
    return vehicle->subscribe->getValue<TOPIC_THREE_GIMBAL_DATA>().gbData[index];
  else if (index <= PAYLOAD_INDEX_1 && index >= PAYLOAD_INDEX_0)
    return vehicle->subscribe->getValue<TOPIC_DUAL_GIMBAL_DATA>().gbData[index];
  else {
    GimbalSingleData data = {0};
    DERROR("Invalid payload index : %d", index);
    return data;
  }
}