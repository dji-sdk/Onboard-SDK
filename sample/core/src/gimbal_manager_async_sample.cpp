/*! @file gimbal_manager_async_sample.cpp
 *  @version 4.0 Beta
 *  @date December 12 2019
 *
 *  @brief
 *  Demonstrate how to use the asynchronous apis of gimbal manager.
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

#include "gimbal_manager_async_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

GimbalManagerAsyncSample::GimbalManagerAsyncSample(Vehicle *vehiclePtr)
    : vehicle(vehiclePtr) {}

GimbalManagerAsyncSample::~GimbalManagerAsyncSample() {}

void GimbalManagerAsyncSample::resetAsyncSample(PayloadIndexType index,
                                                void (*UserCallBack)(
                                                    ErrorCode::ErrorCodeType retCode,
                                                    UserData userData),
                                                UserData userData) {
  if (!vehicle || !vehicle->gimbalManager) {
    DERROR("vehicle or gimbalManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  GimbalManager *p = vehicle->gimbalManager;

  DSTATUS("Start to rotate the gimbal %d", index);
  p->resetAsync(index, UserCallBack, userData);
}

void GimbalManagerAsyncSample::rotateAsyncSample(PayloadIndexType index,
                                                GimbalModule::Rotation rotation,
                                                void (*UserCallBack)(
                                                    ErrorCode::ErrorCodeType retCode,
                                                    UserData userData),
                                                UserData userData) {
  if (!vehicle || !vehicle->gimbalManager) {
    DERROR("vehicle or gimbalManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  GimbalManager *p = vehicle->gimbalManager;

  DSTATUS("Start to rotate the gimbal %d, (p,r,y) = (%0.2f,%0.2f,%0.2f) mode:%d"
          " time:%0.2fs", index, rotation.pitch, rotation.roll, rotation.yaw,
          rotation.rotationMode, rotation.time);
  p->rotateAsync(index, rotation, UserCallBack, userData);
}
