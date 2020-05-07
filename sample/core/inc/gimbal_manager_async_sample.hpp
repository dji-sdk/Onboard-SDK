/*! @file gimbal_manager_async_sample.hpp
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

#ifndef ONBOARDSDK_GIMBAL_MANAGER_ASYNC_SAMPLE_HPP
#define ONBOARDSDK_GIMBAL_MANAGER_ASYNC_SAMPLE_HPP

#include <dji_vehicle.hpp>
#include "dji_gimbal_manager.hpp"
#include "dji_telemetry.hpp"

using namespace DJI::OSDK::Telemetry;

/*! @brief gimbal manager async sample
 */
class GimbalManagerAsyncSample {
 public:
  GimbalManagerAsyncSample(Vehicle* vehiclePtr);

  ~GimbalManagerAsyncSample();

 public:
  void resetAsyncSample(PayloadIndexType index,
                        void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);
  void rotateAsyncSample(PayloadIndexType index, GimbalModule::Rotation rotation,
                        void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);

  GimbalSingleData getGimbalData(PayloadIndexType index);

 private:
  Vehicle* vehicle;
};

#endif  // ONBOARDSDK_GIMBAL_MANAGER_ASYNC_SAMPLE_HPP
