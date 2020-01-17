/*! @file GimbalManagerSample.cpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief Tesf for the GimbalManager. All tests are basic on callback method.
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

#include "GimbalManagerSample.hpp"
#include "gimbal_manager_sync_sample.hpp"
#include "timer.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int gimbalManagerTest(Vehicle *vehicle, PayloadIndexType index) {
  static GimbalManagerSyncSample *p = NULL;

  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting. \n");
    return -1;
  }
  if (index >= PAYLOAD_INDEX_CNT) {
    DERROR("Unknown test case. \n");
    return -1;
  }
  if (!p) {
    ErrorCode::ErrorCodeType ret = vehicle->gimbalManager->initGimbalModule(
        PAYLOAD_INDEX_0, "sample_gimbal_1");
    ret |= vehicle->gimbalManager->initGimbalModule(PAYLOAD_INDEX_1,
                                                    "sample_gimbal_2");
    if (ret != ErrorCode::SysCommonErr::Success) {
      DERROR("create payload node error\n");
      return -1;
    }
    p = new GimbalManagerSyncSample(vehicle);
    if (!p) {
      DERROR("Create GimbalManagerSyncSample error\n");
      return -1;
    }
  }

  /*! print the current angle of gimbal */
  DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2fdeg, %0.2fdeg, %0.2fdeg", index,
          p->getGimbalData(PAYLOAD_INDEX_0).pitch,
          p->getGimbalData(PAYLOAD_INDEX_0).roll,
          p->getGimbalData(PAYLOAD_INDEX_0).yaw);

  GimbalModule::Rotation rotation;
  rotation.roll = 0.0f;
  rotation.pitch = 25.0f;
  rotation.yaw = 90.0f;
  rotation.rotationMode = 0;
  rotation.time = 0.5;
  p->rotateSyncSample(index, rotation);
  delay_nms(2000);

  /*! print the current angle of gimbal */
  DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2fdeg, %0.2fdeg, %0.2fdeg", index,
          p->getGimbalData(PAYLOAD_INDEX_0).pitch,
          p->getGimbalData(PAYLOAD_INDEX_0).roll,
          p->getGimbalData(PAYLOAD_INDEX_0).yaw);
  p->resetSyncSample(index);
  delay_nms(2000);

  /*! print the current angle of gimbal */
  DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2fdeg, %0.2fdeg, %0.2fdeg", index,
          p->getGimbalData(PAYLOAD_INDEX_0).pitch,
          p->getGimbalData(PAYLOAD_INDEX_0).roll,
          p->getGimbalData(PAYLOAD_INDEX_0).yaw);

  delete p;
  return 0;
}
