/** @file dji_perception_impl.hpp
 *  @version 4.0
 *  @date Jan 2020
 *
 *  @brief Camera dji perception API implementation of OSDK
 *
 *  @Copyright (c) 2020 DJI
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

#ifndef ONBOARDSDK_DJI_PERCEPTION_IMPL_H
#define ONBOARDSDK_DJI_PERCEPTION_IMPL_H

#include <cstring>
#include "dji_perception.hpp"
#include "dji_vehicle.hpp"
#include "dji_linker.hpp"

namespace DJI {
namespace OSDK {

// Forward Declaration
class Vehicle;

class PerceptionImpl {
 public:
  PerceptionImpl(Vehicle *vehiclePtr);

  ~PerceptionImpl();

 public:
  typedef struct PerceptionImageHandler {
    Perception::PerceptionImageCB cb;
    void* userData;
  } PerceptionImageHandler;

  typedef struct PerceptionCamParamHandler {
    Perception::PerceptionCamParamCB cb;
    void* userData;
  } PerceptionCamParamHandler;

  static E_OsdkStat cameraImageHandler(struct _CommandHandle *cmdHandle,
                                       const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData);

  static E_OsdkStat cameraParamHandler(struct _CommandHandle *cmdHandle,
                                       const T_CmdInfo *cmdInfo, const uint8_t *cmdData, void *userData);

  E_OsdkStat subscribePerceptionImage(const char camChoice[11]);

  E_OsdkStat subscribePerceptionImage(Perception::CamPositionType camChoice);

  E_OsdkStat subscribePerceptionImage(Perception::DirectionType directionChoice);

  E_OsdkStat unsubscribePerceptionImage(const char camChoice[11]);

  E_OsdkStat unsubscribePerceptionImage(Perception::CamPositionType camChoice);

  E_OsdkStat unsubscribePerceptionImage(Perception::DirectionType directionChoice);

  E_OsdkStat subscribeCameraParam();

  void cancelAllSubsciptions();

  vector<Perception::DirectionType> getUpdatingDiretcion();
 public:
  static PerceptionImageHandler imageHandler;
  static PerceptionCamParamHandler camParamHandler;

  static const char rectifyDownLeft[11];
  static const char rectifyDownRight[11];
  static const char rectifyFrontLeft[11];
  static const char rectifyFrontRight[11];
  static const char rectifyRearLeft[11];
  static const char rectifyRearRight[11];
  static const char rectifyUpLeft[11];
  static const char rectifyUpRight[11];
  static const char rectifyLeftLeft[11];
  static const char rectifyLeftRight[11];
  static const char rectifyRightLeft[11];
  static const char rectifyRightRight[11];

 private:
  Vehicle *vehicle;
  static uint32_t imageUpdateSysMs[IMAGE_MAX_DIRECTION_NUM];
  static uint32_t updateJudgingInMs;
  string getSubscribeString(Perception::CamPositionType camChoice);
};
} // OSDK
} // DJI

#endif //ONBOARDSDK_DJI_PERCEPTION_IMPL_H
