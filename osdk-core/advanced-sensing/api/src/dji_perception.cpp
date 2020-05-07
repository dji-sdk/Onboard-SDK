/** @file dji_perception.cpp
 *  @version 4.0
 *  @date Jan 2020
 *
 *  @brief DJI perception API of OSDK
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


#include "dji_perception.hpp"
#include "dji_perception_impl.hpp"
#include "osdk_osal.h"

using namespace DJI;
using namespace DJI::OSDK;

Perception::Perception(Vehicle *vehiclePtr) : vehicle(vehiclePtr) {
  impl = new PerceptionImpl(vehicle);
  OsdkOsal_TaskSleepMs(300);
  cancelAllSubsciptions();
}

Perception::~Perception() {
  cancelAllSubsciptions();
  if (impl) delete impl;
}

Perception::PerceptionErrCode Perception::subscribePerceptionImage(DirectionType direction,
                                          PerceptionImageCB cb,
                                          void *userData) {
  const char *camChoice1;
  const char *camChoice2;

  switch (direction) {
    case RECTIFY_DOWN:
      camChoice1 = PerceptionImpl::rectifyDownLeft;
      camChoice2 = PerceptionImpl::rectifyDownRight;
      break;
    case RECTIFY_FRONT:
      camChoice1 = PerceptionImpl::rectifyFrontLeft;
      camChoice2 = PerceptionImpl::rectifyFrontRight;
      break;
    case RECTIFY_REAR:
      camChoice1 = PerceptionImpl::rectifyRearLeft;
      camChoice2 = PerceptionImpl::rectifyRearRight;
      break;
    case RECTIFY_UP:
      camChoice1 = PerceptionImpl::rectifyUpLeft;
      camChoice2 = PerceptionImpl::rectifyUpRight;
      break;
    case RECTIFY_LEFT:
      camChoice1 = PerceptionImpl::rectifyLeftLeft;
      camChoice2 = PerceptionImpl::rectifyLeftRight;
      break;
    case RECTIFY_RIGHT:
      camChoice1 = PerceptionImpl::rectifyRightLeft;
      camChoice2 = PerceptionImpl::rectifyRightRight;
      break;
    default: return OSDK_PERCEPTION_PARAM_ERR;
  }
  if (impl->subscribePerceptionImage(camChoice1) == OSDK_STAT_OK) {
    DSTATUS("Subscribe perception image %s successfully", camChoice1);
    if (impl->subscribePerceptionImage(camChoice2) == OSDK_STAT_OK) {
      DSTATUS("Subscribe perception image %s successfully", camChoice2);
      impl->imageHandler = {cb, userData};
      return OSDK_PERCEPTION_PASS;
    } else {
      DERROR("Subscribe perception image %s failed", camChoice2);
      DERROR("Now unsubscribePerceptionImage perception image : %s", camChoice1);
      impl->unsubscribePerceptionImage(camChoice1);
      return OSDK_PERCEPTION_SUBSCRIBE_FAIL;
    }
  } else {
    return OSDK_PERCEPTION_SUBSCRIBE_FAIL;
  }
}

Perception::PerceptionErrCode Perception::unsubscribePerceptionImage(DirectionType direction) {
  Perception::PerceptionErrCode ret = OSDK_PERCEPTION_PASS;
  auto result = impl->unsubscribePerceptionImage(direction);

  if (result == OSDK_STAT_OK) return OSDK_PERCEPTION_PASS;
  else if (result == OSDK_STAT_ERR_PARAM) return OSDK_PERCEPTION_PARAM_ERR;
  else if (result == OSDK_STAT_ERR_TIMEOUT) return OSDK_PERCEPTION_REQ_REFUSED;
  else return OSDK_PERCEPTION_SUBSCRIBE_FAIL;

}

Perception::PerceptionErrCode Perception::triggerStereoCamParamsPushing() {
  E_OsdkStat ret = impl->subscribeCameraParam();
  if (ret == OSDK_STAT_OK) {
    return OSDK_PERCEPTION_PASS;
  } else if (ret == OSDK_STAT_ERR_TIMEOUT) {
    return OSDK_PERCEPTION_TIMEOUT;
  } else {
    return OSDK_PERCEPTION_REQ_REFUSED;
  }
}

void Perception::setStereoCamParamsObserver(PerceptionCamParamCB cb, void *userData) {
  impl->camParamHandler = {cb, userData};
}

void Perception::cancelAllSubsciptions() {
  impl->cancelAllSubsciptions();
}
