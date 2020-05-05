/** @file dji_liveview.cpp
 *  @version 4.0
 *  @date Jan 2020
 *
 *  @brief Camera liveview API of OSDK
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

#define ADVANCED_SENSING

#include <dji_vehicle.hpp>
#include "dji_liveview.hpp"
#include "dji_liveview_impl.hpp"
#include "osdk_osal.h"

using namespace DJI;
using namespace DJI::OSDK;

LiveView::LiveView(Vehicle* vehiclePtr) : vehicle(vehiclePtr)
{
  impl = new LiveViewImpl(vehiclePtr);
}

LiveView::~LiveView()
{
  if(impl) delete(impl);
}

LiveView::LiveViewErrCode LiveView::startH264Stream(LiveViewCameraPosition pos, H264Callback cb, void *userData) {
  if(vehicle->isM300()) {
    return impl->startH264Stream(pos, cb, userData);
  } else if(vehicle->isM210V2()) {
    switch (pos) {
      case OSDK_CAMERA_POSITION_FPV:return (vehicle->advancedSensing->startFPVCameraH264(cb, userData)) ? OSDK_LIVEVIEW_PASS : OSDK_LIVEVIEW_UNKNOWN;
      case OSDK_CAMERA_POSITION_NO_1:return(vehicle->advancedSensing->startMainCameraH264(cb, userData)) ? OSDK_LIVEVIEW_PASS : OSDK_LIVEVIEW_UNKNOWN;
      default:
        DERROR("M210 V2 series only support FPV and MainCam H264 steam in OSDK.");
        return OSDK_LIVEVIEW_INDEX_ILLEGAL;
    }
  } else {
    return OSDK_LIVEVIEW_UNSUPPORT_AIRCRAFT;
  }
}

LiveView::LiveViewErrCode LiveView::stopH264Stream(LiveViewCameraPosition pos) {
  if (vehicle->isM300()) {
    return impl->stopH264Stream(pos);
  } else if (vehicle->isM210V2()) {
    switch (pos) {
      case OSDK_CAMERA_POSITION_FPV:
        vehicle->advancedSensing->stopFPVCameraH264();
        return OSDK_LIVEVIEW_PASS;
      case OSDK_CAMERA_POSITION_NO_1:
        vehicle->advancedSensing->stopMainCameraH264();
        return OSDK_LIVEVIEW_PASS;
      default:
        DERROR(
            "M210 V2 series only support FPV and MainCam H264 steam in OSDK.");
        return OSDK_LIVEVIEW_INDEX_ILLEGAL;
    }
  } else {
    DERROR("Only support M210 V2 and M300.");
    return OSDK_LIVEVIEW_UNSUPPORT_AIRCRAFT;
  }
}
