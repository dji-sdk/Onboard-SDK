/** @file dji_liveview.hpp
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

#ifndef ONBOARDSDK_DJI_LIVEVIEW_H
#define ONBOARDSDK_DJI_LIVEVIEW_H

#include <cstring>
#include "dji_camera_image.hpp"

namespace DJI {
namespace OSDK {

// Forward Declaration
class LiveViewImpl;

class LiveView {
 public:
  typedef enum {
    OSDK_CAMERA_POSITION_NO_1 = 0,
    OSDK_CAMERA_POSITION_NO_2 = 1,
    OSDK_CAMERA_POSITION_NO_3 = 2,
    OSDK_CAMERA_POSITION_FPV = 7
  } LiveViewCameraPosition;

  typedef enum {
    OSDK_LIVEVIEW_PASS = 0,
    OSDK_LIVEVIEW_TIMEOUT = 1,
    OSDK_LIVEVIEW_SUBSCRIBE_FAIL = 2,
    OSDK_LIVEVIEW_INDEX_ILLEGAL = 3,
    OSDK_LIVEVIEW_HEART_BEAT_START_FAIL = 4,
    OSDK_LIVEVIEW_CAM_NOT_MOUNTED = 5,
    OSDK_LIVEVIEW_UNSUPPORT_AIRCRAFT = 6,
    OSDK_LIVEVIEW_UNKNOWN = 0xFF,
  } LiveViewErrCode;

 public:
  LiveView(Vehicle *vehiclePtr);

  ~LiveView();

  LiveViewErrCode startH264Stream(LiveViewCameraPosition pos, H264Callback cb, void *userData);

  LiveViewErrCode stopH264Stream(LiveViewCameraPosition pos);

 private:
  Vehicle *vehicle;
  LiveViewImpl *impl;

};
} // OSDK
} // DJI

#endif //ONBOARDSDK_DJI_LIVEVIEW_H
