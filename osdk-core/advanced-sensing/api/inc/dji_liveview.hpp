/** @file dji_liveview.hpp
 *  @version 4.0.0
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
    OSDK_CAMERA_SOURCE_DEF = 0,
    OSDK_CAMERA_SOURCE_H20_WIDE = 1,
    OSDK_CAMERA_SOURCE_H20T_WIDE = 1,
    OSDK_CAMERA_SOURCE_H20_ZOOM = 2,
    OSDK_CAMERA_SOURCE_H20T_ZOOM = 2,
    OSDK_CAMERA_SOURCE_H20T_IR = 3
  } LiveViewCameraSource;

  typedef enum {
    OSDK_LIVEVIEW_PASS = 0,
    OSDK_LIVEVIEW_TIMEOUT = 1,
    OSDK_LIVEVIEW_SUBSCRIBE_FAIL = 2,
    OSDK_LIVEVIEW_INDEX_ILLEGAL = 3,
    OSDK_LIVEVIEW_HEART_BEAT_START_FAIL = 4,
    OSDK_LIVEVIEW_CAM_NOT_MOUNTED = 5,
    OSDK_LIVEVIEW_UNSUPPORT_AIRCRAFT = 6,
    OSDK_LIVEVIEW_UNSUPPORT_CAMERA = 7,
    OSDK_LIVEVIEW_UNKNOWN = 0xFF,
  } LiveViewErrCode;

 public:
  LiveView(Vehicle *vehiclePtr);

  ~LiveView();
  /*! @brief
   *
   *  Start the FPV or Camera H264 Stream
   *
   *  @platforms M300
   *  @param pos point out which camera to output the H264 stream
   *  @param cb callback function that is called in a callback thread when a new
   *            h264 frame is received
   *  @param cbParam a void pointer that users can manipulate inside the callback
   *  @return Errorcode of liveivew, ref to DJI::OSDK::LiveView::LiveViewErrCode
   */
  LiveViewErrCode startH264Stream(LiveViewCameraPosition pos, H264Callback cb, void *userData);

  /*! @brief
   *
   *  Stop the FPV or Camera H264 Stream
   *
   *  @platforms M300
   *  @param pos point out which camera to output the H264 stream
   *  @return Errorcode of liveivew, ref to DJI::OSDK::LiveView::LiveViewErrCode
   */
  LiveViewErrCode stopH264Stream(LiveViewCameraPosition pos);

  /*! @brief
   *  Change the camera stream source from one payload device. (Beta API)
   *
   *  @platforms M300
   *  @note Only support for payload device : H20/H20T
   *  @param pos point out which camera to output the H264 stream
   *  @param source change to be the target camera of the payload, ref to
   *         LiveView::LiveViewCameraSource
   *  @return Errorcode of liveivew, ref to DJI::OSDK::LiveView::LiveViewErrCode
   */
  LiveViewErrCode changeH264Source(LiveViewCameraPosition pos, LiveViewCameraSource source);

 private:
  Vehicle *vehicle;
  LiveViewImpl *impl;

};
} // OSDK
} // DJI

#endif //ONBOARDSDK_DJI_LIVEVIEW_H
