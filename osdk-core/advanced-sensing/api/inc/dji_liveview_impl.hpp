/** @file dji_liveview_impl.hpp
 *  @version 4.0.0
 *  @date Jan 2020
 *
 *  @brief Camera liveview API code implement
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

#ifndef ONBOARDSDK_DJI_LIVEVIEW_IMPL_H
#define ONBOARDSDK_DJI_LIVEVIEW_IMPL_H

#include "dji_type.hpp"
#include "dji_vehicle.hpp"
#include "dji_liveview.hpp"
#include "dji_linker.hpp"

namespace DJI {
namespace OSDK {

// Forward Declaration
class Vehicle;

class LiveViewImpl {
 public:
  LiveViewImpl(Vehicle *vehiclePtr);

  ~LiveViewImpl();

  LiveView::LiveViewErrCode startH264Stream(LiveView::LiveViewCameraPosition pos, H264Callback cb, void *userData);

  LiveView::LiveViewErrCode stopH264Stream(LiveView::LiveViewCameraPosition pos);

  LiveView::LiveViewErrCode changeH264Source(LiveView::LiveViewCameraPosition pos, LiveView::LiveViewCameraSource source);

  typedef struct H264CallbackHandler {
    H264Callback cb;
    void *userData;
  } H264CallbackHandler;

 private:

  typedef enum E_OSDKCameraType {
    OSDK_CAMERA_TYPE_PSDK = 31,
    OSDK_CAMERA_TYPE_FPV = 39,  //Matrice FPV
    OSDK_CAMERA_TYPE_H20_DOUBLE_CAM = 42,
    OSDK_CAMERA_TYPE_H20_TIRPLE_CAM = 43, //IR
    OSDK_CAMERA_TYPE_UNKNOWN = 0xFF
  } E_OSDKCameraType;

  enum {
    UUID_MAJOR_TYPE_CAMERA = 0,
    UUID_MAJOR_TYPE_RADAR = 1,
    UUID_MAJOR_TYPE_PSDK = 2,
    UUID_MAJOR_TYPE_UNKNOWN = 255
  };

  typedef struct {
    // 2:OSDK
    uint8_t role;
    // 0:subscribe 1:unsubscribe
    uint8_t action;
    // always 0
    uint8_t type;
  } __attribute__((packed)) T_SubscribeData;

  typedef struct {
    uint8_t role;
  } __attribute__((packed)) T_HeartBeatData;

  typedef struct {
    uint8_t role;
    uint8_t chnNum;
  } __attribute__((packed)) T_LiveViewHeader;

  typedef struct {
    uint16_t cmdSize;
    uint8_t mode;
    uint8_t action;
    uint8_t channelId;
    uint8_t priority;
  } __attribute__((packed)) T_LiveViewMiniChannelItem;

  typedef struct {
    uint16_t cmdSize;
    uint8_t mode;
    uint8_t action;
    uint8_t channelId;
    uint8_t priority;
    uint8_t fps;
    uint16_t width;
    uint16_t height;
    uint8_t codecStrategy;
    uint8_t codecFormat;
    uint8_t adaptiveResolutionEnable;
    uint8_t sourceNum;
    uint8_t size;
  } __attribute__((packed)) T_LiveViewChannelItem;

  typedef struct {
    uint8_t version;
    uint8_t major;
    uint8_t minor;
    uint8_t dataIdx : 3;
    uint8_t devicePos : 3;
    uint8_t reserved : 2;
  } __attribute__((packed)) T_LiveViewUuid;

  typedef struct {
    T_LiveViewUuid uuid;
    uint8_t cropEnable;
    float cropOffsetX;
    float cropOffsetY;
    float cropWidth;
    float cropHeight;
  } __attribute__((packed)) T_LiveViewSingleSourceItem;

  typedef struct {
    T_LiveViewUuid uuid;
    uint8_t cropEnable;
    float cropOffsetX;
    float cropOffsetY;
    float cropWidth;
    float cropHeight;
    uint8_t order;
    float blendingOffsetX;
    float blendingOffsetY;
    float blendingWidth;
    float blendingHeight;
  } __attribute__((packed)) T_LiveViewMultiSourceItem;

  typedef struct {
    T_LiveViewHeader header;
    T_LiveViewChannelItem channel;
    T_LiveViewSingleSourceItem source;
  } __attribute__((packed)) T_LiveViewSubscribeItem;

  typedef struct {
    T_LiveViewHeader header;
    T_LiveViewMiniChannelItem channel;
  } __attribute__((packed)) T_LiveViewUnsubscribeItem;

  typedef struct CameraListType
  {
    E_OSDKCameraType cameraType[3];
    bool isMounted[3];
  } CameraListType;

  Vehicle *vehicle;

 private:
  static std::map<LiveView::LiveViewCameraPosition, H264CallbackHandler> h264CbHandlerMap;
  static T_RecvCmdItem bulkCmdList[];
  static E_OsdkStat RecordStreamHandler(struct _CommandHandle *cmdHandle,
                                        const T_CmdInfo *cmdInfo,
                                        const uint8_t *cmdData,
                                        void *userData);
  static E_OsdkStat getCameraPushing(struct _CommandHandle *cmdHandle,
                                     const T_CmdInfo *cmdInfo,
                                     const uint8_t *cmdData, void *userData);

  CameraListType getCameraList();
  int subscribeLiveViewData(E_OSDKCameraType type, LiveView::LiveViewCameraPosition pos);
  int unsubscribeLiveViewData(LiveView::LiveViewCameraPosition pos);

  T_OsdkTaskHandle h264TaskHandle;
  static void *heartBeatTask(void *p);
  E_OsdkStat startHeartBeatTask();
  E_OsdkStat stopHeartBeatTask();

};
} // OSDK
} // DJI

#endif //ONBOARDSDK_DJI_LIVEVIEW_IMPL_H
