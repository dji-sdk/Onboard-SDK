/** @file dji_perception.hpp
 *  @version 4.0
 *  @date Jan 2020
 *
 *  @brief Camera dji perception API of OSDK
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

#ifndef ONBOARDSDK_DJI_PERCEPTION_H
#define ONBOARDSDK_DJI_PERCEPTION_H

#include "cstring"
#include "stdint.h"

#define IMAGE_MAX_DIRECTION_NUM        (6)

namespace DJI {
namespace OSDK {

// Forward Declaration
class Vehicle;
class PerceptionImpl;

class Perception {
 public:
  Perception(Vehicle *vehiclePtr);

  ~Perception();

 public:

  typedef enum {
    OSDK_PERCEPTION_PASS = 0,
    OSDK_PERCEPTION_TIMEOUT = 1,
    OSDK_PERCEPTION_SUBSCRIBE_FAIL = 2,
    OSDK_PERCEPTION_PARAM_ERR = 3,
    OSDK_PERCEPTION_REQ_REFUSED = 4,
    OSDK_LIVEVIEW_UNKNOWN = 0xFF,
  } PerceptionErrCode;

  typedef enum CamPositionType : uint32_t {
    RECTIFY_DOWN_LEFT = 1,
    RECTIFY_DOWN_RIGHT = 2,
    RECTIFY_FRONT_LEFT = 3,
    RECTIFY_FRONT_RIGHT = 4,
    RECTIFY_REAR_LEFT = 5,
    RECTIFY_REAR_RIGHT = 6,
    RECTIFY_UP_LEFT = 21,
    RECTIFY_UP_RIGHT = 22,
    RECTIFY_LEFT_LEFT = 23,
    RECTIFY_LEFT_RIGHT = 24,
    RECTIFY_RIGHT_LEFT = 25,
    RECTIFY_RIGHT_RIGHT = 26
  } CamPositionType;

  typedef enum DirectionType : uint8_t {
    RECTIFY_DOWN = 0,
    RECTIFY_FRONT = 1,
    RECTIFY_REAR = 2,
    RECTIFY_UP = 3,
    RECTIFY_LEFT = 4,
    RECTIFY_RIGHT = 5
  } DirectionType;

#pragma pack(1)
  typedef struct RawImageInfoType {
    uint32_t index;
    DirectionType direction;
    uint8_t bpp;
    uint32_t width;
    uint32_t height;
  } RawImageInfoType;

  typedef struct ImageInfoType {
    RawImageInfoType rawInfo;
    uint16_t dataId;
    uint16_t sequence;
    CamPositionType dataType;
    uint64_t timeStamp;
  } ImageInfoType;

  typedef struct CamParamType {
    DirectionType direction;
    float leftIntrinsics[9];
    float rightIntrinsics[9];
    float rotaionLeftInRight[9];
    float translationLeftInRight[3];
  } CamParamType;

  typedef struct {
    uint32_t timeStamp;//ms
    uint32_t directionNum;
    CamParamType cameraParam[IMAGE_MAX_DIRECTION_NUM];
  } CamParamPacketType;
#pragma pack()

  /*! @bref callback type to receive stereo camera parameters */
  typedef void(*PerceptionCamParamCB)
      (Perception::CamParamPacketType paramPacket, void *userData);

  /*! @bref callback type to receive stereo camera image */
  typedef void(*PerceptionImageCB)
      (Perception::ImageInfoType, uint8_t *imageRawBuffer, int bufferLen, void *userData);

 public:

  /*! @brief subscribe the raw images of both stereo cameras in the same
   * direction. Default frequency at 20 Hz.
   *
   *  @param direction to specifly the direction of the subscription. Ref to
   * DJI::OSDK::Perception::DirectionType
   *  @param cb callback to observer the stereo camera image and info.
   *  @param userData when cb is called, used in cb.
   *  @return error code. Ref to DJI::OSDK::Perception::PerceptionErrCode
   */
  PerceptionErrCode subscribePerceptionImage(DirectionType direction, PerceptionImageCB cb, void* userData);

  /*! @brief unsubscribe the raw image of both stereo cameras in the same
   * direction.
   *
   *  @param direction to specifly the direction of the subscription. Ref to
   * DJI::OSDK::Perception::DirectionType
   *  @return error code. Ref to DJI::OSDK::Perception::PerceptionErrCode
   */
  PerceptionErrCode unsubscribePerceptionImage(DirectionType direction);

  /*! @brief trigger stereo cameras parameters pushing once.
   *
   *  @param direction to specifly the direction of the subscription. Ref to
   * DJI::OSDK::Perception::DirectionType
   *  @return error code. Ref to DJI::OSDK::Perception::PerceptionErrCode
   */
  PerceptionErrCode triggerStereoCamParamsPushing();

  /*! @brief set callback to get stereo camera parameters after trigger stereo
   * camera parameters pushing.
   *
   *  @param cb callback to observer the parameters of stereo cameras. Ref to
   * DJI::OSDK:Perception::PerceptionCamParamCB
   *  @param userData when cb is called, used in cb.
   */
  void setStereoCamParamsObserver(PerceptionCamParamCB cb, void *userData);

  /*! @brief unsubscribe all the stereo camera parameters pushing.
   */
  void cancelAllSubsciptions();

 private:
  Vehicle *vehicle;
  PerceptionImpl *impl;
};
} // OSDK
} // DJI

#endif //ONBOARDSDK_DJI_PERCEPTION_H
