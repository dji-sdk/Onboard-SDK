/** @file dji_internal_command.hpp
 *  @version 4.0
 *  @date April 2020
 *
 *  @brief All DJI OSDK internal Command IDs
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

#ifndef DJI_INTERNAL_COMMAND_H
#define DJI_INTERNAL_COMMAND_H

#include <stdint.h>

namespace DJI
{
namespace OSDK
{

class V1ProtocolCMD
{
 public:
  const static int MAX_CMD_ARRAY_SIZE = 2;

  typedef struct Camera
  {
    const static uint8_t takePhoto[MAX_CMD_ARRAY_SIZE];
    const static uint8_t takeVideo[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setFocusParameter[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setExposureMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getExposureMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setFocusMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getFocusMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setEvParameter[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getEvParameter[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getFocusParameter[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setPointZoomMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getPointZoomMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t pointZoomCtrl[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setZoomParameter[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setCommonZoomPara[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getCommonZoomPara[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setIsoParameter[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getIsoParameter[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setShutterSpeed[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getShutterSpeed[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setApertureSize[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getApertureSize[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setMeteringMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getMeteringMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setSpotFocusAera[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getSpotFocusAera[MAX_CMD_ARRAY_SIZE];
    const static uint8_t setShotMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t getShotMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t controlOptizoom[MAX_CMD_ARRAY_SIZE];
  } Camera;

  typedef struct Gimbal
  {
    const static uint8_t resetAngle[MAX_CMD_ARRAY_SIZE];
    const static uint8_t rotateAngle[MAX_CMD_ARRAY_SIZE];
  } Gimbal;

  typedef struct Common
  {
    const static uint8_t getVersion[MAX_CMD_ARRAY_SIZE];
    const static uint8_t downloadFile[MAX_CMD_ARRAY_SIZE];
    const static uint8_t downloadFileAck[MAX_CMD_ARRAY_SIZE];
  } Common;

  typedef struct fc
  {
    const static uint8_t usbFlightMode[MAX_CMD_ARRAY_SIZE];
    const static uint8_t batteryInfo[MAX_CMD_ARRAY_SIZE];
  } fc;

  typedef struct SDK
  {
    const static uint8_t obtainDownloadRight[MAX_CMD_ARRAY_SIZE];
  } SDK;

  typedef struct PSDK
  {
    const static uint8_t IDVerification[MAX_CMD_ARRAY_SIZE];
    const static uint8_t uploadPolicyFile[MAX_CMD_ARRAY_SIZE];
  } PSDK;

  typedef struct HMS
  {
    const static uint8_t hmsStatus[MAX_CMD_ARRAY_SIZE];
    const static uint8_t hmsPushData[MAX_CMD_ARRAY_SIZE];
  } HMS;

  typedef struct waypointV2
  {
    const static uint8_t waypointInitV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointUploadV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointUploadActionV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointStartStopV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointResumePauseV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointBreakRestoreV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointDownloadInitV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointDownloadPtV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointDownloadActionV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointSetGlobVelocityV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetGlobVelocityV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetMaxPtNumV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetWayptIdxInListV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetRemainSpaceV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetBreakInfoV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetStatePushDataV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetEventPushDataV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetInfoV2[MAX_CMD_ARRAY_SIZE];
    const static uint8_t waypointGetMinMaxActionIDV2[MAX_CMD_ARRAY_SIZE];
  }waypointV2;

  typedef struct BatteryCmd
  {
    const static uint8_t getBatteryDynamicInfo[MAX_CMD_ARRAY_SIZE];
  } BatteryCmd;

  class CMDSet
  {
   public:
    //! CMD SET definitions
    const static uint8_t common       = 0;
    const static uint8_t special      = 1;
    const static uint8_t camera       = 2;
    const static uint8_t fc           = 3;
    const static uint8_t gimbal       = 4;
    const static uint8_t rc           = 6;
    const static uint8_t battery      = 13;
    const static uint8_t rtk          = 15;
    const static uint8_t waypointV2   = 34;
    const static uint8_t hms          = 33;
    const static uint8_t psdk         = 60;
    const static uint8_t sdk          = 73;
  };
};

} // namespace
} // namespace

#endif /* DJI_INTERNAL_COMMAND_H */
