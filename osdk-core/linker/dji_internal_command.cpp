/** @file dji_internal_command.cpp
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

#include "dji_internal_command.hpp"

/*! v1 commands */
const uint8_t DJI::OSDK::V1ProtocolCMD::Gimbal::resetAngle[] = {
    V1ProtocolCMD::CMDSet::gimbal, 0x4C
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Gimbal::rotateAngle[] = {
    V1ProtocolCMD::CMDSet::gimbal, 0x0A
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Common::getVersion[] = {
    V1ProtocolCMD::CMDSet::common, 0x01
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Common::downloadFile[] = {
    V1ProtocolCMD::CMDSet::common, 0x26
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Common::downloadFileAck[] = {
    V1ProtocolCMD::CMDSet::common, 0x27
};
const uint8_t DJI::OSDK::V1ProtocolCMD::fc::usbFlightMode[] = {
    V1ProtocolCMD::CMDSet::fc, 0x1B
};
const uint8_t DJI::OSDK::V1ProtocolCMD::fc::batteryInfo[] = {
        V1ProtocolCMD::CMDSet::fc, 0x51
};
const uint8_t DJI::OSDK::V1ProtocolCMD::SDK::obtainDownloadRight[] = {
    V1ProtocolCMD::CMDSet::sdk, 0x20
};
const uint8_t DJI::OSDK::V1ProtocolCMD::PSDK::IDVerification[] = {
    V1ProtocolCMD::CMDSet::psdk, 0x37
};
const uint8_t DJI::OSDK::V1ProtocolCMD::PSDK::uploadPolicyFile[] = {
    V1ProtocolCMD::CMDSet::psdk, 0x40
};

const uint8_t DJI::OSDK::V1ProtocolCMD::HMS::hmsStatus[] = {
    V1ProtocolCMD::CMDSet::hms, 0x05
};
const uint8_t DJI::OSDK::V1ProtocolCMD::HMS::hmsPushData[] = {
    V1ProtocolCMD::CMDSet::hms, 0x06
};

const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::takePhoto[] = {
    V1ProtocolCMD::CMDSet::camera, 0x01
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::takeVideo[] = {
    V1ProtocolCMD::CMDSet::camera, 0x02
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x10
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x11
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setFocusParameter[] = {
    V1ProtocolCMD::CMDSet::camera, 0xc4
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setExposureMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x1e
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getExposureMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x1f
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setFocusMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x24
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getFocusMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x25
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setEvParameter[] = {
    V1ProtocolCMD::CMDSet::camera, 0x2e
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getEvParameter[] = {
    V1ProtocolCMD::CMDSet::camera, 0x2f
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getFocusParameter[] = {
    V1ProtocolCMD::CMDSet::camera, 0xc5
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setPointZoomMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0xc4
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getPointZoomMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0xc5
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::pointZoomCtrl[] = {
    V1ProtocolCMD::CMDSet::camera, 0xc6
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setZoomParameter[] = {
    V1ProtocolCMD::CMDSet::camera, 0xb8
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setIsoParameter[] = {
    V1ProtocolCMD::CMDSet::camera, 0x2a
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getIsoParameter[] = {
    V1ProtocolCMD::CMDSet::camera, 0x2b
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setShutterSpeed[] = {
    V1ProtocolCMD::CMDSet::camera, 0x28
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getShutterSpeed[] = {
    V1ProtocolCMD::CMDSet::camera, 0x29
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setApertureSize[] = {
    V1ProtocolCMD::CMDSet::camera, 0x26
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getApertureSize[] = {
    V1ProtocolCMD::CMDSet::camera, 0x27
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setMeteringMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x22
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getMeteringMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x23
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setSpotFocusAera[] = {
    V1ProtocolCMD::CMDSet::camera, 0x30
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getSpotFocusAera[] = {
    V1ProtocolCMD::CMDSet::camera, 0x31
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::setShotMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x6a
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::getShotMode[] = {
    V1ProtocolCMD::CMDSet::camera, 0x6b
};
const uint8_t DJI::OSDK::V1ProtocolCMD::Camera::controlOptizoom[] = {
    V1ProtocolCMD::CMDSet::camera, 0xb8
};

const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointInitV2[] = {
  V1ProtocolCMD::CMDSet::waypointV2, 0x01};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointUploadV2[] = {
  V1ProtocolCMD::CMDSet::waypointV2, 0x02};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointStartStopV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x03};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointSetGlobVelocityV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x05};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetGlobVelocityV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x06};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointResumePauseV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x07};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointDownloadInitV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x08};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointDownloadPtV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x09};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetMaxPtNumV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x0d};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetWayptIdxInListV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x0E};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointBreakRestoreV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x17};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointUploadActionV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x18};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointDownloadActionV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x19};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetRemainSpaceV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x1A};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetStatePushDataV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x1B};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetEventPushDataV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x1C};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetInfoV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x1D};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetMinMaxActionIDV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x1E};
const uint8_t DJI::OSDK::V1ProtocolCMD::waypointV2::waypointGetBreakInfoV2[]= {
  V1ProtocolCMD::CMDSet::waypointV2, 0x27};
const uint8_t DJI::OSDK::V1ProtocolCMD::BatteryCmd::getBatteryDynamicInfo[]= {
  V1ProtocolCMD::CMDSet::battery, 0x02};
