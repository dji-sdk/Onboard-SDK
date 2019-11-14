/** @file dji_payload_manager.cpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of the manager for payload nodes
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

#include "dji_camera_manager.hpp"

using namespace DJI;
using namespace DJI::OSDK;

CameraManager::CameraManager(Vehicle* vehiclePtr) {
  payloadLink = new PayloadLink(vehiclePtr);
  for (int index = PAYLOAD_INDEX_0; index < PAYLOAD_INDEX_CNT; index++) {
    CameraModule* module = new CameraModule(
        payloadLink, (PayloadIndexType)index, defaultCameraName, false);
    cameraModuleVector.push_back(module);
  }
}

CameraManager::~CameraManager() {
  for (int i = 0; i < cameraModuleVector.size(); i++) {
    if (cameraModuleVector[i]) {
      delete cameraModuleVector[i];
    }
  }
  delete payloadLink;
}

CameraModule* CameraManager::getCameraModule(PayloadIndexType index) {
  for (int i = 0; i < cameraModuleVector.size(); ++i) {
    if (cameraModuleVector[i]->getIndex() == index) {
      return cameraModuleVector[i];
    }
  }
  return NULL;
}

CameraModule* CameraManager::getCameraModule(std::string name) {
  for (int i = 0; i < cameraModuleVector.size(); ++i) {
    if (cameraModuleVector[i]->getName() == name) {
      return cameraModuleVector[i];
    }
  }
  return NULL;
}

ErrorCode::ErrorCodeType CameraManager::initCameraModule(PayloadIndexType index,
                                                         const char* name) {
  /* @TODO lock protest CameraModule */
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setName(name);
    cameraMgr->setEnable(true);

    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType CameraManager::deinitCameraModule(
    PayloadIndexType index) {
  /* @TODO lock protest cameraMgr */
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setName(defaultCameraName);
    cameraMgr->setEnable(false);
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::deinitAllCameraModule() {
  for (int i = 0; i < cameraModuleVector.size(); ++i) {
    cameraModuleVector[i]->setName(defaultCameraName);
    cameraModuleVector[i]->setEnable(false);
  }
}

ErrorCode::ErrorCodeType CameraManager::getCameraModuleName(
    PayloadIndexType index, std::string& name) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    name = cameraMgr->getName();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType CameraManager::getCameraModuleIndex(const char* name,
                                                             uint8_t& index) {
  CameraModule* cameraMgr = getCameraModule(name);
  if (cameraMgr) {
    index = cameraMgr->getIndex();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType CameraManager::getCameraModuleEnable(
    PayloadIndexType index, bool& enable) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    enable = cameraMgr->getEnable();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::startShootPhotoAsync(
    PayloadIndexType index, CameraModule::ShootPhotoMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->startShootPhotoAsync(mode, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::startShootPhotoSync(
    PayloadIndexType index, CameraModule::ShootPhotoMode mode, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->startShootPhotoSync(mode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setISOAsync(
    PayloadIndexType index, CameraModule::ISO iso,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setISOAsync(iso, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setISOSync(PayloadIndexType index,
                                                   CameraModule::ISO iso,
                                                   int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setISOSync(iso, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getISOAsync(PayloadIndexType index,
                                void (*UserCallBack)(ErrorCode::ErrorCodeType,
                                                     CameraModule::ISO iso,
                                                     UserData userData),
                                UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getISOAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::ISO::ISO_UNKNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getISOSync(PayloadIndexType index,
                                                   CameraModule::ISO& iso,
                                                   int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getISOSync(iso, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::stopShootPhotoAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->stopShootPhotoAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::stopShootPhotoSync(
    PayloadIndexType index, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->stopShootPhotoSync(timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setShootPhotoModeAsync(
    PayloadIndexType index, CameraModule::ShootPhotoMode takePhotoMode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setShootPhotoModeAsync(takePhotoMode, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setShootPhotoModeSync(
    PayloadIndexType index, CameraModule::ShootPhotoMode takePhotoMode,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setShootPhotoModeSync(takePhotoMode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getShootPhotoModeAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::ShootPhotoMode takePhotoMode,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getShootPhotoModeAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::ShootPhotoMode::SHOOT_PHOTO_MODE_UNKNOWN,
                   userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getShootPhotoModeSync(
    PayloadIndexType index, CameraModule::ShootPhotoMode& takePhotoMode,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getShootPhotoModeSync(takePhotoMode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setPhotoBurstCountAsync(
    PayloadIndexType index, CameraModule::PhotoBurstCount count,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setPhotoBurstCountAsync(count, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setPhotoBurstCountSync(
    PayloadIndexType index, CameraModule::PhotoBurstCount count, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setPhotoBurstCountSync(count, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getPhotoBurstCountAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::PhotoBurstCount count,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getPhotoBurstCountAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::PhotoBurstCount::BURST_COUNT_KNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getPhotoBurstCountSync(
    PayloadIndexType index, CameraModule::PhotoBurstCount& count, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getPhotoBurstCountSync(count, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setPhotoAEBCountAsync(
    PayloadIndexType index, CameraModule::PhotoAEBCount count,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setPhotoAEBCountAsync(count, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setPhotoAEBCountSync(
    PayloadIndexType index, CameraModule::PhotoAEBCount count, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setPhotoAEBCountSync(count, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getPhotoAEBCountAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::PhotoAEBCount count, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getPhotoAEBCountAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::PhotoAEBCount::AEB_COUNT_KNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getPhotoAEBCountSync(
    PayloadIndexType index, CameraModule::PhotoAEBCount& count, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getPhotoAEBCountSync(count, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setPhotoTimeIntervalSettingsAsync(
    PayloadIndexType index, CameraModule::PhotoIntervalData intervalSetting,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setPhotoTimeIntervalSettingsAsync(intervalSetting, UserCallBack,
                                                 userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setPhotoTimeIntervalSettingsSync(
    PayloadIndexType index, CameraModule::PhotoIntervalData intervalSetting,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setPhotoTimeIntervalSettingsSync(intervalSetting,
                                                       timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getPhotoIntervalDatasAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::PhotoIntervalData intervalSetting,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getPhotoIntervalDatasAsync(UserCallBack, userData);
  } else {
    CameraModule::PhotoIntervalData intervalSetting = {0};
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, intervalSetting,
                   userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getPhotoIntervalDatasSync(
    PayloadIndexType index, CameraModule::PhotoIntervalData& intervalSetting,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getPhotoIntervalDatasSync(intervalSetting, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::startRecordVideoAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->startRecordVideoAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::startRecordVideoSync(
    PayloadIndexType index, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->startRecordVideoSync(timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::stopRecordVideoAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->stopRecordVideoAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::stopRecordVideoSync(
    PayloadIndexType index, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->stopRecordVideoSync(timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setModeAsync(
    PayloadIndexType index, CameraModule::WorkMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setModeAsync(mode, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setModeSync(PayloadIndexType index,
                                                    CameraModule::WorkMode mode,
                                                    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setModeSync(mode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getModeAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::WorkMode workingMode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getModeAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::WorkMode::WORK_MODE_UNKNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getModeSync(
    PayloadIndexType index, CameraModule::WorkMode& workingMode, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getModeSync(workingMode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setFocusModeAsync(
    PayloadIndexType index, CameraModule::FocusMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setFocusModeAsync(mode, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setFocusModeSync(
    PayloadIndexType index, CameraModule::FocusMode mode, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setFocusModeSync(mode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType CameraManager::getFocusModeSync(
    PayloadIndexType index, CameraModule::FocusMode& focusMode, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getFocusModeSync(focusMode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getFocusModeAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::FocusMode focusMode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getFocusModeAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::FocusMode::FOCUS_MODE_UNKNOWN, userData);
  }
}

void CameraManager::setFocusTargetAsync(
    PayloadIndexType index, CameraModule::TapFocusPosData tapFocusPos,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setFocusTargetAsync(tapFocusPos, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setFocusTargetSync(
    PayloadIndexType index, CameraModule::TapFocusPosData tapFocusPos,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setFocusTargetSync(tapFocusPos, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getFocusTargetAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType,
                         CameraModule::TapFocusPosData tapFocusPos,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getFocusTargetAsync(UserCallBack, userData);
  } else {
    CameraModule::TapFocusPosData pos = {0};
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, pos, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getFocusTargetSync(
    PayloadIndexType index, CameraModule::TapFocusPosData& tapFocusPos,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getFocusTargetSync(tapFocusPos, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::startContinuousOpticalZoomAsync(
    PayloadIndexType index, CameraModule::zoomDirectionData zoomDirection,
    CameraModule::zoomSpeedData zoomSpeed,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->startContinuousOpticalZoomAsync(zoomDirection, zoomSpeed,
                                               UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::startContinuousOpticalZoomSync(
    PayloadIndexType index, CameraModule::zoomDirectionData zoomDirection,
    CameraModule::zoomSpeedData zoomSpeed, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->startContinuousOpticalZoomSync(zoomDirection, zoomSpeed,
                                                     timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::stopContinuousOpticalZoomAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->stopContinuousOpticalZoomAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::stopContinuousOpticalZoomSync(
    PayloadIndexType index, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->stopContinuousOpticalZoomSync(timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setTapZoomEnabledAsync(
    PayloadIndexType index, bool param,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setTapZoomEnabledAsync(param, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setTapZoomEnabledSync(
    PayloadIndexType index, bool param, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setTapZoomEnabledSync(param, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getTapZoomEnabledAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, bool param,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getTapZoomEnabledAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, false, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getTapZoomEnabledSync(
    PayloadIndexType index, bool& param, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getTapZoomEnabledSync(param, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setTapZoomMultiplierAsync(
    PayloadIndexType index, CameraModule::TapZoomMultiplierData param,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr && (param >= 1) && (param <= 5)) {
    cameraMgr->setTapZoomMultiplierAsync(param, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setTapZoomMultiplierSync(
    PayloadIndexType index, CameraModule::TapZoomMultiplierData param,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr && (param >= 1) && (param <= 5)) {
    return cameraMgr->setTapZoomMultiplierSync(param, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getTapZoomMultiplierAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::TapZoomMultiplierData param,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getTapZoomMultiplierAsync(UserCallBack, userData);
  } else {
    CameraModule::TapZoomMultiplierData multiplier = {0};

    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, multiplier,
                   userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getTapZoomMultiplierSync(
    PayloadIndexType index, CameraModule::TapZoomMultiplierData& param,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getTapZoomMultiplierSync(param, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::tapZoomAtTargetAsync(
    PayloadIndexType index, CameraModule::TapZoomPosData tapZoomPos,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->tapZoomAtTargetAsync(tapZoomPos, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::tapZoomAtTargetSync(
    PayloadIndexType index, CameraModule::TapZoomPosData tapZoomPos,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->tapZoomAtTargetSync(tapZoomPos, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setExposureModeAsync(
    PayloadIndexType index, CameraModule::ExposureMode mode,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setExposureModeAsync(mode, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setExposureModeSync(
    PayloadIndexType index, CameraModule::ExposureMode mode, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setExposureModeSync(mode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getExposureModeAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::ExposureMode mode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getExposureModeAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::ExposureMode::EXPOSURE_UNKNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getExposureModeSync(
    PayloadIndexType index, CameraModule::ExposureMode& mode, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getExposureModeSync(mode, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setApertureAsync(
    PayloadIndexType index, CameraModule::Aperture aperture,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setApertureAsync(aperture, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setApertureSync(
    PayloadIndexType index, CameraModule::Aperture aperture, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setApertureSync(aperture, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getApertureAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType,
                         CameraModule::Aperture aperture, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getApertureAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::Aperture::F_UNKNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getApertureSync(
    PayloadIndexType index, CameraModule::Aperture& aperture, int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getApertureSync(aperture, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setShutterSpeedAsync(
    PayloadIndexType index, CameraModule::ShutterSpeed shutterSpeed,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setShutterSpeedAsync(shutterSpeed, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setShutterSpeedSync(
    PayloadIndexType index, CameraModule::ShutterSpeed shutterSpeed,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setShutterSpeedSync(shutterSpeed, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getShutterSpeedAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::ShutterSpeed shutterSpeed,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getShutterSpeedAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::ShutterSpeed::SHUTTER_SPEED_UNKNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getShutterSpeedSync(
    PayloadIndexType index, CameraModule::ShutterSpeed& shutterSpeed,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getShutterSpeedSync(shutterSpeed, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::setExposureCompensationAsync(
    PayloadIndexType index, CameraModule::ExposureCompensation ev,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->setExposureCompensationAsync(ev, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::setExposureCompensationSync(
    PayloadIndexType index, CameraModule::ExposureCompensation ev,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->setExposureCompensationSync(ev, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void CameraManager::getExposureCompensationAsync(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                         CameraModule::ExposureCompensation ev,
                         UserData userData),
    UserData userData) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    cameraMgr->getExposureCompensationAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed,
                   CameraModule::ExposureCompensation::UNKNOWN, userData);
  }
}

ErrorCode::ErrorCodeType CameraManager::getExposureCompensationSync(
    PayloadIndexType index, CameraModule::ExposureCompensation& ev,
    int timeout) {
  CameraModule* cameraMgr = getCameraModule(index);
  if (cameraMgr) {
    return cameraMgr->getExposureCompensationSync(ev, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}
