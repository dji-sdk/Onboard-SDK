/*! @file camera_manager_async_sample.cpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief
 *  Demonstrate how to use the asynchronous apis of camera manager.
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

#include "camera_manager_async_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

CameraManagerAsyncSample::CameraManagerAsyncSample(Vehicle *vehiclePtr)
    : vehicle(vehiclePtr) {}

CameraManagerAsyncSample::~CameraManagerAsyncSample() {}

void CameraManagerAsyncSample::getExposureModeCb(
    ErrorCode::ErrorCodeType retCode,
    CameraModule::ExposureMode exposureModeGet, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get exposure mode = %d", exposureModeGet);
    if (uData->pm) {
      /*! compare the exposure mode set and get */
      if (*(CameraModule::ExposureMode *)uData->dataTarget == exposureModeGet) {
        DSTATUS("The exposure mode is already %d.", exposureModeGet);
        if (uData->userCallBack) {
          void (*cb)(ErrorCode::ErrorCodeType, UserData);
          cb =
              (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
          cb(ErrorCode::SysCommonErr::Success, uData->userData);
        }
      } else {
        uData->pm->setExposureModeAsync(
            uData->index, *(CameraModule::ExposureMode *)uData->dataTarget,
            (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
            uData->userData);
      }
    }

  } else {
    DERROR("Get exposure mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::getISOCb(ErrorCode::ErrorCodeType retCode,
                                        CameraModule::ISO isoGet,
                                        UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get iso = %d", isoGet);
    if (uData->pm) {
      /*! compare the iso set and get */
      if (*(CameraModule::ISO *)uData->dataTarget == isoGet) {
        DSTATUS("The iso value is already %d.", isoGet);
        if (uData->userCallBack) {
          void (*cb)(ErrorCode::ErrorCodeType, UserData);
          cb =
              (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
          cb(ErrorCode::SysCommonErr::Success, uData->userData);
        }
      } else {
        uData->pm->setISOAsync(
            uData->index, *(CameraModule::ISO *)uData->dataTarget,
            (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
            uData->userData);
      }
    }

  } else {
    DERROR("Get iso error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::getShutterSpeedCb(
    ErrorCode::ErrorCodeType retCode,
    CameraModule::ShutterSpeed shutterSpeedGet, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get shutter speed = %d", shutterSpeedGet);
    if (uData->pm) {
      /*! compare the shutter speed set and get */
      if (*(CameraModule::ShutterSpeed *)uData->dataTarget == shutterSpeedGet) {
        DSTATUS("The shutter speed  value is already %d.", shutterSpeedGet);
        if (uData->userCallBack) {
          void (*cb)(ErrorCode::ErrorCodeType, UserData);
          cb =
              (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
          cb(ErrorCode::SysCommonErr::Success, uData->userData);
        }
      } else {
        uData->pm->setShutterSpeedAsync(
            uData->index, *(CameraModule::ShutterSpeed *)uData->dataTarget,
            (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
            uData->userData);
      }
    }

  } else {
    DERROR("Get shutter speed error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::getApertureCb(ErrorCode::ErrorCodeType retCode,
                                             CameraModule::Aperture apertureGet,
                                             UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get aperture = %d", apertureGet);
    if (uData->pm) {
      /*! compare the aperture set and get */
      if (*(CameraModule::Aperture *)uData->dataTarget == apertureGet) {
        DSTATUS("The aperture value is already %d.", apertureGet);
        if (uData->userCallBack) {
          void (*cb)(ErrorCode::ErrorCodeType, UserData);
          cb =
              (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
          cb(ErrorCode::SysCommonErr::Success, uData->userData);
        }
      } else {
        uData->pm->setApertureAsync(
            uData->index, *(CameraModule::Aperture *)uData->dataTarget,
            (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
            uData->userData);
      }
    }

  } else {
    DERROR("Get aperture error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::getEVCb(ErrorCode::ErrorCodeType retCode,
                                       CameraModule::ExposureCompensation evGet,
                                       UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get exposure compensation value = %d", evGet);
    if (uData->pm) {
      /*! compare the exposure compensation set and get */
      if (*(CameraModule::ExposureCompensation *)uData->dataTarget == evGet) {
        DSTATUS("The exposure compensation value is already %d.", evGet);
        if (uData->userCallBack) {
          void (*cb)(ErrorCode::ErrorCodeType, UserData);
          cb =
              (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
          cb(ErrorCode::SysCommonErr::Success, uData->userData);
        }
      } else {
        uData->pm->setExposureCompensationAsync(
            uData->index,
            *(CameraModule::ExposureCompensation *)uData->dataTarget,
            (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
            uData->userData);
      }
    }

  } else {
    DERROR("Get exposure compensation value error. Error code : 0x%lX",
           retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setExposureModeAsyncSample(
    PayloadIndexType index, CameraModule::ExposureMode dataTarget,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &dataTarget, sizeof(dataTarget));

  /*! get the value from camera */
  DSTATUS("Get exposure mode now ...");
  pm->getExposureModeAsync(index, getExposureModeCb, &uData);
}

void CameraManagerAsyncSample::setISOAsyncSample(
    PayloadIndexType index, CameraModule::ISO dataTarget,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  DSTATUS(
      "For the X5, X5R, X4S and X5S, the ISO value can be set for all "
      "modes. For the other cameras, the ISO value can only be set when "
      "the camera exposure mode is in Manual mode.");
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &dataTarget, sizeof(dataTarget));

  /*! get the value from camera */
  DSTATUS("Get iso now ...");
  pm->getISOAsync(index, getISOCb, &uData);
}

void CameraManagerAsyncSample::setShutterSpeedAsyncSample(
    PayloadIndexType index, CameraModule::ShutterSpeed dataTarget,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  DSTATUS(
      "The shutter speed can be set only when the camera exposure mode "
      "is Shutter mode or Manual mode. The shutter speed should not be "
      "set slower than the video frame rate when the camera's mode is "
      "RECORD_VIDEO.");
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &dataTarget, sizeof(dataTarget));

  /*! get the value from camera */
  DSTATUS("Get shutter speed now ...");
  pm->getShutterSpeedAsync(index, getShutterSpeedCb, &uData);
}

void CameraManagerAsyncSample::setApertureAsyncSample(
    PayloadIndexType index, CameraModule::Aperture dataTarget,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  DSTATUS(
      "In order to use this function, the exposure mode ExposureMode "
      "must be in MANUAL or APERTURE_PRIORITY. Supported only by the X5, "
      "X5R, X4S, X5S camera.");
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &dataTarget, sizeof(dataTarget));

  /*! get the value from camera */
  DSTATUS("Get aperture now ...");
  pm->getApertureAsync(index, getApertureCb, &uData);
}

void CameraManagerAsyncSample::setEVAsyncSample(
    PayloadIndexType index, CameraModule::ExposureCompensation dataTarget,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  DSTATUS(
      "In order to use this function, the camera exposure mode should be "
      "set to be PROGRAM_AUTO, SHUTTER_PRIORITY or APERTURE_PRIORITY "
      "first");
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &dataTarget, sizeof(dataTarget));

  /*! get the value from camera */
  DSTATUS("Get ev now ...");
  pm->getExposureCompensationAsync(index, getEVCb, &uData);
}

void CameraManagerAsyncSample::setFocusModeCb(ErrorCode::ErrorCodeType retCode,
                                              UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set focus mode as Auto successfully ");
    if (uData->pm) {
      /*! set focus point */
      uData->pm->setFocusTargetAsync(
          uData->index, *(CameraModule::TapFocusPosData *)uData->dataTarget,
          (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
          uData->userData);
    }

  } else {
    DERROR("Set focus mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setFocusPointAsyncSample(
    PayloadIndexType index, float x, float y,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  DSTATUS(
      "When the focus mode is auto, the target point is the focal point. "
      "When the focus mode is manual, the target point is the zoom out area "
      "if the focus assistant is enabled for the manual mode. Supported only "
      "by the X5, X5R, Z3 cameras, Mavic Pro camera, Phantom 4 Pro camera, "
      "Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise Camera, X5S. "
      "It's should be attention that X4S will keep focus point as (0.5,0.5) "
      "all the time, the setting of focus point to X4S will quickly replaced "
      "by (0.5, 0.5).");
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  CameraModule::TapFocusPosData tapFocusPos{x, y};
  memcpy(uData.dataTarget, &tapFocusPos, sizeof(tapFocusPos));

  /*!< set camera focus mode to be CameraModule::FocusMode::AUTO */
  DSTATUS("Set focus mode = %d", CameraModule::FocusMode::AUTO);
  pm->setFocusModeAsync(index, CameraModule::FocusMode::AUTO, setFocusModeCb,
                        &uData);
}

void CameraManagerAsyncSample::setTapZoomEnableCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set tap zoom enable successfully ");
    if (uData->pm) {
      /*! set tap zoom point */
      uData->pm->tapZoomAtTargetAsync(
          uData->index, *(CameraModule::TapZoomPosData *)uData->dataTarget,
          (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
          uData->userData);
    }

  } else {
    DERROR("Tap zoom at enable error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setTapZoomMultiplierCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set tap zoom multiplier successfully ");
    if (uData->pm) {
      /*! set tap zoom enable */
      uData->pm->setTapZoomEnabledAsync(uData->index, true, setTapZoomEnableCb,
                                        uData);
    }

  } else {
    DERROR("Set tap zoom multiplier error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setTapZoomPointAsyncSample(
    PayloadIndexType index, uint8_t multiplier, float x, float y,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  DSTATUS("It is only supported Z30 camera.");
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  CameraModule::TapZoomPosData tapZoomPos{x, y};
  memcpy(uData.dataTarget, &tapZoomPos, sizeof(tapZoomPos));

  /*!< set camera tap zoom multiplier parameter */
  DSTATUS("Set tap zoom multiplier = %d", multiplier);
  pm->setTapZoomMultiplierAsync(index, multiplier, setTapZoomMultiplierCb,
                                &uData);
}

void CameraManagerAsyncSample::startZoomAsyncSample(
    PayloadIndexType index, CameraModule::zoomDirectionData direction,
    CameraModule::zoomSpeedData speed,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  DSTATUS(
      "Attention : It is only supported by X5, X5R and X5S camera on Osmo with"
      "lens Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.");
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;

  /*!< stop camera shooting photo */
  DSTATUS("start to continuous zoom");
  pm->startContinuousOpticalZoomAsync(index, direction, speed, UserCallBack,
                                      userData);
}

void CameraManagerAsyncSample::stopZoomAsyncSample(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;

  /*!< stop camera shooting photo */
  DSTATUS("stop continuous zoom");
  pm->stopContinuousOpticalZoomAsync(index, UserCallBack, userData);
}

void CameraManagerAsyncSample::setCameraModeForRecordVideoCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set camera work mode successfully ");
    if (uData->pm) {
      /*! start to record video */
      DSTATUS("Start to RECORD_VIDEO");
      uData->pm->startRecordVideoAsync(
          uData->index,
          (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack,
          uData->userData);
    }
  } else {
    DERROR("Start to record video error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::startRecordVideoAsyncSample(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;

  /*!< set camera work mode as RECORD_VIDEO */
  DSTATUS("Set camera mode to RECORD_VIDEO");
  pm->setModeAsync(index, CameraModule::WorkMode::RECORD_VIDEO,
                   setCameraModeForRecordVideoCb, &uData);
}

void CameraManagerAsyncSample::stopRecordVideoAsyncSample(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;

  /*!< stop camera recording video */
  DSTATUS("stop recording video");
  pm->stopRecordVideoAsync(index, UserCallBack, userData);
}

void CameraManagerAsyncSample::setShootPhotoModeForSingleShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set shoot photo mode as Single successfully ");
    if (uData->pm) {
      /*! start to shoot SINGLE photo */
      uData->pm->startShootPhotoAsync(
          uData->index, CameraModule::ShootPhotoMode::SINGLE,
          (void (*)(ErrorCode::ErrorCodeType retCode,
                    UserData userData))uData->userCallBack,
          uData->userData);
    }
  } else {
    DERROR("Set shoot photo mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setCameraModeForSingleShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }

  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set camera work mode successfully ");
    if (uData->pm) {
      /*! start to shoot SINGLE photo */
      uData->pm->startShootPhotoAsync(
        uData->index, CameraModule::ShootPhotoMode::SINGLE,
        (void (*)(ErrorCode::ErrorCodeType retCode,
                  UserData userData))uData->userCallBack,
        uData->userData);
    }
  } else {
    DERROR("Set camera mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }

  /*! @TODO XT* and Z30 don't support set shoot-photo mode. To fix it in the
   * future */
  /*
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set camera work mode successfully ");
    if (uData->pm) {
      //set shoot-photo mode
      DSTATUS("set shoot-photo mode as SINGLE");
      DSTATUS("If the camera is XT, XT2, or XTS, set shoot-photo mode interface"
      " is temporarily not supported.");
      uData->pm->setShootPhotoModeAsync(
          uData->index, CameraModule::ShootPhotoMode::SINGLE,
          setShootPhotoModeForSingleShootCb, uData);
    }
  } else {
    DERROR("Set camera mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
  */
}

void CameraManagerAsyncSample::startShootSinglePhotoAsyncSample(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;

  /*!< set camera work mode as shoot photo */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  pm->setModeAsync(index, CameraModule::WorkMode::SHOOT_PHOTO,
                   setCameraModeForSingleShootCb, &uData);
}

void CameraManagerAsyncSample::setPhotoBurstCountCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set photo burst count successfully ");
    if (uData->pm) {
      /*! set BURST parameter */
      uData->pm->startShootPhotoAsync(
          uData->index, CameraModule::ShootPhotoMode::BURST,
          (void (*)(ErrorCode::ErrorCodeType retCode,
                    UserData userData))uData->userCallBack,
          uData->userData);
    }
  } else {
    DERROR("Set burst count error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setShootPhotoModeForBurstShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set shoot photo mode as BURST successfully ");
    if (uData->pm) {
      /*! set BURST parameter */
      uData->pm->setPhotoBurstCountAsync(
          uData->index, *(CameraModule::PhotoBurstCount *)uData->dataTarget,
          setPhotoBurstCountCb, uData);
    }
  } else {
    DERROR("Set shoot photo mode as BURST error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;

      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setCameraModeForBurstShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set camera work mode successfully ");
    if (uData->pm) {
      /*!< set shoot-photo mode */
      DSTATUS("set shoot-photo mode as BURST");
      uData->pm->setShootPhotoModeAsync(
          uData->index, CameraModule::ShootPhotoMode::BURST,
          setShootPhotoModeForBurstShootCb, uData);
    }
  } else {
    DERROR("Set camera mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::startShootBurstPhotoAsyncSample(
    PayloadIndexType index, CameraModule::PhotoBurstCount count,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &count, sizeof(count));

  /*!< set camera work mode as shoot photo */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  pm->setModeAsync(index, CameraModule::WorkMode::SHOOT_PHOTO,
                   setCameraModeForBurstShootCb, &uData);
}

void CameraManagerAsyncSample::setPhotoAEBCountCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set photo AEB count successfully ");
    if (uData->pm) {
      /*! set AEB parameter */
      uData->pm->startShootPhotoAsync(
          uData->index, CameraModule::ShootPhotoMode::AEB,
          (void (*)(ErrorCode::ErrorCodeType retCode,
                    UserData userData))uData->userCallBack,
          uData->userData);
    }
  } else {
    DERROR("Set AEB count error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setShootPhotoModeForAEBShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set shoot photo mode as AEB successfully ");
    if (uData->pm) {
      /*! set AEB parameter */
      uData->pm->setPhotoAEBCountAsync(
          uData->index, *(CameraModule::PhotoAEBCount *)uData->dataTarget,
          setPhotoAEBCountCb, uData);
    }
  } else {
    DERROR("Set shoot photo mode as AEB error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setCameraModeForAEBShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set camera work mode successfully ");
    if (uData->pm) {
      /*!< set shoot-photo mode */
      DSTATUS("set shoot-photo mode as AEB");
      uData->pm->setShootPhotoModeAsync(uData->index,
                                        CameraModule::ShootPhotoMode::AEB,
                                        setShootPhotoModeForAEBShootCb, uData);
    }
  } else {
    DERROR("Set camera mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::startShootAEBPhotoAsyncSample(
    PayloadIndexType index, CameraModule::PhotoAEBCount count,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &count, sizeof(count));

  /*!< set camera work mode as shoot photo */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  pm->setModeAsync(index, CameraModule::WorkMode::SHOOT_PHOTO,
                   setCameraModeForAEBShootCb, &uData);
}

void CameraManagerAsyncSample::setPhotoIntervalCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set time interval parameter successfully ");
    if (uData->pm) {
      /*! start to shoot interval photo */
      uData->pm->startShootPhotoAsync(
          uData->index, CameraModule::ShootPhotoMode::INTERVAL,
          (void (*)(ErrorCode::ErrorCodeType retCode,
                    UserData userData))uData->userCallBack,
          uData->userData);
    }
  } else {
    DERROR("Set time interval parameter error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setShootPhotoModeForIntervalShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set shoot photo mode as INTERVAL successfully ");
    if (uData->pm) {
      /*! set timer interval parameter */
      uData->pm->setPhotoTimeIntervalSettingsAsync(
          uData->index, *(CameraModule::PhotoIntervalData *)uData->dataTarget,
          setPhotoIntervalCb, uData);
    }
  } else {
    DERROR("Set shoot photo mode as INTERVAL error. Error code : 0x%lX",
           retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::setCameraModeForIntervalShootCb(
    ErrorCode::ErrorCodeType retCode, UserData userData) {
  AsyncSampleData *uData = (AsyncSampleData *)userData;

  DSTATUS("retCode : 0x%lX", retCode);
  if (!uData) {
    DERROR("User data is a null value.");
    return;
  }
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Set camera work mode successfully ");
    if (uData->pm) {
      /*!< set shoot-photo mode */
      DSTATUS("set shoot-photo mode as INTERVAL");
      uData->pm->setShootPhotoModeAsync(
          uData->index, CameraModule::ShootPhotoMode::INTERVAL,
          setShootPhotoModeForIntervalShootCb, uData);
    }
  } else {
    DERROR("Set camera mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    if (uData->userCallBack) {
      void (*cb)(ErrorCode::ErrorCodeType, UserData);
      cb = (void (*)(ErrorCode::ErrorCodeType, UserData))uData->userCallBack;
      cb(retCode, uData->userData);
    }
  }
}

void CameraManagerAsyncSample::startShootIntervalPhotoAsyncSample(
    PayloadIndexType index, CameraModule::PhotoIntervalData intervalData,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;
  static AsyncSampleData uData;
  uData.index = index;
  uData.pm = pm;
  uData.userCallBack = (void *)UserCallBack;
  uData.userData = userData;
  memcpy(uData.dataTarget, &intervalData, sizeof(intervalData));

  /*!< set camera work mode as shoot photo */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  pm->setModeAsync(index, CameraModule::WorkMode::SHOOT_PHOTO,
                   setCameraModeForIntervalShootCb, &uData);
}

void CameraManagerAsyncSample::stopShootPhotoAsyncSample(
    PayloadIndexType index,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::InstInitParamInvalid, userData);
    return;
  }
  CameraManager *pm = vehicle->cameraManager;

  /*!< stop camera shooting photo */
  DSTATUS("stop shooting photo");
  pm->stopShootPhotoAsync(index, UserCallBack, userData);
}
