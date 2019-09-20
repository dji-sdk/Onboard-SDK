/*! @file camera_manager_sync_sample.cpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief
 *  Demonstrate how to use the synchronous apis of camera manager.
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

#include "camera_manager_sync_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

CameraManagerSyncSample::CameraManagerSyncSample(Vehicle *vehiclePtr)
    : vehicle(vehiclePtr) {}

CameraManagerSyncSample::~CameraManagerSyncSample() {}

ErrorCode::ErrorCodeType CameraManagerSyncSample::setEVSyncSample(
    PayloadIndexType index, CameraModule::ExposureCompensation dataTarget) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;
  CameraModule::ExposureCompensation evGet;

  retCode = pm->getExposureCompensationSync(index, evGet, 1);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get ev = %d", evGet);
    if (dataTarget != evGet) {
      DSTATUS("Set evTarget = %d", dataTarget);
      retCode = pm->setExposureCompensationSync(index, dataTarget, 1);
      if (retCode == ErrorCode::SysCommonErr::Success) {
        DSTATUS("Set ev value successfully.");
      } else {
        DERROR("Set ev parameter error. Error code : 0x%lX", retCode);
        ErrorCode::printErrorCodeMsg(retCode);
        DERROR(
            "In order to use this function, the camera exposure mode should be "
            "set to be PROGRAM_AUTO, SHUTTER_PRIORITY or APERTURE_PRIORITY "
            "first");
      }
    } else {
      DSTATUS("The ev value is already %d.", dataTarget);
    }
  } else {
    DERROR("Get ev error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::setExposureModeSyncSample(
    PayloadIndexType index, CameraModule::ExposureMode dataTarget) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;
  CameraModule::ExposureMode exposureModeGet;

  retCode = pm->getExposureModeSync(index, exposureModeGet, 1);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get exposure mode = %d", exposureModeGet);
    if (dataTarget != exposureModeGet) {
      DSTATUS("Set exposure mode = %d", dataTarget);
      retCode = pm->setExposureModeSync(index, dataTarget, 1);
      if (retCode == ErrorCode::SysCommonErr::Success) {
        DSTATUS("Set exposure mode successfully.");
      } else {
        DERROR("Set exposure mode error. Error code : 0x%lX", retCode);
        ErrorCode::printErrorCodeMsg(retCode);
      }
    } else {
      DSTATUS("The exposure mode is already %d.", dataTarget);
    }
  } else {
    DERROR("Get exposure mode error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::setISOSyncSample(
    PayloadIndexType index, CameraModule::ISO dataTarget) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;
  CameraModule::ISO isoGet;

  retCode = pm->getISOSync(index, isoGet, 1);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get iso = %d", isoGet);
    if (dataTarget != isoGet) {
      DSTATUS("Set iso = %d", dataTarget);
      retCode = pm->setISOSync(index, dataTarget, 1);
      if (retCode == ErrorCode::SysCommonErr::Success) {
        DSTATUS("Set iso successfully");
      } else {
        DERROR("Set ISO parameter error. Error code : 0x%lX", retCode);
        ErrorCode::printErrorCodeMsg(retCode);
        DERROR(
            "For the X5, X5R, X4S and X5S, the ISO value can be set for all "
            "modes. For the other cameras, the ISO value can only be set when "
            "the camera exposure mode is in Manual mode.");
      }
    } else {
      DSTATUS("The iso parameter is already %d.", dataTarget);
    }
  } else {
    DERROR("Get iso error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::setShutterSpeedSyncSample(
    PayloadIndexType index, CameraModule::ShutterSpeed dataTarget) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;
  CameraModule::ShutterSpeed shutterSpeedGet;

  retCode = pm->getShutterSpeedSync(index, shutterSpeedGet, 1);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get shutterSpeed = %d", shutterSpeedGet);
    if (dataTarget != shutterSpeedGet) {
      DSTATUS("Set shutterSpeed = %d", dataTarget);
      retCode = pm->setShutterSpeedSync(index, dataTarget, 1);
      if (retCode == ErrorCode::SysCommonErr::Success) {
        DSTATUS("Set iso successfully");
      } else {
        DERROR("Set shutterSpeed parameter error. Error code : 0x%lX", retCode);
        ErrorCode::printErrorCodeMsg(retCode);
        DERROR(
            "The shutter speed can be set only when the camera exposure mode "
            "is Shutter mode or Manual mode. The shutter speed should not be "
            "set slower than the video frame rate when the camera's mode is "
            "RECORD_VIDEO.");
      }
    } else {
      DSTATUS("The shutterSpeed is already %d.", dataTarget);
    }
  } else {
    DERROR("Get shutterSpeed error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::setApertureSyncSample(
    PayloadIndexType index, CameraModule::Aperture dataTarget) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;
  CameraModule::Aperture apertureGet;

  retCode = pm->getApertureSync(index, apertureGet, 1);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Get aperture = %d", apertureGet);
    if (dataTarget != apertureGet) {
      DSTATUS("Set aperture = %d", dataTarget);
      retCode = pm->setApertureSync(index, dataTarget, 1);
      if (retCode == ErrorCode::SysCommonErr::Success) {
        DSTATUS("Set aperture successfully");
      } else {
        DERROR("Set aperture parameter error. Error code : 0x%lX", retCode);
        ErrorCode::printErrorCodeMsg(retCode);
        DERROR(
            "In order to use this function, the exposure mode ExposureMode "
            "must be in MANUAL or APERTURE_PRIORITY. Supported only by the X5, "
            "X5R, X4S, X5S camera.");
      }
    } else {
      DSTATUS("The aperture is already %d.", dataTarget);
    }
  } else {
    DERROR("Get aperture error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::setFocusPointSyncSample(
    PayloadIndexType index, float x, float y) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< set camera focus mode to be CameraModule::FocusMode::AUTO */
  DSTATUS("Set focus mode = %d", CameraModule::FocusMode::AUTO);
  retCode = pm->setFocusModeSync(index, CameraModule::FocusMode::AUTO, 1);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    /*!< set camera focus point */
    DSTATUS("Set focus point = (%f,%f)", x, y);
    retCode = pm->setFocusTargetSync(index, {x, y}, 1);
    if (retCode == ErrorCode::SysCommonErr::Success) {
      DSTATUS("Set focus point successfully");
    } else {
      DERROR("Set focus point error. Error code : 0x%lX", retCode);
      ErrorCode::printErrorCodeMsg(retCode);
      DERROR(
        "When the focus mode is auto, the target point is the focal point. "
        "When the focus mode is manual, the target point is the zoom out area "
        "if the focus assistant is enabled for the manual mode. Supported only "
        "by the X5, X5R, Z3 cameras, Mavic Pro camera, Phantom 4 Pro camera, "
        "Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise Camera, X5S. "
        "It's should be attention that X4S will keep focus point as (0.5,0.5) "
        "all the time, the setting of focus point to X4S will quickly replaced "
        "by (0.5, 0.5).");
    }
  } else {
    DERROR("Set focus mode parameter error. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    DERROR(
        "When the focus mode is auto, the target point is the focal point. "
        "When the focus mode is manual, the target point is the zoom out area "
        "if the focus assistant is enabled for the manual mode. Supported only "
        "by the X5, X5R, Z3 cameras, Mavic Pro camera, Phantom 4 Pro camera, "
        "Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise Camera, X5S. "
        "It's should be attention that X4S will keep focus point as (0.5,0.5) "
        "all the time, the setting of focus point to X4S will quickly replaced "
        "by (0.5, 0.5).");
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::setTapZoomPointSyncSample(
    PayloadIndexType index, uint8_t multiplier, float x, float y) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< set camera tap zoom enable parameter to be enable */
  DSTATUS("Set tap zoom enable  = %d", true);
  retCode = pm->setTapZoomEnabledSync(index, true, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set tap zoom enable fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    DERROR("It is only supported Z30 camera.");
    return retCode;
  }

  /*!< set camera tap zoom multiplier parameter */
  DSTATUS("Set tap zoom multiplier = %d", multiplier);
  retCode = pm->setTapZoomMultiplierSync(index, multiplier, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set tap zoom multiplier fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    DERROR("It is only supported Z30 camera.");
    return retCode;
  }

  /*!< set camera tap zoom multiplier target point */
  DSTATUS("Set tap zoom target point : (%f,%f)", x, y);
  retCode = pm->tapZoomAtTargetSync(index, {x, y}, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set tap zoom target fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    DERROR("It is only supported Z30 camera.");
    return retCode;
  } else {
    DSTATUS(
        "tap zoom at target (%0.2f, %0.2f) successfully, need several seconds "
        "to zoom.",
        x, y);
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::startZoomSyncSample(
    PayloadIndexType index, CameraModule::zoomDirectionData direction,
    CameraModule::zoomSpeedData speed) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  DSTATUS(
      "Attention : It is only supported by X5, X5R and X5S camera on Osmo with"
      "lens Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.");

  DSTATUS("Start continuous optical zoom parameters : direction=%d, speed=%d",
          direction, speed);
  retCode = pm->startContinuousOpticalZoomSync(index, direction, speed, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Start continuous zoom fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::stopZoomSyncSample(
    PayloadIndexType index) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  DSTATUS("Stop continuous optical zoom.");
  retCode = pm->stopContinuousOpticalZoomSync(index, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Stop continuous zoom fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
  return retCode;
}

ErrorCode::ErrorCodeType
CameraManagerSyncSample::startShootSinglePhotoSyncSample(
    PayloadIndexType index) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< set camera work mode as shoot photo */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Camera take photo fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*! @TODO XT* and Z30 don't support set shoot-photo mode. To fix it in the
   * future */
  /*!< set shoot-photo mode
  DSTATUS("set shoot-photo mode as SINGLE");
  retCode =
      pm->setShootPhotoModeSync(index, CameraModule::ShootPhotoMode::SINGLE, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set shoot-photo mode as SINGLE fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    DERROR("If the camera is XT, XT2, or XTS, set shoot-photo mode interface is"
           "temporarily not supported.");
    return retCode;
  }
  */

  /*! wait the APP change the shoot-photo mode display */
  vehicle->getPlatformManager()->millisecSleep(500);

  /*!< start to shoot single photo */
  DSTATUS("start to shoot SINGLE photo");
  retCode =
      pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::SINGLE, 2);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Take SINGLE photo fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

ErrorCode::ErrorCodeType
CameraManagerSyncSample::startShootBurstPhotoSyncSample(
    PayloadIndexType index, CameraModule::PhotoBurstCount count) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< set camera work mode as SHOOT_PHOTO */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set camera as SHOOT_PHOTO fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*!< set shoot-photo mode */
  DSTATUS("set shoot-photo mode as BURST");
  retCode =
      pm->setShootPhotoModeSync(index, CameraModule::ShootPhotoMode::BURST, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set shoot-photo mode as BURST fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*! wait the APP change the shoot-photo mode display */
  vehicle->getPlatformManager()->millisecSleep(500);

  /*!< set shoot-photo mode parameter */
  DSTATUS("set count = %d", count);
  retCode = pm->setPhotoBurstCountSync(index, count, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set the parameter of BURST mode fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*!< start to shoot BURST photo */
  DSTATUS("start to shoot BURST photo");
  retCode =
      pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::BURST, 2);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Take BURST photo fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::startShootAEBPhotoSyncSample(
    PayloadIndexType index, CameraModule::PhotoAEBCount photoNum) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< set camera work mode as SHOOT_PHOTO */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set camera as SHOOT_PHOTO fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*!< set shoot-photo mode */
  DSTATUS("set shoot-photo mode as AEB");
  retCode =
      pm->setShootPhotoModeSync(index, CameraModule::ShootPhotoMode::AEB, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set shoot-photo mode as AEB fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*! wait the APP change the shoot-photo mode display */
  vehicle->getPlatformManager()->millisecSleep(500);

  /*!< set shoot-photo mode parameter */
  DSTATUS("set AEB photo number = %d", photoNum);
  retCode = pm->setPhotoAEBCountSync(index, photoNum, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set the parameter of AEB mode fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*!< start to shoot AEB photo */
  DSTATUS("start to shoot AEB photo");
  retCode =
      pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::AEB, 2);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Take AEB photo fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

ErrorCode::ErrorCodeType
CameraManagerSyncSample::startShootIntervalPhotoSyncSample(
    PayloadIndexType index, CameraModule::PhotoIntervalData intervalData) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< set camera work mode as SHOOT_PHOTO */
  DSTATUS("set camera work mode as SHOOT_PHOTO");
  retCode = pm->setModeSync(index, CameraModule::WorkMode::SHOOT_PHOTO, 2);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set camera as SHOOT_PHOTO fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*!< set shoot-photo mode */
  DSTATUS("set shoot-photo mode as INTERVAL");
  retCode = pm->setShootPhotoModeSync(
      index, CameraModule::ShootPhotoMode::INTERVAL, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set shoot-photo mode as INTERVAL fail. Error code : 0x%lX",
           retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*! wait the APP change the shoot-photo mode display */
  vehicle->getPlatformManager()->millisecSleep(500);

  /*!< set shoot-photo mode parameter */
  DSTATUS("set intervalData : photoNumConticap = %d ,timeInterval = %d",
          intervalData.photoNumConticap, intervalData.timeInterval);
  retCode = pm->setPhotoTimeIntervalSettingsSync(index, intervalData, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set the parameter of INTERVAL mode fail. Error code : 0x%lX",
           retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*!< start to shoot INTERVAL photo */
  DSTATUS("start to shoot INTERVAL photo");
  retCode =
      pm->startShootPhotoSync(index, CameraModule::ShootPhotoMode::INTERVAL, 2);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Take INTERVAL photo fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::shootPhotoStopSyncSample(
    PayloadIndexType index) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< stop camera shooting photo */
  DSTATUS("Stop to shoot photo");
  retCode = pm->stopShootPhotoSync(index, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("stop camera shooting photo fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::startRecordVideoSyncSample(
    PayloadIndexType index) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< set camera work mode as RECORD_VIDEO */
  DSTATUS("Set camera mode to RECORD_VIDEO");
  retCode = pm->setModeSync(index, CameraModule::WorkMode::RECORD_VIDEO, 2);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Set camera as RECORD_VIDEO mode fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  /*!< start to take video */
  DSTATUS("Start to RECORD_VIDEO");
  retCode = pm->startRecordVideoSync(index, 1);
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Start to record video fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}

ErrorCode::ErrorCodeType CameraManagerSyncSample::stopRecordVideoSyncSample(
    PayloadIndexType index) {
  if (!vehicle || !vehicle->cameraManager) {
    DERROR("vehicle or cameraManager is a null value.");
    return ErrorCode::SysCommonErr::InstInitParamInvalid;
  }
  ErrorCode::ErrorCodeType retCode;
  CameraManager *pm = vehicle->cameraManager;

  /*!< stop to take video */
  retCode = pm->stopRecordVideoSync(index, 1);
  DSTATUS("Stop RECORD_VIDEO");
  if (retCode != ErrorCode::SysCommonErr::Success) {
    DERROR("Stop to record video fail. Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
    return retCode;
  }

  return retCode;
}
