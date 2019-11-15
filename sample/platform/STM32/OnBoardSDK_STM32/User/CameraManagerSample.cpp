/*! @file CameraManagerSample.cpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief Tesf for the CameraManager. All tests are basic on callback method.
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

#include "CameraManagerSample.hpp"
#include "camera_manager_async_sample.hpp"
#include "timer.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void asyncSampleCallBack(ErrorCode::ErrorCodeType retCode, UserData SampleLog) {
  DSTATUS("retCode : 0x%lX", retCode);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    DSTATUS("Pass : %s.", SampleLog);
  } else {
    DERROR("Error : %s. Error code : %d", SampleLog, retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}

void callbackToSetShutterSpeed(ErrorCode::ErrorCodeType retCode,
                               UserData userData) {
  DSTATUS("retCode : 0x%lX", retCode);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    CameraManagerAsyncSample *p = (CameraManagerAsyncSample *)userData;
    if (p)
      p->setShutterSpeedAsyncSample(
          PAYLOAD_INDEX_0, CameraModule::SHUTTER_SPEED_1_50,
          asyncSampleCallBack, (UserData) "Set exposure mode");
  } else {
    DERROR("Set exposure mode failure, Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}

void callbackToSetAperture(ErrorCode::ErrorCodeType retCode, UserData userData) {
  DSTATUS("retCode : 0x%lX", retCode);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    CameraManagerAsyncSample *p = (CameraManagerAsyncSample *)userData;
    if (p)
      p->setApertureAsyncSample(PAYLOAD_INDEX_0, CameraModule::F_3_DOT_5,
                                asyncSampleCallBack,
                                (UserData) "Set camera aperture");
  } else {
    DERROR("Set exposure mode failure, Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}

void callbackToSetISO(ErrorCode::ErrorCodeType retCode, UserData userData) {
  DSTATUS("retCode : 0x%lX", retCode);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    CameraManagerAsyncSample *p = (CameraManagerAsyncSample *)userData;
    if (p)
      p->setISOAsyncSample(PAYLOAD_INDEX_0, CameraModule::ISO_200,
                           asyncSampleCallBack, (UserData) "Set camera ISO");
  } else {
    DERROR("Set exposure mode failure, Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}

void callbackToSetExposureCompensation(ErrorCode::ErrorCodeType retCode,
                                       UserData userData) {
  DSTATUS("retCode : 0x%lX", retCode);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    CameraManagerAsyncSample *p = (CameraManagerAsyncSample *)userData;
    if (p)
      p->setEVAsyncSample(PAYLOAD_INDEX_0, CameraModule::P_0_3,
                          asyncSampleCallBack,
                          (UserData) "Set camera EV(exposure compensation)");
  } else {
    DERROR("Set exposure mode failure, Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}

int cameraManagerTest(Vehicle *vehicle, CameraManagerTestCase testCase) {
  static CameraManagerAsyncSample *p = NULL;

  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting. \n");
    return -1;
  }
  if (testCase >= UNKNOWN_TEST_CASE) {
    DERROR("Unknown test case. \n");
    return -1;
  }
  if (!p) {
    ErrorCode::ErrorCodeType ret = vehicle->cameraManager->initCameraModule(
        PAYLOAD_INDEX_0, "sample_camera_1");
    ret |= vehicle->cameraManager->initCameraModule(PAYLOAD_INDEX_1,
                                                    "sample_camera_2");
    if (ret != ErrorCode::SysCommonErr::Success) {
      DERROR("create payload node error\n");
      return -1;
    }
    p = new CameraManagerAsyncSample(vehicle);
    if (!p) {
      DERROR("Create CameraManagerAsyncSample error\n");
      return -1;
    }
  }

  switch (testCase) {
    case X5S_AT_PAYLOAD_0:
      /*! Set shutter speed parameter test */
      DSTATUS("Set shutter speed parameter for X5S");
      p->setExposureModeAsyncSample(PAYLOAD_INDEX_0,
                                    CameraModule::SHUTTER_PRIORITY,
                                    callbackToSetShutterSpeed, (UserData)p);
      delay_nms(2000);

      /*! Set aperture parameter test */
      DSTATUS("Set aperture parameter for X5S");
      p->setExposureModeAsyncSample(PAYLOAD_INDEX_0,
                                    CameraModule::APERTURE_PRIORITY,
                                    callbackToSetAperture, (UserData)p);
      delay_nms(2000);

      /*! Set exposure compensation parameter test */
      DSTATUS("Set exposure compensation parameter for X5S");
      p->setExposureModeAsyncSample(PAYLOAD_INDEX_0, CameraModule::PROGRAM_AUTO,
                                    callbackToSetExposureCompensation,
                                    (UserData)p);
      delay_nms(2000);

      /*! Set ISO parameter test */
      DSTATUS("Set ISO parameter for X5S");
      p->setExposureModeAsyncSample(PAYLOAD_INDEX_0,
                                    CameraModule::EXPOSURE_MANUAL,
                                    callbackToSetISO, (UserData)p);
      delay_nms(2000);

      /*! Set focus point test */
      DSTATUS("Set focus point for X5S");
      p->setFocusPointAsyncSample(PAYLOAD_INDEX_0, 0.8, 0.8,
                                  asyncSampleCallBack,
                                  (UserData) "set focus point");
      delay_nms(2000);

      /*! Shoot photo test */
      DSTATUS("Start to shoot single photo");
      p->startShootSinglePhotoAsyncSample(
          PAYLOAD_INDEX_0, asyncSampleCallBack,
          (UserData) "start to shoot single photo");
      delay_nms(5000);
      DSTATUS("Start to shoot AEB photo");
      p->startShootAEBPhotoAsyncSample(
          PAYLOAD_INDEX_0, CameraModule::AEB_COUNT_5, asyncSampleCallBack,
          (UserData) "start to shoot AEB photos");
      delay_nms(8000);
      DSTATUS("Start to shoot Burst photo");
      p->startShootBurstPhotoAsyncSample(
          PAYLOAD_INDEX_0, CameraModule::BURST_COUNT_7, asyncSampleCallBack,
          (UserData) "start to shoot burst photos");
      delay_nms(10000);
      DSTATUS("Start to shoot Interval photo");
      CameraModule::PhotoIntervalData intervalData;
      intervalData.photoNumConticap = 255;
      intervalData.timeInterval = 4;
      p->startShootIntervalPhotoAsyncSample(
          PAYLOAD_INDEX_0, intervalData, asyncSampleCallBack,
          (UserData) "start to shoot interval photos");
      DSTATUS("Sleep 15 seconds");
      delay_nms(15000);
      p->stopShootPhotoAsyncSample(PAYLOAD_INDEX_0, asyncSampleCallBack,
                                   (UserData) "stop shooting interval photos");
      break;

    case Z30_AT_PAYLOAD_1:
      /*! take video test */
      DSTATUS("Test video function on Z30");
      p->startRecordVideoAsyncSample(PAYLOAD_INDEX_1, asyncSampleCallBack,
                                     (UserData) "start to record video");
      delay_nms(2000);

      /*! tap zoom test */
      DSTATUS("Test tap-zoom function on Z30");
      p->setTapZoomPointAsyncSample(PAYLOAD_INDEX_1, 5, 0.3, 0.3,
                                    asyncSampleCallBack,
                                    (UserData) "set tap zoom point (0.3, 0.3)");
      delay_nms(5000);
      p->setTapZoomPointAsyncSample(PAYLOAD_INDEX_1, 5, 0.8, 0.7,
                                    asyncSampleCallBack,
                                    (UserData) "set tap zoom point (0.8, 0.7)");
      delay_nms(5000);

      /*! zoom test */
      DSTATUS("Test zoom function on Z30");
      p->startZoomAsyncSample(PAYLOAD_INDEX_1, CameraModule::ZOOM_IN,
                              CameraModule::NORMAL, asyncSampleCallBack,
                              (UserData) "start continuous zoom");
      delay_nms(4000);
      p->stopZoomAsyncSample(PAYLOAD_INDEX_1, asyncSampleCallBack,
                             (UserData) "stop continuous zoom");
      delay_nms(2000);
      p->startZoomAsyncSample(PAYLOAD_INDEX_1, CameraModule::ZOOM_OUT,
                              CameraModule::FASTEST, asyncSampleCallBack,
                              (UserData) "start continuous zoom");
      delay_nms(8000);
      p->stopZoomAsyncSample(PAYLOAD_INDEX_1, asyncSampleCallBack,
                             (UserData) "stop continuous zoom");

      /*! take video finished */
      p->stopRecordVideoAsyncSample(PAYLOAD_INDEX_1, asyncSampleCallBack,
                                    (UserData) "stop recording video");
      break;

    default:
      break;
  }
  delete p;
  return 0;
}
