/*! @file payloads/main_async.cpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief
 *  main for CameraManager usage in a Linux environment.
 *  Shows example usage of CameraManager asynchronous APIs.
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

#include <dji_linux_helpers.hpp>
#include "camera_manager_async_sample.hpp"

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
          PAYLOAD_INDEX_0, CameraModule::ShutterSpeed::SHUTTER_SPEED_1_50,
          asyncSampleCallBack, (UserData) "Set exposure mode");
  } else {
    DERROR("Set exposure mode failure, Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}

void callbackToSetAperture(ErrorCode::ErrorCodeType retCode,
                           UserData userData) {
  DSTATUS("retCode : 0x%lX", retCode);
  if (retCode == ErrorCode::SysCommonErr::Success) {
    CameraManagerAsyncSample *p = (CameraManagerAsyncSample *)userData;
    if (p)
      p->setApertureAsyncSample(
          PAYLOAD_INDEX_0, CameraModule::Aperture::F_3_DOT_5,
          asyncSampleCallBack, (UserData) "Set camera aperture");
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
      p->setISOAsyncSample(PAYLOAD_INDEX_0, CameraModule::ISO::ISO_200,
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
      p->setEVAsyncSample(PAYLOAD_INDEX_0,
                          CameraModule::ExposureCompensation::P_0_3,
                          asyncSampleCallBack,
                          (UserData) "Set camera EV(exposure compensation)");
  } else {
    DERROR("Set exposure mode failure, Error code : 0x%lX", retCode);
    ErrorCode::printErrorCodeMsg(retCode);
  }
}

int main(int argc, char **argv) {
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting. \n";
    return -1;
  }
  std::string sampleCase = linuxEnvironment.getEnvironment()->getSampleCase();

  /*! init camera modules for cameraManager */
  ErrorCode::ErrorCodeType ret = vehicle->cameraManager->initCameraModule(
      PAYLOAD_INDEX_0, "Sample_camera_1");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera module Sample_camera_1 failed.");
    ErrorCode::printErrorCodeMsg(ret);
  }

  ret = vehicle->cameraManager->initCameraModule(PAYLOAD_INDEX_1,
                                                 "Sample_camera_2");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init Camera module Sample_camera_2 failed.");
    ErrorCode::printErrorCodeMsg(ret);
  }

  CameraManagerAsyncSample *p = new CameraManagerAsyncSample(vehicle);
  bool sampleCaseValidFlag = false;
  char inputChar = 0;
  if (sampleCase.size() == 1) {
    if ((sampleCase <= "l") && (sampleCase >= "a")) {
      inputChar = sampleCase[0];
    } else {
      inputChar = 0;
      DERROR("sample_case is an invalid string !");
      sleep(2);
    }
  }

  /*! sample loop start */
  while (true) {
    std::cout << std::endl;
    std::cout
        << "| Available commands:                                            |"
        << std::endl
        << "| [a] Set camera shutter speed                                   |"
        << std::endl
        << "| [b] Set camera aperture                                        |"
        << std::endl
        << "| [c] Set camera EV value                                        |"
        << std::endl
        << "| [d] Set camera ISO value                                       |"
        << std::endl
        << "| [e] Set camera focus point                                     |"
        << std::endl
        << "| [f] Set camera tap zoom point                                  |"
        << std::endl
        << "| [g] Set camera zoom parameter                                  |"
        << std::endl
        << "| [h] Shoot Single photo Sample                                  |"
        << std::endl
        << "| [i] Shoot AEB photo Sample                                     |"
        << std::endl
        << "| [j] Shoot Burst photo Sample                                   |"
        << std::endl
        << "| [k] Shoot Interval photo Sample                                |"
        << std::endl
        << "| [l] Record video Sample                                        |"
        << std::endl
        << "| [q] Quit                                                       |"
        << std::endl;

    if (inputChar != 0) {
      sampleCaseValidFlag = true;
      DSTATUS("Get inputChar from argv, case [%c] will be executed", inputChar);
      sleep(3);
    } else {
      std::cin >> inputChar;
    }

    switch (inputChar) {
      case 'a':
        p->setExposureModeAsyncSample(
            PAYLOAD_INDEX_0, CameraModule::ExposureMode::SHUTTER_PRIORITY,
            callbackToSetShutterSpeed, (UserData)p);
        sleep(2);
        break;
      case 'b':
        p->setExposureModeAsyncSample(
            PAYLOAD_INDEX_0, CameraModule::ExposureMode::APERTURE_PRIORITY,
            callbackToSetAperture, (UserData)p);
        sleep(2);
        break;
      case 'c':
        p->setExposureModeAsyncSample(
            PAYLOAD_INDEX_0, CameraModule::ExposureMode::PROGRAM_AUTO,
            callbackToSetExposureCompensation, (UserData)p);
        sleep(2);
        break;
      case 'd':
        p->setExposureModeAsyncSample(
            PAYLOAD_INDEX_0, CameraModule::ExposureMode::EXPOSURE_MANUAL,
            callbackToSetISO, (UserData)p);
        sleep(2);
        break;
      case 'e':
        p->setFocusPointAsyncSample(PAYLOAD_INDEX_0, 0.8, 0.8,
                                    asyncSampleCallBack,
                                    (UserData) "set focus point");
        break;
      case 'f':
        p->setTapZoomPointAsyncSample(
            PAYLOAD_INDEX_1, 5, 0.3, 0.3, asyncSampleCallBack,
            (UserData) "set tap zoom point (0.3, 0.3)");
        sleep(5);
        p->setTapZoomPointAsyncSample(
            PAYLOAD_INDEX_1, 5, 0.8, 0.7, asyncSampleCallBack,
            (UserData) "set tap zoom point (0.8, 0.7)");
        sleep(5);
        break;
      case 'g':
        p->startZoomAsyncSample(
            PAYLOAD_INDEX_1, CameraModule::ZoomDirection::ZOOM_IN,
            CameraModule::ZoomSpeed::NORMAL, asyncSampleCallBack,
            (UserData) "start continuous zoom");
        sleep(4);
        p->stopZoomAsyncSample(PAYLOAD_INDEX_1, asyncSampleCallBack,
                               (UserData) "stop continuous zoom");
        sleep(2);
        p->startZoomAsyncSample(
            PAYLOAD_INDEX_1, CameraModule::ZoomDirection::ZOOM_OUT,
            CameraModule::ZoomSpeed::FASTEST, asyncSampleCallBack,
            (UserData) "start continuous zoom");
        sleep(8);
        p->stopZoomAsyncSample(PAYLOAD_INDEX_1, asyncSampleCallBack,
                               (UserData) "stop continuous zoom");
        break;
      case 'h':
        p->startShootSinglePhotoAsyncSample(
            PAYLOAD_INDEX_0, asyncSampleCallBack,
            (UserData) "start to shoot single photo");
        break;
      case 'i':
        p->startShootAEBPhotoAsyncSample(
            PAYLOAD_INDEX_0, CameraModule::PhotoAEBCount::AEB_COUNT_5,
            asyncSampleCallBack, (UserData) "start to shoot AEB photos");
        break;
      case 'j':
        p->startShootBurstPhotoAsyncSample(
            PAYLOAD_INDEX_0, CameraModule::PhotoBurstCount::BURST_COUNT_7,
            asyncSampleCallBack, (UserData) "start to shoot burst photos");
        break;
      case 'k':
        CameraModule::PhotoIntervalData intervalData;
        intervalData.photoNumConticap = 255;
        intervalData.timeInterval = 4;
        p->startShootIntervalPhotoAsyncSample(
            PAYLOAD_INDEX_0, intervalData, asyncSampleCallBack,
            (UserData) "start to shoot interval photos");
        DSTATUS("Sleep 15 seconds");
        sleep(15);
        p->stopShootPhotoAsyncSample(
            PAYLOAD_INDEX_0, asyncSampleCallBack,
            (UserData) "stop shooting interval photos");
        break;
      case 'l':
        p->startRecordVideoAsyncSample(PAYLOAD_INDEX_0, asyncSampleCallBack,
                                       (UserData) "start to record video");
        sleep(10);
        p->stopRecordVideoAsyncSample(PAYLOAD_INDEX_0, asyncSampleCallBack,
                                      (UserData) "stop recording video");
        break;
      case 'q':
        DSTATUS("Quit now ...");
        delete p;
        return 0;
      default:
        break;
    }

    DSTATUS("Sample end ...");
    sleep(2);
    if (sampleCaseValidFlag) break;
    inputChar = 0;
  }
  delete p;
  return 0;
}