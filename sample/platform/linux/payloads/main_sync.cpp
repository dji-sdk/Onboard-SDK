/*! @file payloads/main_sync.cpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief
 *  main for CameraManager usage in a Linux environment.
 *  Shows example usage of CameraManager synchronous APIs.
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
#include "camera_manager_sync_sample.hpp"
#include "gimbal_manager_sync_sample.hpp"
#include <map>

#define LOG_NAME_INT(DATA) std::make_pair((int)DATA, #DATA)

static const std::pair<const int, const char *> orientationPairs[] = {
    {LOG_NAME_INT(CameraOrientation::DEFAULT)},
    {LOG_NAME_INT(CameraOrientation::CW90)},
    {LOG_NAME_INT(CameraOrientation::CW180)},
    {LOG_NAME_INT(CameraOrientation::CW270)},
    {LOG_NAME_INT(CameraOrientation::UNKNOWN)},
};

static const std::pair<const int, const char *> videoResolutionPairs[] = {
    {LOG_NAME_INT(VideoResolution::RESOLUTION_640X480P)},  //0, // 640X480P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_640X480I)},  //1, // 640X480I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1280X640P)},  //2, // 1280X640P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1280X640I)},  //3, // 1280X640I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1280X720P)},  //4, // 1280X720P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1280X720I)},  //5, // 1280X720I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1280X960P)},  //6, // 1280X960P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1280X960I)},  //7, // 1280X960I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1920X960P)},  //8, // 1920X960P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1920X960I)},  //9, // 1920X960I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1920X1080P)},  //10, // 1920X1080P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1920X1080I)},  //11, // 1920X1080I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1920X1440P)},  //12, // 1920X1440P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1920X1440I)},  //13, // 1920X1440I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X1920P)},  //14, // 3840X1920P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X1920I)},  //15, // 3840X1920I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X2160P)},  //16, // 3840X2160P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X2160I)},  //17, // 3840X2160I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X2880P)},  //18, // 3840X2880P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X2280I)},  //19, // 3840X2280I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4096X2048P)},  //20, // 4096X2048P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4960X2048I)},  //21, // 4960X2048I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4096X2160P)},  //22, // 4096X2160P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4096X2160I)},  //23, // 4096X2160I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2704X1520P_16COLON9)},  //24, // 2704X1520P,16:9
    {LOG_NAME_INT(VideoResolution::RESOLUTION_640X512P_FLIR)},  //26, // 640X512P_FLIR
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4608X2160)},  //27, // 4608X2160
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4608X2592)},  //28, // 4608X2592
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2720X1530P)},  //31, // 2720X1530P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_5280X2160P)},  //32, // 5280X2160P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_5280X2970P)},  //33, // 5280X2970P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X1572P)},  //34, // 3840X1572P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_5760X3240P)},  //35, // 5760X3240P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_6016X3200P)},  //36, // 6016X3200P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2048X1080P)},  //37, // 2048X1080P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_336X256P_FLIR)},  //38, // 336X256P_FLIR
    {LOG_NAME_INT(VideoResolution::RESOLUTION_5120X2880P)},  //39, // 5120x2880P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4096X2160P_RAW14)},  //40, // 4096X2160P_RAW14
    {LOG_NAME_INT(VideoResolution::RESOLUTION_3840X2160P_RAW14)},  //41, // 3840X2160P_RAW14
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2720X1530P_RAW14)},  //42, // 2720X1530P_RAW14
    {LOG_NAME_INT(VideoResolution::RESOLUTION_1920X1080P_RAW14)},  //43, // 1920X1080P_RAW14
    {LOG_NAME_INT(VideoResolution::RESOLUTION_5440X2880P)},  //44, // 5440X2880P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2688X1512P)},  //45, // 2688X1512P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_640X360P)},  //46, // 640X360P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4000X3000P)},  //48, // 4000X3000P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_4000X3000I)},  //49, // 4000X3000I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2880X1620P)},  //50, // 2880X1620P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2880X1620I)},  //51, // 2880X1620I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2720X2040P)},  //52, // 2720X2040P
    {LOG_NAME_INT(VideoResolution::RESOLUTION_2720X2040I)},  //53, // 2720X2040I
    {LOG_NAME_INT(VideoResolution::RESOLUTION_720X576)},  //54, // 720X576
    {LOG_NAME_INT(VideoResolution::RESOLUTION_MAX)},  //253, // MAX
    {LOG_NAME_INT(VideoResolution::RESOLUTION_UNSET)},  //254, // UNSET
    {LOG_NAME_INT(VideoResolution::RESOLUTION_KNOWN)},  //255, // KNOWN
};

static const std::pair<const int, const char *> videoFrameRatePairs[] = {
    {LOG_NAME_INT(VideoFrameRate::RATE_15FPS)},  //0,
    {LOG_NAME_INT(VideoFrameRate::RATE_24FPS)},  //1,
    {LOG_NAME_INT(VideoFrameRate::RATE_25FPS)},  //2,
    {LOG_NAME_INT(VideoFrameRate::RATE_30FPS)},  //3,
    {LOG_NAME_INT(VideoFrameRate::RATE_48FPS)},  //4,
    {LOG_NAME_INT(VideoFrameRate::RATE_50FPS)},  //5,
    {LOG_NAME_INT(VideoFrameRate::RATE_60FPS)},  //6,
    {LOG_NAME_INT(VideoFrameRate::RATE_120FPS)},  //7,
    {LOG_NAME_INT(VideoFrameRate::RATE_240FPS)},  //8,
    {LOG_NAME_INT(VideoFrameRate::RATE_480FPS)},  //9,
    {LOG_NAME_INT(VideoFrameRate::RATE_100PS)},  //10,
    {LOG_NAME_INT(VideoFrameRate::RATE_96FPS)},  //11,
    {LOG_NAME_INT(VideoFrameRate::RATE_180FPS)},  //12,
    {LOG_NAME_INT(VideoFrameRate::RATE_TRUE24FPS)},  //13,
    {LOG_NAME_INT(VideoFrameRate::RATE_TRUE30FPS)},  //14,
    {LOG_NAME_INT(VideoFrameRate::RATE_TRUE48FPS)},  //15,
    {LOG_NAME_INT(VideoFrameRate::RATE_TRUE60FPS)},  //16,
    {LOG_NAME_INT(VideoFrameRate::RATE_90FPS)},  //17,
    {LOG_NAME_INT(VideoFrameRate::RATE_192FPS)},  //18,
    {LOG_NAME_INT(VideoFrameRate::RATE_200FPS)},  //19,
    {LOG_NAME_INT(VideoFrameRate::RATE_400FPS)},  //20,
    {LOG_NAME_INT(VideoFrameRate::RATE_8FPS)},  //21,
    {LOG_NAME_INT(VideoFrameRate::RATE_20FPS)},  //22,
    {LOG_NAME_INT(VideoFrameRate::RATE_8_DOT_8FPS)},  //23,
};

static const std::pair<const int, const char *> photoRatioPairs[] = {
    {LOG_NAME_INT(PhotoRatio::RATIO_4COLON3)}, // 0, // 4:3
    {LOG_NAME_INT(PhotoRatio::RATIO_16COLON9)}, // 1, // 16:9
    {LOG_NAME_INT(PhotoRatio::RATIO_3COLON2)}, // 2, // 3:2
    {LOG_NAME_INT(PhotoRatio::RATIO_SQUARE)}, // 3, // 1:1
    {LOG_NAME_INT(PhotoRatio::RATIO_18COLON9)}, // 4, // 18:9
    {LOG_NAME_INT(PhotoRatio::UNKNOWN)}, // 0xFFFF, //Unknown
};

static const std::pair<const int, const char *> fileTypePairs[] = {
    {(int)MediaFileType::JPEG, "jpg"},// 0,
    {(int)MediaFileType::DNG, "dng"},// 1,
    {(int)MediaFileType::MOV, "mov"},// 2,
    {(int)MediaFileType::MP4, "mp4"},// 3,
    {(int)MediaFileType::TIFF, "tiff"},// 5,
};

static void printMediaFileMsg(MediaFile file) {
  static const std::map<const int, const char*> orientationMsgMap(orientationPairs, orientationPairs + sizeof orientationPairs / sizeof orientationPairs[0]);
  static const std::map<const int, const char*> videoResolutionMap(orientationPairs, videoResolutionPairs + sizeof videoResolutionPairs / sizeof videoResolutionPairs[0]);
  static const std::map<const int, const char*> videoFrameRateMap(videoFrameRatePairs, videoFrameRatePairs + sizeof videoFrameRatePairs / sizeof videoFrameRatePairs[0]);
  static const std::map<const int, const char*> photoRatioMap(photoRatioPairs, photoRatioPairs + sizeof photoRatioPairs / sizeof photoRatioPairs[0]);
  static const std::map<const int, const char*> fileTypeMap(fileTypePairs, fileTypePairs + sizeof fileTypePairs / sizeof fileTypePairs[0]);
  if (file.valid) {
    char logBuf[2048] = {0};
    /*! file name */
    sprintf(logBuf + strlen(logBuf), "##File [DJI_00%02d%s%s]", file.fileIndex % 100, ".",
            (fileTypeMap.find((int) file.fileType) == fileTypeMap.end() ? "???" : (fileTypeMap.find((int) file.fileType)->second)));
    /*! file index and size */
    if (file.fileSize > 1024 * 1024 * 1024)
      sprintf(logBuf + strlen(logBuf),"Index-%d; %0.2fGB; ", file.fileIndex, file.fileSize *1.0f / 1024 / 1024 / 1024);
    else if (file.fileSize > 1024 * 1024)
      sprintf(logBuf + strlen(logBuf),"Index-%d; %0.1fMB; ", file.fileIndex, file.fileSize *1.0f / 1024 / 1024);
    else if (file.fileSize > 1024)
      sprintf(logBuf + strlen(logBuf),"Index-%d; %0.1fKB; ", file.fileIndex, file.fileSize *1.0f / 1024);
    else
      sprintf(logBuf + strlen(logBuf),"Index-%d; %ldB; ", file.fileIndex, file.fileSize);
    /*! file time */
    sprintf(logBuf + strlen(logBuf), "%d-%d-%d_%d-%d-%d; ", file.date.year + 1980,
            file.date.month, file.date.day, file.date.hour, file.date.minute,
            file.date.second);
    if ((file.fileType == MediaFileType::MOV)
        || (file.fileType == MediaFileType::MP4)) {
      sprintf(logBuf + strlen(logBuf), "Video details : %lds; %s; %s; %s;", file.duration,
          (orientationMsgMap.find((int) file.orientation) == orientationMsgMap.end()) ? "ORIENTATION_UNKOWN" : orientationMsgMap.find((int) file.orientation)->second,
          (videoResolutionMap.find((int) file.resolution) == videoResolutionMap.end()) ? "RESOLUTION_UNKOWN" : videoResolutionMap.find((int) file.resolution)->second,
          (videoFrameRateMap.find((int) file.frameRate) == videoFrameRateMap.end()) ? "FRAME_RATE_UNKOWN" : videoFrameRateMap.find((int) file.frameRate)->second);
    } else if ((file.fileType == MediaFileType::JPEG)
        || (file.fileType == MediaFileType::DNG)
        || (file.fileType == MediaFileType::TIFF)) {
      sprintf(logBuf + strlen(logBuf), "Photo details : %s; %s;",
          (orientationMsgMap.find((int) file.orientation) == orientationMsgMap.end()) ? "ORIENTATION_UNKOWN" : orientationMsgMap.find((int) file.orientation)->second,
          (photoRatioMap.find((int) file.photoRatio) == photoRatioMap.end()) ? "RATIO_UNKOWN" : photoRatioMap.find((int) file.photoRatio)->second);
    }
    DSTATUS("%s", logBuf);
  } else {
    DERROR("This file is a valid file.");
  }
}

FilePackage cur_file_list;
void fileListReqCB(E_OsdkStat ret_code, const FilePackage file_list, void* udata) {
  DSTATUS("\033[1;32;40m##[%s] : ret = %d \033[0m", udata, ret_code);
  if (ret_code == OSDK_STAT_OK) {
    cur_file_list = file_list;
    DSTATUS("file_list.type = %d", file_list.type);
    DSTATUS("file_list.media.size() = %d", file_list.media.size());
    for (auto &file : file_list.media) {
      printMediaFileMsg(file);
    }
  }
}

bool fileDataDownloadFinished = false;
void fileDataReqCB(E_OsdkStat ret_code, void *udata) {
  if (ret_code == OSDK_STAT_OK) {
    DSTATUS("\033[1;32;40m##Download file [%s] successfully. \033[0m", udata);
  } else {
    DERROR("\033[1;31;40m##Download file data failed. \033[0m");
  }
  fileDataDownloadFinished = true;
}

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char **argv) {
  /*! Setup the OSDK: Read config file, create vehicle, activate. */
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }

  /*! Create an example object, which users can modify according to their own needs */
  CameraManagerSyncSample *p = new CameraManagerSyncSample(vehicle);
  GimbalManagerSyncSample *g = new GimbalManagerSyncSample(vehicle);

  /*! check whether enviroment passing valid running parameter or not */
  bool sampleCaseValidFlag = false;
  char inputChar = 0;
  std::string sampleCase = linuxEnvironment.getEnvironment()->getSampleCase();
  if (sampleCase.size() == 1) {
    if ((sampleCase <= "n") && (sampleCase >= "a")) {
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
        << "| [m] Rotate gimbal sample                                       |"
        << std::endl
        << "| [n] Reset gimbal sample                                        |"
        << std::endl
        << "| [o] Download camera filelist                                   |"
        << std::endl
        << "| [p] Download camera filedata from case o                       |"
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
        p->setExposureModeSyncSample(
            PAYLOAD_INDEX_0, CameraModule::ExposureMode::SHUTTER_PRIORITY);
        p->setShutterSpeedSyncSample(
            PAYLOAD_INDEX_0, CameraModule::ShutterSpeed::SHUTTER_SPEED_1_50);
        sleep(2);
        break;
      case 'b':
        p->setExposureModeSyncSample(
            PAYLOAD_INDEX_0, CameraModule::ExposureMode::APERTURE_PRIORITY);
        p->setApertureSyncSample(PAYLOAD_INDEX_0,
                                 CameraModule::Aperture::F_3_DOT_5);
        sleep(2);
        break;
      case 'c':
        p->setExposureModeSyncSample(PAYLOAD_INDEX_0,
                                     CameraModule::ExposureMode::PROGRAM_AUTO);
        p->setEVSyncSample(PAYLOAD_INDEX_0,
                           CameraModule::ExposureCompensation::P_0_3);
        sleep(2);
        break;
      case 'd':
        p->setExposureModeSyncSample(
            PAYLOAD_INDEX_0, CameraModule::ExposureMode::EXPOSURE_MANUAL);
        p->setISOSyncSample(PAYLOAD_INDEX_0, CameraModule::ISO::ISO_200);
        sleep(2);
        break;
      case 'e':
        p->setFocusPointSyncSample(PAYLOAD_INDEX_0, 0.8, 0.8);
        break;
      case 'f':
        p->setTapZoomPointSyncSample(PAYLOAD_INDEX_1, 5, 0.3, 0.3);
        sleep(5);
        p->setTapZoomPointSyncSample(PAYLOAD_INDEX_1, 5, 0.8, 0.7);
        sleep(5);
        break;
      case 'g':
        p->setZoomSyncSample(PAYLOAD_INDEX_0, 5);
        sleep(4);
        p->setZoomSyncSample(PAYLOAD_INDEX_0, 10);
        sleep(4);
        p->startZoomSyncSample(PAYLOAD_INDEX_0,
                               CameraModule::ZoomDirection::ZOOM_OUT,
                               CameraModule::ZoomSpeed::FASTEST);
        sleep(8);
        p->stopZoomSyncSample(PAYLOAD_INDEX_0);
        break;
      case 'h':
        p->startShootSinglePhotoSyncSample(PAYLOAD_INDEX_0);
        break;
      case 'i':
        p->startShootAEBPhotoSyncSample(
            PAYLOAD_INDEX_0, CameraModule::PhotoAEBCount::AEB_COUNT_5);
        break;
      case 'j':
        p->startShootBurstPhotoSyncSample(
            PAYLOAD_INDEX_0, CameraModule::PhotoBurstCount::BURST_COUNT_7);
        break;
      case 'k':
        CameraModule::PhotoIntervalData intervalData;
        intervalData.photoNumConticap = 255;
        intervalData.timeInterval = 3;
        p->startShootIntervalPhotoSyncSample(PAYLOAD_INDEX_0, intervalData);
        DSTATUS("Sleep 15 seconds");
        sleep(15);
        p->shootPhotoStopSyncSample(PAYLOAD_INDEX_0);
        break;
      case 'l':
        p->startRecordVideoSyncSample(PAYLOAD_INDEX_0);
        sleep(10);
        p->stopRecordVideoSyncSample(PAYLOAD_INDEX_0);
        break;
      case 'm':
        DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
                g->getGimbalData(PAYLOAD_INDEX_0).pitch,
                g->getGimbalData(PAYLOAD_INDEX_0).roll,
                g->getGimbalData(PAYLOAD_INDEX_0).yaw);
        GimbalModule::Rotation rotation;
        rotation.roll = 0.0f;
        rotation.pitch = 25.0f;
        rotation.yaw = 90.0f;
        rotation.rotationMode = 0;
        rotation.time = 0.5;
        g->rotateSyncSample(PAYLOAD_INDEX_0, rotation);
        sleep(2);
        DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
                g->getGimbalData(PAYLOAD_INDEX_0).pitch,
                g->getGimbalData(PAYLOAD_INDEX_0).roll,
                g->getGimbalData(PAYLOAD_INDEX_0).yaw);
        break;
      case 'n':
        DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
                g->getGimbalData(PAYLOAD_INDEX_0).pitch,
                g->getGimbalData(PAYLOAD_INDEX_0).roll,
                g->getGimbalData(PAYLOAD_INDEX_0).yaw);
        g->resetSyncSample(PAYLOAD_INDEX_0);
        sleep(2);
        DSTATUS("Current gimbal %d angle (p,r,y) = (%0.2f°, %0.2f°, %0.2f°)", PAYLOAD_INDEX_0,
                g->getGimbalData(PAYLOAD_INDEX_0).pitch,
                g->getGimbalData(PAYLOAD_INDEX_0).roll,
                g->getGimbalData(PAYLOAD_INDEX_0).yaw);
        break;
      case 'o': {
        DSTATUS("Play back mode setting......");
        vehicle->cameraManager->setModeSync(PAYLOAD_INDEX_0,
                                            CameraModule::WorkMode::PLAYBACK,
                                            2);
        DSTATUS("Get liveview right......");
        ErrorCode::ErrorCodeType ret =
            vehicle->cameraManager->obtainDownloadRightSync(PAYLOAD_INDEX_0,
                                                            true, 2);
        ErrorCode::printErrorCodeMsg(ret);
        DSTATUS("Try to download file list  .......");
        ret = vehicle->cameraManager->startReqFileList(fileListReqCB, (void *)("Download main camera file list"));
        ErrorCode::printErrorCodeMsg(ret);
        break;
      }
      case 'p': {

        DSTATUS("Download file number : %d", cur_file_list.media.size());
        for (uint32_t i = 0; i < cur_file_list.media.size(); i++) {
          fileDataDownloadFinished = false;
          DSTATUS("回放模式......");
          vehicle->cameraManager->setModeSync(PAYLOAD_INDEX_0,
                                              CameraModule::WorkMode::PLAYBACK,
                                              2);
          DSTATUS("Get liveview right......");
          ErrorCode::ErrorCodeType ret =
              vehicle->cameraManager->obtainDownloadRightSync(PAYLOAD_INDEX_0,
                                                              true, 2);
          ErrorCode::printErrorCodeMsg(ret);

          DSTATUS("Try to download file list  .......");
          char pathBuffer[100] = {0};
          sprintf(pathBuffer, "./DJI0%03d", cur_file_list.media[i].fileIndex - 99900);
          std::string localPath(pathBuffer);

          DSTATUS("cur_file_list.media[i].fileIndex = %d, localPath = %s", cur_file_list.media[i].fileIndex, localPath.c_str());
          ret = vehicle->cameraManager->startReqFileData(cur_file_list.media[i].fileIndex, localPath, fileDataReqCB, (void *) (localPath.c_str()));
          ErrorCode::printErrorCodeMsg(ret);
          while (fileDataDownloadFinished == false) {
            OsdkOsal_TaskSleepMs(1000);
          }
          DSTATUS("Prepare to do next downloading ...");
          OsdkOsal_TaskSleepMs(1000);
        }
        break;
      }
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
  delete g;
  return 0;
}
