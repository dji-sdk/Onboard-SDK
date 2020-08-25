/** @file dji_file_mgr_define.cpp
 *  @version 4.0.0
 *  @date July 2020
 *
 *  @brief Definitions and enums for file manager
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

#include "dji_file_mgr_define.hpp"
#include <cstring>
#include "dji_log.hpp"

using namespace DJI;
using namespace DJI::OSDK;

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

const std::map<const int, const char*> DJI::OSDK::orientationMsgMap(orientationPairs, orientationPairs + sizeof orientationPairs / sizeof orientationPairs[0]);
const std::map<const int, const char*> DJI::OSDK::videoResolutionMap(orientationPairs, videoResolutionPairs + sizeof videoResolutionPairs / sizeof videoResolutionPairs[0]);
const std::map<const int, const char*> DJI::OSDK::videoFrameRateMap(videoFrameRatePairs, videoFrameRatePairs + sizeof videoFrameRatePairs / sizeof videoFrameRatePairs[0]);
const std::map<const int, const char*> DJI::OSDK::photoRatioMap(photoRatioPairs, photoRatioPairs + sizeof photoRatioPairs / sizeof photoRatioPairs[0]);
const std::map<const int, const char*> DJI::OSDK::fileTypeMap(fileTypePairs, fileTypePairs + sizeof fileTypePairs / sizeof fileTypePairs[0]);

void DJI::OSDK::printMediaFileMsg(DJI::OSDK::MediaFile file) {

  if (file.valid) {
    char logBuf[2048] = {0};
    /*! file name */
    sprintf(logBuf + strlen(logBuf), "##File [%s]", file.fileName.c_str());
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
    sprintf(logBuf + strlen(logBuf), "%d-%d-%d_%d-%d-%d; ", file.date.year,
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