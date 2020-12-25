/** @file dji_file_mgr_define.hpp
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

#ifndef DJI_FILE_MGR_DEFINE_HPP
#define DJI_FILE_MGR_DEFINE_HPP

#include <vector>
#include <string>
#include <map>

namespace DJI {
namespace OSDK {

typedef enum {
  DJI_CAMERA_TYPE_UNK = 255,
} DJI_CAMERA_TYPE;

enum class FileType {
  MEDIA = 0, //Media files
  COMMON = 1, //Common file, logs
  SPEAKER_AUDIO = 2, // Audio files
  UNKNOWN = 0xFFFF, //Unknown
};

enum class FileLocation {
  SD_CARD = 0,
  INTERNAL_STORAGE = 1,
  EXTENDED_SD_CARD = 2,
  UNKNOWN = 0xFFFF, //Unknown
};

enum class MediaFileType {
  JPEG = 0,
  DNG = 1,
  MOV = 2,
  MP4 = 3,
  PANORAMA = 4,
  TIFF = 5,
  UL_CTRL_INFO = 6,
  UL_CTRL_INFO_LZ4 = 7,
  AUDIO = 10,
  UNKNOWN = 0xFFFF,
};

struct DateTime {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
};

struct CommonFile {
  int fileIndex;
  MediaFileType fileType;
  std::string fileName;
  int64_t fileSize;
  DateTime date;
};

struct FileExifInfo {
  bool focalLength35mmFormatEnabled;
  bool lightSourceEnabled;
  bool meteringModeEnabled;
  bool exposureCompensationEnabled;
  bool isoEnabled;
  bool exposureProgramEnabled;
  bool fNumberEnabled;
  bool exposureTimeEnabled;
  int exposureProgram;
  int iso;
  int meteringMode;
  int lightSource;
  int focalLength35mmFormat;
  std::string shutterSpeedText;
  std::string apertureText;
  std::string exposureCompensationText;
};

enum class MediaFileStarTag {
  NONE = 0,
  TAGGED = 1,
  UNKNOWN = 0xFF,
};

enum class CameraOrientation {
  DEFAULT = 0,
  CW90 = 1,
  CW180 = 2,
  CW270 = 3,
  UNKNOWN = 0xFFFF, //Unknown
};

enum class VideoFrameRate {
  RATE_15FPS = 0,
  RATE_24FPS = 1,
  RATE_25FPS = 2,
  RATE_30FPS = 3,
  RATE_48FPS = 4,
  RATE_50FPS = 5,
  RATE_60FPS = 6,
  RATE_120FPS = 7,
  RATE_240FPS = 8,
  RATE_480FPS = 9,
  RATE_100PS = 10,
  RATE_96FPS = 11,
  RATE_180FPS = 12,
  RATE_TRUE24FPS = 13,
  RATE_TRUE30FPS = 14,
  RATE_TRUE48FPS = 15,
  RATE_TRUE60FPS = 16,
  RATE_90FPS = 17,
  RATE_192FPS = 18,
  RATE_200FPS = 19,
  RATE_400FPS = 20,
  RATE_8FPS = 21,
  RATE_20FPS = 22,
  RATE_8_DOT_8FPS = 23,
};

enum class VideoResolution {
  RESOLUTION_640X480P = 0, // 640X480P
  RESOLUTION_640X480I = 1, // 640X480I
  RESOLUTION_1280X640P = 2, // 1280X640P
  RESOLUTION_1280X640I = 3, // 1280X640I
  RESOLUTION_1280X720P = 4, // 1280X720P
  RESOLUTION_1280X720I = 5, // 1280X720I
  RESOLUTION_1280X960P = 6, // 1280X960P
  RESOLUTION_1280X960I = 7, // 1280X960I
  RESOLUTION_1920X960P = 8, // 1920X960P
  RESOLUTION_1920X960I = 9, // 1920X960I
  RESOLUTION_1920X1080P = 10, // 1920X1080P
  RESOLUTION_1920X1080I = 11, // 1920X1080I
  RESOLUTION_1920X1440P = 12, // 1920X1440P
  RESOLUTION_1920X1440I = 13, // 1920X1440I
  RESOLUTION_3840X1920P = 14, // 3840X1920P
  RESOLUTION_3840X1920I = 15, // 3840X1920I
  RESOLUTION_3840X2160P = 16, // 3840X2160P
  RESOLUTION_3840X2160I = 17, // 3840X2160I
  RESOLUTION_3840X2880P = 18, // 3840X2880P
  RESOLUTION_3840X2280I = 19, // 3840X2280I
  RESOLUTION_4096X2048P = 20, // 4096X2048P
  RESOLUTION_4960X2048I = 21, // 4960X2048I
  RESOLUTION_4096X2160P = 22, // 4096X2160P
  RESOLUTION_4096X2160I = 23, // 4096X2160I
  RESOLUTION_2704X1520P_16COLON9 = 24, // 2704X1520P,16:9
  RESOLUTION_640X512P_FLIR = 26, // 640X512P_FLIR
  RESOLUTION_4608X2160 = 27, // 4608X2160
  RESOLUTION_4608X2592 = 28, // 4608X2592
  RESOLUTION_2720X1530P = 31, // 2720X1530P
  RESOLUTION_5280X2160P = 32, // 5280X2160P
  RESOLUTION_5280X2970P = 33, // 5280X2970P
  RESOLUTION_3840X1572P = 34, // 3840X1572P
  RESOLUTION_5760X3240P = 35, // 5760X3240P
  RESOLUTION_6016X3200P = 36, // 6016X3200P
  RESOLUTION_2048X1080P = 37, // 2048X1080P
  RESOLUTION_336X256P_FLIR = 38, // 336X256P_FLIR
  RESOLUTION_5120X2880P = 39, // 5120x2880P
  RESOLUTION_4096X2160P_RAW14 = 40, // 4096X2160P_RAW14
  RESOLUTION_3840X2160P_RAW14 = 41, // 3840X2160P_RAW14
  RESOLUTION_2720X1530P_RAW14 = 42, // 2720X1530P_RAW14
  RESOLUTION_1920X1080P_RAW14 = 43, // 1920X1080P_RAW14
  RESOLUTION_5440X2880P = 44, // 5440X2880P
  RESOLUTION_2688X1512P = 45, // 2688X1512P
  RESOLUTION_640X360P = 46, // 640X360P
  RESOLUTION_4000X3000P = 48, // 4000X3000P
  RESOLUTION_4000X3000I = 49, // 4000X3000I
  RESOLUTION_2880X1620P = 50, // 2880X1620P
  RESOLUTION_2880X1620I = 51, // 2880X1620I
  RESOLUTION_2720X2040P = 52, // 2720X2040P
  RESOLUTION_2720X2040I = 53, // 2720X2040I
  RESOLUTION_720X576 = 54, // 720X576
  RESOLUTION_MAX = 253, // MAX
  RESOLUTION_UNSET = 254, // UNSET
  RESOLUTION_KNOWN = 255, // KNOWN
};

enum class MediaVideoType {
  NORMAL = 0, // 普通录像
  SLOW_MOTION = 1, // 慢动作录像
  HYPER_LAPSE = 2, // Hyperlapse
  TIME_LAPSE = 3, // 延时视频（拍照合成方式）
  HDR = 4, // HDR录像
  LOOP = 5, // 循环录像
  UNKNOWN = 0xFFFF, //Unknown
};

enum class MediaPhotoType {
  NORMAL = 0, //普通
  HDR = 1, //HDR单拍
  AEB = 2, //AEB连拍
  INTERVAL = 3, //定时拍
  BURST = 4, //定时拍
  UNKNOWN = 0xFFFF, //Unknown
};

enum class CameraPanoType {
  TYPE_360AUTO = 1, // 360 pano
  TYPE_BALL = 2, // ball pano
  TYPE_SELF = 3, // self pano
  TYPE_MANUAL = 4, // manul pano
  TYPE_CYLINDRICAL = 5, // Cylindrical_180
  TYPE_VERTICAL = 6, // 竖直拼接
  TYPE_SECTORIAL = 7, // 3x3广角
  UNKNOWN = 0xFFFF,
};

enum class PhotoRatio {
  RATIO_4COLON3 = 0, // 4:3
  RATIO_16COLON9 = 1, // 16:9
  RATIO_3COLON2 = 2, // 3:2
  RATIO_SQUARE = 3, // 1:1
  RATIO_18COLON9 = 4, // 18:9
  UNKNOWN = 0xFFFF, //Unknown
};

struct MediaFile {
  bool valid; //File valid or not
  //bool isManualGroupFile; // reserve
  int fileIndex; //Using for download
  MediaFileType fileType;
  std::string fileName;
  int64_t fileSize; //Bytes
  DateTime date; //Create date
  //MediaFileStarTag starTag;
  int64_t duration;
  CameraOrientation orientation;
  VideoFrameRate frameRate;
  VideoResolution resolution;
  MediaVideoType videoType;
  MediaPhotoType photoType;
  CameraPanoType panoType;
  //int videoSpeedRatio;
  //int panoCount;
  //int guid;
  //int fileGroupIndex;
  //int subIndex;
  //int segSubIndex;
  //int timeLapseInterval;
  //FileExifInfo EXIFInfo;
  PhotoRatio photoRatio;
  //std::vector<MediaFile> subMediaFile;
};

struct FilePackage {
  FileType type;
  std::vector<MediaFile> media;
  //std::vector<CommonFile> common;
};

extern const std::map<const int, const char*> orientationMsgMap;
extern const std::map<const int, const char*> videoResolutionMap;
extern const std::map<const int, const char*> videoFrameRateMap;
extern const std::map<const int, const char*> photoRatioMap;
extern const std::map<const int, const char*> fileTypeMap;

void printMediaFileMsg(DJI::OSDK::MediaFile file);

}
}

#endif //DJI_FILE_MGR_DEFINE_HPP
