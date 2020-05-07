//
// Created by dji on 4/8/20.
//

#ifndef DJI_FILE_MGR_DEFINE_HPP
#define DJI_FILE_MGR_DEFINE_HPP

#include <vector>

namespace DJI {
namespace OSDK {

typedef enum {
  DJI_CAMERA_TYPE_FC350 = 0, // Inpire 1
  DJI_CAMERA_TYPE_FC550 = 1, // Inpire 1 pro
  DJI_CAMERA_TYPE_FC260 = 2, // phantom3C
  DJI_CAMERA_TYPE_FC300S = 3, // phantom3S
  DJI_CAMERA_TYPE_FC300X = 4, // phantom3X
  DJI_CAMERA_TYPE_FC550RAW = 5, // Inpire 1 pro Raw
  DJI_CAMERA_TYPE_FC330X = 6, // Phantom 4
  DJI_CAMERA_TYPE_TAU640 = 7, // Flir 640
  DJI_CAMERA_TYPE_TAU336 = 8, // Flir 336
  DJI_CAMERA_TYPE_FC220 = 9, // wm220 camera
  DJI_CAMERA_TYPE_FC300XW = 10, // phantom3 4k
  DJI_CAMERA_TYPE_CV600 = 11, // cv600 3.5x
  DJI_CAMERA_TYPE_FC65XXUNKNOWN = 12, // IN2的 SENSOR未检测到
  DJI_CAMERA_TYPE_FC6310 = 13, // P4PRO IMX183 相机
  DJI_CAMERA_TYPE_FC6510 = 14, // IN2的 H1   （IMX183, X4
  DJI_CAMERA_TYPE_FC6520 = 15, // IN2的 H1（IMX269） 目前确定在用, X5S
  DJI_CAMERA_TYPE_FC6532 = 16, // IN2的全画幅(MN34401)
  DJI_CAMERA_TYPE_FC6540 = 17, // IN2的S35
  DJI_CAMERA_TYPE_FC220LOW = 18, // 小飞机低端版本
  DJI_CAMERA_TYPE_FC1102 = 19, // 小小飞机
  DJI_CAMERA_TYPE_GD600 = 20, // 30X变焦
  DJI_CAMERA_TYPE_FC6310A = 21, // P4A IMX183 相机
  DJI_CAMERA_TYPE_P3SE = 22, // P3C魔改版，P3SE
  DJI_CAMERA_TYPE_WM230 = 23, // WM230 Mavic Air
  DJI_CAMERA_TYPE_HG200 = 24, // OSMO MINI 小相机
  DJI_CAMERA_TYPE_FC2204 = 25, // WM240 IMX477 相机
  DJI_CAMERA_TYPE_FC1705 = 26, // XT2
  DJI_CAMERA_TYPE_HG330 = 27, // HG330 相机
  DJI_CAMERA_TYPE_FC6310S = 28, // P4PRO V2.0（P4P SDR版本）IMX183 相机
  DJI_CAMERA_TYPE_FC2211 = 29, // WM240 IMX283 相机
  DJI_CAMERA_TYPE_OCUSYNC = 30, // ZV811
  DJI_CAMERA_TYPE_P_SDK = 31, // 第三方相机通用相机类型
  DJI_CAMERA_TYPE_OCUSYNC2 = 32, // OcuSync2型
  DJI_CAMERA_TYPE_AC101 = 33, // AC101运动相机
  DJI_CAMERA_TYPE_EC1704 = 34, // EC1704
  DJI_CAMERA_TYPE_EC1709 = 35, // EC1709
  DJI_CAMERA_TYPE_BR1609 = 36, // BR1609
  DJI_CAMERA_TYPE_XW0607_OV5695 = 37, // XW0607（教育机器人） OV5695 相机
  DJI_CAMERA_TYPE_FC2220 = 38, // WM245 IMX477 相机
  DJI_CAMERA_TYPE_MATRICE_FPV = 39, // M200系列、VITOL系列FPV相机
  DJI_CAMERA_TYPE_FC2403 = 40, // WM245 IMX378 相机
  DJI_CAMERA_TYPE_TP1810 = 41, // 国产红外相机，基于Iray LT模组
  DJI_CAMERA_TYPE_GD610_DOUBLE_CAM =
  42, // GD610 双光相机（一路imx378 Wide，一路imx204 Zoom)
  DJI_CAMERA_TYPE_GD610_TRIPLE_CAM =
  43, // GD610 三光相机（一路imx378 Wide，一路imx204 Zoom，一路自研红外）
  DJI_CAMERA_TYPE_FC7103 = 44, // WM160 IMX378 相机
  DJI_CAMERA_TYPE_WM231 = 45, // WM231 IMX586 相机
  DJI_CAMERA_TYPE_WM170 = 46, // WM170 IMX577 相机
  DJI_CAMERA_TYPE_THIRDPARTYSTART = 160, // 比这个大的都认为是第三方相机
  DJI_CAMERA_TYPE_HASSELH6D_50C = 166, // 哈苏
  DJI_CAMERA_TYPE_HASSELH6D_100C = 167,
  DJI_CAMERA_TYPE_UNK = 255, // Inpire 1
} DJI_CAMERA_TYPE;

enum class FileType {
  MEDIA = 0, //媒体文件
  COMMON = 1, //普通文件例如日志
  SPEAKER_AUDIO = 2, //扬声器音频文件
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
  UNKNOWN = 0xFFFF, //Unknown
};

struct DateTime {
  int year; // 年
  int month; // 月
  int day; // 日
  int hour; // 小时
  int minute; // 分钟
  int second; // 秒钟
};

struct CommonFile {
  int fileIndex; //文件编号
  MediaFileType fileType; //文件类型
  std::string fileName; //文件名
  int64_t fileSize; //文件大小
  DateTime date; //创建日期
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
  std::string shutterSpeedText; //exposureTime格式化后的Shutter 1/12.5
  std::string apertureText; //fnumber格式化后的光圈 1.3
  std::string exposureCompensationText; //exposureCompensation格式化后的EV +3.5
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
  bool valid; //文件是否有效
  //bool isManualGroupFile; // 是否上层(SDK)手动成组文件, 即拉列表拉到几个index不一样但fileGroupIndex一致文件时，上层手动成组最终返回一个isManualGroupFile为true对象
  int fileIndex; //文件编号
  MediaFileType fileType; //文件类型
  std::string fileName; //文件名
  int64_t fileSize; //文件大小
  DateTime date; //创建日期
  //MediaFileStarTag starTag; //星标
  int64_t duration; //时长
  CameraOrientation orientation; //朝向
  VideoFrameRate frameRate; //帧率
  VideoResolution resolution; //分辨率信息
  MediaVideoType videoType; //视频类型
  MediaPhotoType photoType; //拍照类型
  CameraPanoType panoType; // 全景拍照类型
  //int videoSpeedRatio; // SlowMotion 及 FastMotion 倍速
  //int panoCount; // 全景照片张数
  //int guid; //视频guid
  //int fileGroupIndex; //文件组
  //int subIndex; // html占位组文件子index，正常文件默认为0
  //int segSubIndex; // fat32文件系统中 4g以上文件会做切分
  //int timeLapseInterval; //TimeLapse视频帧间隔，存储值乘以100ms
  //FileExifInfo EXIFInfo;
  PhotoRatio photoRatio;
  //std::vector<MediaFile> subMediaFile;
};

struct FilePackage {
  FileType type;
  std::vector<MediaFile> media; //媒体文件
  //std::vector<CommonFile> common; //普通文件
};


}
}
#endif //DJI_FILE_MGR_DEFINE_HPP
