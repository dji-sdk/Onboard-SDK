//
//  newfilelistrequest.h
//  djisdk
//
//  Created by Xietong LU on 2018/5/27.
//

#ifndef newfilelistrequest_h
#define newfilelistrequest_h

#include <stdio.h>
#include <list>
#include <map>
#include "transferrequest.h"

namespace DJI {
namespace OSDK {


typedef enum
{
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
  DJI_CAMERA_TYPE_GD610_DOUBLE_CAM = 42, // GD610 双光相机（一路imx378 Wide，一路imx204 Zoom)
  DJI_CAMERA_TYPE_GD610_TRIPLE_CAM = 43, // GD610 三光相机（一路imx378 Wide，一路imx204 Zoom，一路自研红外）
  DJI_CAMERA_TYPE_FC7103 = 44, // WM160 IMX378 相机
  DJI_CAMERA_TYPE_WM231 = 45, // WM231 IMX586 相机
  DJI_CAMERA_TYPE_WM170 = 46, // WM170 IMX577 相机
  DJI_CAMERA_TYPE_THIRDPARTYSTART = 160, // 比这个大的都认为是第三方相机
  DJI_CAMERA_TYPE_HASSELH6D_50C = 166, // 哈苏
  DJI_CAMERA_TYPE_HASSELH6D_100C = 167,
  DJI_CAMERA_TYPE_UNK = 255, // Inpire 1
} DJI_CAMERA_TYPE;


enum class FileType{
  MEDIA = 0, //媒体文件
  COMMON = 1, //普通文件例如日志
  SPEAKER_AUDIO = 2, //扬声器音频文件
  UNKNOWN = 0xFFFF, //Unknown
};

enum class FileLocation{
  SD_CARD = 0,
  INTERNAL_STORAGE = 1,
  EXTENDED_SD_CARD = 2,
  UNKNOWN = 0xFFFF, //Unknown
};

enum class MediaFileType{
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

struct DateTime
{
  int year; // 年
  int month; // 月
  int day; // 日
  int hour; // 小时
  int minute; // 分钟
  int second; // 秒钟
};

struct CommonFile
{
  int fileIndex; //文件编号
  MediaFileType fileType; //文件类型
  std::string fileName; //文件名
  int64_t fileSize; //文件大小
  DateTime date; //创建日期
};

struct FileExifInfo
{
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

enum class MediaFileStarTag{
  NONE = 0,
  TAGGED = 1,
  UNKNOWN = 0xFF,
};

enum class CameraOrientation{
  DEFAULT = 0,
  CW90 = 1,
  CW180 = 2,
  CW270 = 3,
  UNKNOWN = 0xFFFF, //Unknown
};

enum class VideoFrameRate{
  RATE_24FPS = 0,
  RATE_25FPS = 1,
  RATE_30FPS = 2,
  RATE_48FPS = 3,
  RATE_50FPS = 4,
  RATE_60FPS = 5,
  RATE_90FPS = 6,
  RATE_96FPS = 7,
  RATE_100FPS = 8,
  RATE_120FPS = 9,
  RATE_180FPS = 10,
  RATE_192FPS = 11,
  RATE_200FPS = 12,
  RATE_240FPS = 13,
  RATE_400FPS = 14,
  RATE_480FPS = 15,
  RATE_PRECISE_24FPS = 16,
  RATE_PRECISE_30FPS = 17,
  RATE_PRECISE_48FPS = 18,
  RATE_PRECISE_60FPS = 19,
  RATE_8DOT7_FPS = 20,
  UNKNOWN = 0xFFFF,
};

enum class VideoResolution{
  RESOLUTION_640x480 = 0, // The camera's video resolution is 640x480.
  RESOLUTION_640x512 = 1, // The camera's video resolution is 640x512.
  RESOLUTION_1280x720 = 2, // The camera's video resolution is 1280x720.
  RESOLUTION_1920x960 = 3, // The camera's video resolution is 1920x960.
  RESOLUTION_1920x1080 = 4, // The camera's video resolution is 1920x1080.
  RESOLUTION_2048x1080 = 5, // The camera's video resolution is 2048x1080.
  RESOLUTION_2704x1520 = 6, // The camera's video resolution is 2704x1520.
  RESOLUTION_2720x1530 = 7, // The camera's video resolution is 2720x1530.
  RESOLUTION_3840x1572 = 8, // The camera's video resolution is 3840x1572.
  RESOLUTION_3840x2160 = 9, // The camera's video resolution is 3840x2160.
  RESOLUTION_4096x2160 = 10, // The camera's video resolution is 4096x2160.
  RESOLUTION_4608x2160 = 11, // The camera's video resolution is 4608x2160.
  RESOLUTION_4608x2592 = 12, // The camera's video resolution is 4608x2592.
  RESOLUTION_5280x2160 = 13, // The camera's video resolution is 5280x2160.
  RESOLUTION_5280x2972 = 14, // The camera's video resolution is 5280x2972.
  RESOLUTION_5760x3240 = 15, // The camera's video resolution is 5760x3240.
  RESOLUTION_6016x3200 = 16, // The camera's video resolution is 6016x3200.
  RESOLUTION_MAX = 17, // The camera's video resolution will be maximum resolution supported by the camera sensor. For X5S and X4S,  the maximum resolution is 5280x2972.
  RESOLUTION_NO_SSD_VIDEO = 18, // The camera's SSD video resolution is unset. When the SSD resolution is unset, camera will not store video to SSD.   SSD's resolution is determined by both license key and SD card's video frame rate. If there is no supported  resolution with the current configuration, <code>DJIVideoResolutionUnset</code> will be selected and user  should adjust either the license or the frame rate.
  RESOLUTION_2688x1512 = 19,
  RESOLUTION_4000x3000 = 48, // The camera's video resolution is 4000x3000.
  RESOLUTION_2880x1620 = 50, // The camera's video resolution is 2880x1620
  RESOLUTION_2720x2040P = 52, // The camera's video resolution is 2720X2040P
  UNKNOWN = 0xFFFF, //Unknown
};

enum class MediaVideoType{
  NORMAL = 0, // 普通录像
  SLOW_MOTION = 1, // 慢动作录像
  HYPER_LAPSE = 2, // Hyperlapse
  TIME_LAPSE = 3, // 延时视频（拍照合成方式）
  HDR = 4, // HDR录像
  LOOP = 5, // 循环录像
  UNKNOWN = 0xFFFF, //Unknown
};

enum class MediaPhotoType{
  NORMAL = 0, //普通
  HDR = 1, //HDR单拍
  AEB = 2, //AEB连拍
  INTERVAL = 3, //定时拍
  BURST = 4, //定时拍
  UNKNOWN = 0xFFFF, //Unknown
};

enum class CameraPanoType{
  TYPE_360AUTO = 1, // 360 pano
  TYPE_BALL = 2, // ball pano
  TYPE_SELF = 3, // self pano
  TYPE_MANUAL = 4, // manul pano
  TYPE_CYLINDRICAL = 5, // Cylindrical_180
  TYPE_VERTICAL = 6, // 竖直拼接
  TYPE_SECTORIAL = 7, // 3x3广角
  UNKNOWN = 0xFFFF,
};

enum class PhotoRatio{
  RATIO_4COLON3 = 0, // 4:3
  RATIO_16COLON9 = 1, // 16:9
  RATIO_3COLON2 = 2, // 3:2
  RATIO_SQUARE = 3, // 1:1
  RATIO_18COLON9 = 4, // 18:9
  UNKNOWN = 0xFFFF, //Unknown
};

struct MediaFile
{
  bool valid; //文件是否有效
  bool isManualGroupFile; // 是否上层(SDK)手动成组文件, 即拉列表拉到几个index不一样但fileGroupIndex一致文件时，上层手动成组最终返回一个isManualGroupFile为true对象
  int fileIndex; //文件编号
  MediaFileType fileType; //文件类型
  std::string fileName; //文件名
  int64_t fileSize; //文件大小
  DateTime date; //创建日期
  MediaFileStarTag starTag; //星标
  int64_t duration; //时长
  CameraOrientation orientation; //朝向
  VideoFrameRate frameRate; //帧率
  VideoResolution resolution; //分辨率信息
  MediaVideoType videoType; //视频类型
  MediaPhotoType photoType; //拍照类型
  CameraPanoType panoType; // 全景拍照类型
  int videoSpeedRatio; // SlowMotion 及 FastMotion 倍速
  int panoCount; // 全景照片张数
  int guid; //视频guid
  int fileGroupIndex; //文件组
  int subIndex; // html占位组文件子index，正常文件默认为0
  int segSubIndex; // fat32文件系统中 4g以上文件会做切分
  int timeLapseInterval; //TimeLapse视频帧间隔，存储值乘以100ms
  FileExifInfo EXIFInfo;
  PhotoRatio photoRatio;
  std::vector<MediaFile> subMediaFile;
};

struct FilePackage
{
  FileType type;
  std::vector<MediaFile> media; //媒体文件
  std::vector<CommonFile> common; //普通文件
};

struct FileList
{
  FileLocation location;
  FilePackage files;
  bool hasInvalidFile; //是否读取到无效的文件，比如sdcard中有10个文件，有两个录制失败的视频，上传了8个有效文件的信息
};

using FileListCallback = std::function<void(int ret_code, std::shared_ptr<const FileList> file_list)>;

class IFileListOutputHandler {
public:
    virtual void TriggerCallback(int err_code) = 0;
    virtual void ResetOutput() = 0;
    virtual void OnTail(int file_count) = 0;
};

class FileListOutputHandler : public IFileListOutputHandler {
public:
    FileListCallback file_list_callback_;
    std::shared_ptr<FileList> output_;
    FileListOutputHandler(FileListCallback file_list_callback) : file_list_callback_(file_list_callback){};
    void TriggerCallback(int err_code) override {
#if 0
        LOGI << "[FileMgr] FileListOutputHandler::TriggerCallback err_code -> " << err_code << ", files_num -> "
             << (output_ ? std::to_string(output_->files.media.size()) : "output_ nullptr") << ", file_list_callback_ == nullptr -> "
             << (int)(file_list_callback_ == nullptr);
#endif
        if (file_list_callback_ != nullptr) {
            if (err_code != kNoError) {
                file_list_callback_(err_code, nullptr);
            } else {
                if (output_ == nullptr) {
                    output_ = std::make_shared<FileList>();
                    output_->hasInvalidFile = false;
                }
                file_list_callback_(err_code, output_);
            }
            file_list_callback_ = nullptr;
        }
    }
    void ResetOutput() override {
        output_ = nullptr;
    };

    void OnTail(int file_count) override{};
};


struct FileListRequest;
class ListTransferRequest : public TransferRequest {
public:
    ListTransferRequest(uint16_t session_id, std::shared_ptr<const FileListRequest> config, DJI_CAMERA_TYPE camera_type, FileListCallback callback);
    //ListTransferRequest(uint16_t session_id, std::shared_ptr<const FileListRequest> config, SpeakerAudioFileListCallback callback);

    ~ListTransferRequest();
    virtual dji::core::dji_cmd_req CreateStartRequestPack() override;
    virtual void OnHandlerFailure(int err_code) override;
    virtual void ResetInternalData() override;
    std::string GetDescription() override;
    virtual int GetAbortSessionTaskID() override;

    virtual int GetTimeoutDuration() override;

protected:
    int ValidateRequestConfig();
    virtual void ParseData(bool is_tail, ParsingResultHandler handler) override;
    size_t TotalBufferSize();

private:
    void StoreData(uint8_t *data, size_t data_size, bool front);
    void ConfigFilterData();
    int ConsumeMediaItem(ListDataItem *item, uint8_t *data);
    int ConsumeCommonItem(ListDataItem *item, uint8_t *data);
    int ConsumeSpeakerAudioItem(ListDataItem *item, uint8_t *data);

    enum class ParseState {
        INITIAL,
        PARSING_ITEM_HEADER,
        PARSING_ITEM_BODY,
        DONE,
        FAILED,
    };

private:
    DJI_DOWNLOAD_FILE_LIST_TYPE type_ = DJI_DOWNLOAD_FILE_LIST_TYPE::DJI_MEDIA;
    DJI_FILE_LIST_FILTER_LIKE likeFilter_ = DJI_FILE_LIST_FILTER_LIKE::DJI_all;
    uint32_t photoFilter_ = 0;
    uint32_t videoFilter_ = 0;
    std::shared_ptr<const FileListRequest> config_;
    std::list<std::pair<uint8_t *, size_t>> data_buffers_;
    ParseState parse_state_ = ParseState::INITIAL;
    ListDataPack *header_ = nullptr;
    ListDataItem *item_header_ = nullptr;
    std::shared_ptr<IFileListOutputHandler> output_handler_;
    DJI_CAMERA_TYPE camera_type_ = DJI_CAMERA_TYPE_UNK;
};
}  // namespace sdk
}  // namespace dji

#endif /* newfilelistrequest_h */
