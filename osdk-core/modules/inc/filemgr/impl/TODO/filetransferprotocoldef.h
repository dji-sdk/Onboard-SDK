#ifndef __filetransferprotocoldef__
#define __filetransferprotocoldef__

#include <stdint.h>
#include <deque>

namespace DJI {
namespace OSDK {

#pragma pack(1)
static const int max_packet_size = 1024;

static unsigned char g_request_buf[max_packet_size] = {0};
// 文件index  定义
typedef PACKED_STRUCT {
    // 文件index 信息
    uint32_t index : 30;
    // 逻辑分区信息， 表示逻辑分区0 - 逻辑分区3
    uint32_t drive : 2;
}
dji_file_index_t;

typedef struct {
    uint32_t startIndex;
    uint16_t file_count;
    uint8_t sub_type;
    uint8_t index;  //当前只有视频剪辑在用, 当为0xFF时使用expend_sub_index
    uint32_t offset;
    uint32_t data_size;
    uint32_t expend_sub_index;
    uint16_t seg_sub_index;  // fat32系统文件超过4g时的subindex
} DownloadRequest;

typedef struct {
    uint8_t like;
    uint32_t video;
    uint32_t photo;
} ListRequestFilter;

//由App向相机发出的文件列表请求
typedef struct {
    uint32_t startIndex;
    uint16_t file_count;
    uint8_t sub_type;
    uint8_t filter_enable;
    ListRequestFilter filters;
} ListRequest;

//由App向设备发出的audio文件列表请求
typedef struct {
    dji_file_index_t file_index;
    uint16_t file_count;
    uint8_t sub_type;
} AudioFileListRequest;

typedef struct {
    uint32_t second : 5;  // value*2
    uint32_t minute : 6;
    uint32_t hour : 5;
    uint32_t day : 5;
    uint32_t month : 4;
    uint32_t year : 7;  // value + 1980
} FileCreationTime;

typedef struct {
    FileCreationTime tm;
    uint32_t file_size;
    uint32_t file_index;
    union {  // support on v2, 只有v2 app以后的版本才支持
        struct {
            uint16_t duration;      // in seconds
            uint8_t frameRate : 6;  // DJICameraVideoFrameRate
            uint8_t rotation : 2;   // DJICameraImageOrientationProtocolType
            uint8_t resoultion;     // DJICameraVideoSizeType
        };
        struct {
            // 保留位
            uint32_t reserved : 22;
            // 照片旋转方向
            uint32_t attribute_photo_rotation : 2;  // enum-type: DJICameraImageOrientationProtocolType
            uint32_t photoratio : 8;
        };
        uint32_t attr;
    };
    uint8_t file_type;
    /*
     * pathLen 小于5时APP不解析，跳过path这个数据段
     * 在pathLen == 5 时，存储4字节VideoGuid + \0
     * 在pathLen >= 6 时，使用 RemoteFileInfoExtType + content + RemoteFileInfoExtType + content... 来扩展
     *
     * 照片没有guid，因此pathLen可能为0，或者pathLen == 5，guid为0.
     *
     */
    uint8_t path_length;
    uint8_t path[1];
} ListDataItem;

//由相机向App返回的文件列表
typedef struct {
    uint32_t file_count;
    uint32_t data_size;  // ListDataPack 长度
    ListDataItem item[1];
} ListDataPack;

typedef struct {
    uint32_t size;
    uint32_t index;
    uint8_t path_len;
    uint8_t data[1];
} FileFragment;

//由相机向App返回的文件数据
typedef struct {
    uint32_t reserved;
    FileFragment frag[1];
} FileData;

typedef struct {
    uint32_t startSeqNum;
    uint32_t fragCount;
} TransMissSegmentRange;

typedef struct {
    uint32_t ackSeq;
    uint8_t missSegmentCount;  //丢失块数，现在默认1块
    TransMissSegmentRange missSegment[0];
} CommonFileAck;

//由App向相机发出的流传输请求，CmdId=0x26
typedef struct {
    uint32_t index;
    uint16_t startPos;
    uint32_t length;
    uint8_t sub_type;
    uint8_t sub_index;
    uint8_t speed_ratio : 7;
    uint8_t speed_flag : 1;
    uint8_t reserved;
} StreamRequest;

//由相机向App返回的流数据，CmdId=0x27
typedef struct {
    uint32_t tick_per_sec;
    uint32_t frame_time;
    uint32_t start_time;
    uint32_t duration;
    uint8_t data[1];
} StreamData;

typedef struct {
    uint32_t code;  //退出原因
} ExitDownload;

enum class CommonDataRequestStatus {
    CommonDataRequestStatusInitial,   // initial status
    CommonDataRequestStatusWorking,   // requesting with camera
    CommonDataRequestStatusFinished,  // finished status (success/failed)
};

enum class RemoteFileInfoTag {
    RemoteFileInfoTagNone = 0,
    RemoteFileInfoTagLike = 1,
};

enum class ProtocolPlaybackPhotoType {
    ProtocolPlaybackPhotoTypeReserved = 0,
    ProtocolPlaybackPhotoTypeNormal = 1,
    ProtocolPlaybackPhotoTypeHDR = 2,
    ProtocolPlaybackPhotoTypePano360 = 3,
    ProtocolPlaybackPhotoTypeBurst = 4,
    ProtocolPlaybackPhotoTypeAEB = 5,
    ProtocolPlaybackPhotoTypeInterval = 6,
    ProtocolPlaybackPhotoTypePanoApp = 7,
};  //相机在file list ext字段中提供的附加信息类型

typedef struct {
    uint8_t take_photo_type;     // ProtocolPlaybackPhotoType
    uint16_t photo_group_index;  //定时拍等成组用的id
} RemoteFileInfoExtPhotoGroupInfo;

// 回放视频类型
enum class ProtocolPlaybackVideoType {
    ProtocolPlaybackVideoTypeNormal = 0,
    ProtocolPlaybackVideoTypeTimeLapse = 1,
    ProtocolPlaybackVideoTypeTransSpeed = 2,
    ProtocolPlaybackVideoTypeQuickShortVideo = 3,
    ProtocolPlaybackVideoTypeQuickHDRVideo = 4,
    ProtocolPlaybackVideoTypeHyperlapse = 5,
    ProtocolPlaybackVideoTypeLoopRecord = 8,
};

// 变速录像视频子类型
enum class ProtocolPlaybackVideoTransSpeedType {
    ProtocolPlaybackVideoTransSpeedTypeSlow = 0,
    ProtocolPlaybackVideoTransSpeedTypeFast = 1,
};

// timelapse录像视频子类型
enum class ProtocolPlaybackVideoHyperlapseType {
    ProtocolPlaybackVideoHyperlapseTypeInterval = 0,
    ProtocolPlaybackVideoHyperlapseTypeSpeedRatio = 1,
};

//视频信息扩展字段, since wm100
typedef struct {
    uint8_t video_type;             // ProtocolPlaybackVideoType
    uint8_t video_sub_type;         // ProtocolPlaybackVideoTransSpeedType
    uint8_t video_encoding_type;    // 264,265
    uint8_t video_framerate_scale;  // 变速录像倍速
    uint8_t reserved;
} RemoteFileInfoExtVideoExtInfo;

typedef struct {
    uint8_t type;
    uint16_t count : 14;
    uint16_t camera_generate_state : 2;
} RemoteFileInfoExtPanoExtInfo;

typedef PACKED_STRUCT {
    uint8_t encoding_format : 5;
    uint8_t reserved : 1;
    uint8_t storage_type : 2;
    uint8_t sampling_rate : 5;  // 采样率
    uint8_t bit_depth : 3;      // 采样位宽
    uint16_t audio_duration;    // 音频时长
    uint8_t file_name_len;
    uint8_t file_name[1];
}
dji_ext_audio_file;

typedef struct {
    uint8_t index;
    uint8_t status;
} VideoClipStateStruct;

typedef struct {
    uint16_t commit_no;
    uint16_t clip_num;
    VideoClipStateStruct clips[1];
} VideoClipListDataStruct;

enum class MediaTaskState {
    IDLE = 0,
    WORKING = 1,
    WAITING_CALLBACK = 2,
};

enum class TransAbortReason {
    TransAbortReasonError = 0,
    TransAbortReasonForce,
    TransAbortReasonSizeError,
    TransAbortReasonReadFailed,
};

template <class T, class FileType>
struct FileBatchContext {
    std::vector<FileType> total_files;
    std::vector<FileType> completed_files;
    std::vector<FileType> failed_files;
    std::deque<T> requests;
    std::deque<std::vector<FileType> > request_aux_info;
};

enum class FileDownloaderState {
    Init,
    WaittingDownloadMode,
    DownloadingFile,
    StartDownloadModeFail,
    WattingDownloadModeCommandFail,
    WattingDownloadModeFail,
    DownloadFail,
    DownloadTimeout,
    Cancel,
    Completed
};

//        enum class PlaybackHandleState
//        {
//            Idle,
//            DownloadingMoov,
//            ReadyToPlay,
//            MoovFailed,
//        };

enum class IOHelperErrorCode { FileClosed, WriteFailed, WriteNotContinues, ReadTimeout, ReadFailed };

#define H264_HEADER_MAGIC 0x01000000

#define CHECK_PTR(ptr)        \
    do {                      \
        if (nullptr == ptr) { \
            return false;     \
        }                     \
    } while (0)

#pragma pack()
}  // namespace sdk

}  // namespace dji

#endif
