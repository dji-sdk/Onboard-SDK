/** @file dji_file_mgr_internal_define.hpp
 *  @version 4.0
 *  @date July 2020
 *
 *  @brief Data struct defination for file manager
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

#ifndef _DJI_FILE_INTERNAL_DEFINE_HPP_
#define _DJI_FILE_INTERNAL_DEFINE_HPP_

#include <stdint.h>

// 下载文件失败返回值
typedef enum
{
  DJI_DOWNLOAD_FILE_ABORT_ERROR = 1, // 下载错误
  DJI_DOWNLOAD_FILE_ABORT_FORCE = 2, // 强制退出
  DJI_DOWNLOAD_FILE_ABORT_ERROR_FLASH = 3, // FLASH 介质问题导致读取文件错误
  DJI_DOWNLOAD_FILE_ABORT_SESSION_REALLOC = 4, // Session 重复
  DJI_DOWNLOAD_FILE_ABORT_SESSION_BUSY = 5, // Seesion 资源分配错误
  DJI_DOWNLOAD_FILE_ABORT_ERROR_OTHER = 4294967295, // 未知错误
} DJI_DOWNLOAD_FILE_ABORT_ERROR_CODE;

typedef enum : uint32_t {
  TransAbortReasonError = 0,
  TransAbortReasonForce,
  TransAbortReasonSizeError,
  TransAbortReasonReadFailed,
} TransAbortReason;

typedef enum : uint8_t {
  DJI_CAMERA_VIDEO_FRAME_RATE_15FPS = 0, // 14.985
  DJI_CAMERA_VIDEO_FRAME_RATE_24FPS = 1, // 23.976
  DJI_CAMERA_VIDEO_FRAME_RATE_25FPS = 2, // 25.000
  DJI_CAMERA_VIDEO_FRAME_RATE_30FPS = 3, // 29.970
  DJI_CAMERA_VIDEO_FRAME_RATE_48FPS = 4, // 47.952
  DJI_CAMERA_VIDEO_FRAME_RATE_50FPS = 5, // 50FPS
  DJI_CAMERA_VIDEO_FRAME_RATE_60FPS = 6, // 59.940
  DJI_CAMERA_VIDEO_FRAME_RATE_120FPS = 7, // 119.880
  DJI_CAMERA_VIDEO_FRAME_RATE_240FPS = 8, // 239.760
  DJI_CAMERA_VIDEO_FRAME_RATE_480FPS = 9, // 479.520
  DJI_CAMERA_VIDEO_FRAME_RATE_100PS = 10, // 100PS
  DJI_CAMERA_VIDEO_FRAME_RATE_96FPS = 11, // 95.904
  DJI_CAMERA_VIDEO_FRAME_RATE_180FPS = 12, // 179.820
  DJI_CAMERA_VIDEO_FRAME_RATE_TRUE24FPS = 13, // 24
  DJI_CAMERA_VIDEO_FRAME_RATE_TRUE30FPS = 14, // 30
  DJI_CAMERA_VIDEO_FRAME_RATE_TRUE48FPS = 15, // 48
  DJI_CAMERA_VIDEO_FRAME_RATE_TRUE60FPS = 16, // 60
  DJI_CAMERA_VIDEO_FRAME_RATE_90FPS = 17, // 89.910
  DJI_CAMERA_VIDEO_FRAME_RATE_192FPS = 18, // 191.808
  DJI_CAMERA_VIDEO_FRAME_RATE_200FPS = 19, // 200FPS
  DJI_CAMERA_VIDEO_FRAME_RATE_400FPS = 20, // 400FPS
  DJI_CAMERA_VIDEO_FRAME_RATE_8FPS = 21, // 7.5fps(for flir camera)
  DJI_CAMERA_VIDEO_FRAME_RATE_20FPS = 22, // 20
  DJI_CAMERA_VIDEO_FRAME_RATE_8_DOT_8FPS = 23, // 8.8fps
} DJI_CAMERA_VIDEO_FRAME_RATE;

// 多媒体素材（照片，视频）旋转方向(顺时针旋转)
typedef enum : uint8_t {
  DJI_ROTATE_0 = 0, // 旋转角度0°
  DJI_ROTATE_90 = 1, // 旋转角度90°
  DJI_ROTATE_180 = 2, // 旋转角度180°
  DJI_ROTATE_270 = 3, // 旋转角度270°
} DJI_MULTIMEDIA_ROTATE;

typedef enum : uint8_t {
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_REQ = 0, // 请求
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_DATA = 1, // 数据
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ACK = 2, // 应答
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_PUSH = 3, // 催促
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ABORT = 4, // 退出
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_DEL = 5, // 删除
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_PAUSE = 6, // 暂停
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_RESUME = 7, // 继续
} DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE;

typedef enum : uint8_t {
  DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST = 0, // 文件列表
  DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE = 1, // 文件
  DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_STREAM = 2, // 流
  DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_MULTI_STREAM =
  3, // 多类型流，可以是H264、H265、MJPEG等。该类型相比于GENERAL_DOWNLOAD_FILE_TASK_TYPE_STREAM，响应数据段增加了头部信息
} DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE;

typedef enum : uint8_t {
  DJI_CAMERA_VIDEO_RESOLUTION_640X480P = 0, // 640X480P
  DJI_CAMERA_VIDEO_RESOLUTION_640X480I = 1, // 640X480I
  DJI_CAMERA_VIDEO_RESOLUTION_1280X640P = 2, // 1280X640P
  DJI_CAMERA_VIDEO_RESOLUTION_1280X640I = 3, // 1280X640I
  DJI_CAMERA_VIDEO_RESOLUTION_1280X720P = 4, // 1280X720P
  DJI_CAMERA_VIDEO_RESOLUTION_1280X720I = 5, // 1280X720I
  DJI_CAMERA_VIDEO_RESOLUTION_1280X960P = 6, // 1280X960P
  DJI_CAMERA_VIDEO_RESOLUTION_1280X960I = 7, // 1280X960I
  DJI_CAMERA_VIDEO_RESOLUTION_1920X960P = 8, // 1920X960P
  DJI_CAMERA_VIDEO_RESOLUTION_1920X960I = 9, // 1920X960I
  DJI_CAMERA_VIDEO_RESOLUTION_1920X1080P = 10, // 1920X1080P
  DJI_CAMERA_VIDEO_RESOLUTION_1920X1080I = 11, // 1920X1080I
  DJI_CAMERA_VIDEO_RESOLUTION_1920X1440P = 12, // 1920X1440P
  DJI_CAMERA_VIDEO_RESOLUTION_1920X1440I = 13, // 1920X1440I
  DJI_CAMERA_VIDEO_RESOLUTION_3840X1920P = 14, // 3840X1920P
  DJI_CAMERA_VIDEO_RESOLUTION_3840X1920I = 15, // 3840X1920I
  DJI_CAMERA_VIDEO_RESOLUTION_3840X2160P = 16, // 3840X2160P
  DJI_CAMERA_VIDEO_RESOLUTION_3840X2160I = 17, // 3840X2160I
  DJI_CAMERA_VIDEO_RESOLUTION_3840X2880P = 18, // 3840X2880P
  DJI_CAMERA_VIDEO_RESOLUTION_3840X2280I = 19, // 3840X2280I
  DJI_CAMERA_VIDEO_RESOLUTION_4096X2048P = 20, // 4096X2048P
  DJI_CAMERA_VIDEO_RESOLUTION_4960X2048I = 21, // 4960X2048I
  DJI_CAMERA_VIDEO_RESOLUTION_4096X2160P = 22, // 4096X2160P
  DJI_CAMERA_VIDEO_RESOLUTION_4096X2160I = 23, // 4096X2160I
  DJI_CAMERA_VIDEO_RESOLUTION_2704X1520P_16COLON9 = 24, // 2704X1520P,16:9
  DJI_CAMERA_VIDEO_RESOLUTION_640X512P_FLIR = 26, // 640X512P_FLIR
  DJI_CAMERA_VIDEO_RESOLUTION_4608X2160 = 27, // 4608X2160
  DJI_CAMERA_VIDEO_RESOLUTION_4608X2592 = 28, // 4608X2592
  DJI_CAMERA_VIDEO_RESOLUTION_2720X1530P = 31, // 2720X1530P
  DJI_CAMERA_VIDEO_RESOLUTION_5280X2160P = 32, // 5280X2160P
  DJI_CAMERA_VIDEO_RESOLUTION_5280X2970P = 33, // 5280X2970P
  DJI_CAMERA_VIDEO_RESOLUTION_3840X1572P = 34, // 3840X1572P
  DJI_CAMERA_VIDEO_RESOLUTION_5760X3240P = 35, // 5760X3240P
  DJI_CAMERA_VIDEO_RESOLUTION_6016X3200P = 36, // 6016X3200P
  DJI_CAMERA_VIDEO_RESOLUTION_2048X1080P = 37, // 2048X1080P
  DJI_CAMERA_VIDEO_RESOLUTION_336X256P_FLIR = 38, // 336X256P_FLIR
  DJI_CAMERA_VIDEO_RESOLUTION_5120X2880P = 39, // 5120x2880P
  DJI_CAMERA_VIDEO_RESOLUTION_4096X2160P_RAW14 = 40, // 4096X2160P_RAW14
  DJI_CAMERA_VIDEO_RESOLUTION_3840X2160P_RAW14 = 41, // 3840X2160P_RAW14
  DJI_CAMERA_VIDEO_RESOLUTION_2720X1530P_RAW14 = 42, // 2720X1530P_RAW14
  DJI_CAMERA_VIDEO_RESOLUTION_1920X1080P_RAW14 = 43, // 1920X1080P_RAW14
  DJI_CAMERA_VIDEO_RESOLUTION_5440X2880P = 44, // 5440X2880P
  DJI_CAMERA_VIDEO_RESOLUTION_2688X1512P = 45, // 2688X1512P
  DJI_CAMERA_VIDEO_RESOLUTION_640X360P = 46, // 640X360P
  DJI_CAMERA_VIDEO_RESOLUTION_4000X3000P = 48, // 4000X3000P
  DJI_CAMERA_VIDEO_RESOLUTION_4000X3000I = 49, // 4000X3000I
  DJI_CAMERA_VIDEO_RESOLUTION_2880X1620P = 50, // 2880X1620P
  DJI_CAMERA_VIDEO_RESOLUTION_2880X1620I = 51, // 2880X1620I
  DJI_CAMERA_VIDEO_RESOLUTION_2720X2040P = 52, // 2720X2040P
  DJI_CAMERA_VIDEO_RESOLUTION_2720X2040I = 53, // 2720X2040I
  DJI_CAMERA_VIDEO_RESOLUTION_720X576 = 54, // 720X576
  DJI_CAMERA_VIDEO_RESOLUTION_MAX = 253, // MAX
  DJI_CAMERA_VIDEO_RESOLUTION_UNSET = 254, // UNSET
  DJI_CAMERA_VIDEO_RESOLUTION_KNOWN = 255, // KNOWN
} DJI_CAMERA_VIDEO_RESOLUTION;

typedef enum : uint8_t {
  DJI_CAMERA_FILE_TYPE_JPEG = 0,
  DJI_CAMERA_FILE_TYPE_DNG = 1,
  DJI_CAMERA_FILE_TYPE_MOV = 2,
  DJI_CAMERA_FILE_TYPE_MP4 = 3,
  DJI_CAMERA_FILE_TYPE_PANORAMA = 4,
  DJI_CAMERA_FILE_TYPE_TIFF = 5,
  DJI_CAMERA_FILE_TYPE_SEQ = 8, // (for XT2 Camera use)
  DJI_CAMERA_FILE_TYPE_TIFFSEQ = 9, // (for XT2 Camera use)
  DJI_CAMERA_FILE_TYPE_AUDIO = 10, // for WM245 accessory
  DJI_CAMERA_FILE_TYPE_SURVEY = 12,
  DJI_CAMERA_FILE_TYPE_USER_CTRL_INFO = 13, // 用户操作信息的原始文件类型
  DJI_CAMERA_FILE_TYPE_USER_CTRL_INFO_LZ4 = 14, // 用户操作信息使用LZ4压缩后类型
  DJI_CAMERA_FILE_TYPE_JSON = 15, // JSON文件
  DJI_CAMERA_FILE_TYPE_PHOTO_FOLDER = 16, // 照片文件夹
  DJI_CAMERA_FILE_TYPE_VIDEO_FOLDER = 17, // 视频文件夹
} DJI_CAMERA_FILE_TYPE;

// 视频传输数据流格式，app告诉camera想要什么格式的视频流。
typedef enum : uint8_t {
  DJI_TRANS_VIDEO_STREAM_ORG = 0, // 原片码流格式
  DJI_TRANS_VIDEO_STREAM_H264 = 1, // h264
  DJI_TRANS_VIDEO_STREAM_H265 = 2, // h265
  DJI_TRANS_VIDEO_STREAM_MJPEG = 3, // motion jpeg
} DJI_TRANS_VIDEO_STREAM_FORMAT;

// 请求下载的文件子类型
typedef enum : uint8_t {
  DJI_DOWNLOAD_FILE_ORG = 0, // 原文件
  DJI_DOWNLOAD_FILE_THM = 1, // 缩略图
  DJI_DOWNLOAD_FILE_SCR = 2, // 截屏图
  DJI_DOWNLOAD_FILE_CLIP = 3, // 视频剪辑
  DJI_DOWNLOAD_FILE_STREAM = 4, // 流传输
  DJI_DOWNLOAD_FILE_PANO = 5, // 全景照片
  DJI_DOWNLOAD_FILE_PANO_SCR = 6, // 全景照片截屏图
  DJI_DOWNLOAD_FILE_PANO_THM = 7, // 全景照片缩略图
  DJI_DOWNLOAD_FILE_TIMELAPES = 8, // Timelapse照片
  DJI_DOWNLOAD_FILE_2ND_STREAM = 9, // 第二路码流文件
  DJI_DOWNLOAD_FILE_RESERVED = 10, // 预留
  DJI_DOWNLOAD_FILE_PHOTO_METADATA = 11, // 照片元数据
  DJI_DOWNLOAD_FILE_USER_CTRL_INFO = 12, // 用户操作信息数据
  DJI_DOWNLOAD_FILE_JSON = 13, // JSON文件
} DJI_DOWNLOAD_FILE_SUBTYPE;

// 文件列表扩展数据信息类型
typedef enum : uint8_t {
  DJI_EXT_TYPE_UUID = 1, // 视频UUID
  DJI_EXT_TYPE_P_TID = 2, // 照片类型和ID
  DJI_EXT_TYPE_STAR = 3, // 星标
  DJI_EXT_TYPE_V_TID = 4, // 视频类型信息
  DJI_EXT_TYPE_FUSION = 5, // 合成照片Group信息
  DJI_EXT_TYPE_V_FILE_SYNC = 6, // 文件信息同步
  DJI_EXT_TYPE_ORIGIN = 7, // 原片信息
  DJI_EXT_TYPE_EXIF = 8, // EXIF信息
  DJI_EXT_TYPE_AUDIO = 9, // 音频信息
  DJI_EXT_TYPE_VIDEO_MISCINFO = 10, // 视频附加类型信息
  DJI_EXT_TYPE_CUSTOMIZE_DCF_INFO = 11, // 自定义DCF照片信息
  DJI_EXT_TYPE_FILE_NAME = 13, // 文件名信息
  DJI_EXT_TYPE_FILE_MD5 = 14, // 文件MD5值
} DJI_FILE_LIST_EXT_TYPE;

typedef enum : uint8_t {
  DJI_all = 0, // 喜欢不喜欢全都要
  DJI_like = 1, // 喜欢
  DJI_dislike = 2, // 不喜欢
  DJI_unknown = 255, // 未知
} DJI_FILE_LIST_FILTER_LIKE;

typedef enum : uint8_t {
  DJI_MEDIA = 0, // 媒体文件
  DJI_THUMBNAIL = 1, // 缩略图
  DJI_SCREENNAIL = 2, // 截屏图
  DJI_VIDEO_CLIP = 3, // 视频剪辑
  DJI_LOG = 12, // 固件日志文件
  DJI_MEDIA_GROUP = 13, // 相机自动成组的媒体文件
  // 逻辑：
  // 成组逻辑包括 AEB、Burst、Interval、视频大于 4G 自动分段、Pano
  // 1. 相机根据 Index 和 ReqCount 推送相应数量的素材信息，遇到 Group 照片、分段视频等，则只推送组内第一个文件信息
  // 2. AEB、Burst 的张数显示在 0x27 的 dji_ext_fusion 的字段 amount，最大 9999 张
  // 3. 分段视频相机自动计算所有分段合并后的总时长填充到视频 attribute_video_duration 字段
      DJI_PSDK_WIDGET =
      14, // PSDK 自定义控件配置，包括配置文件和图标文件，存储在第三方负载中。DJI Pilot 拉取配置文件和图标文件后可以显示控件，用于显示第三方负载状态数据并控制第三方负载。
} DJI_DOWNLOAD_FILE_LIST_TYPE;

typedef enum : uint8_t {
  MAJOR_CAMERA = 0,
  MAJOR_RADAR = 1,
  MAJOR_PSDK = 2,
} MAJOR_TYPE;

typedef enum : uint8_t {
  MINOR_CAMERA = 0,
  MINOR_PSDK = 2,
} MINOR_TYPE;

#pragma pack(1)

/*! (0x00,0x26) 相关结构体定义*/
// 文件下载请求，由 App 向相机发出的文件下载请求。
typedef struct {
  // 报文头长度
  uint8_t header_length:6;
  // 传输协议版本号
  uint8_t version      :2;
  // 传输协议类型
  uint8_t func_id:5;    // enum-type: DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE
  // 子业务ID
  uint8_t task_id:3;    // enum-type: DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE
  // 报文总长度
  uint16_t msg_length:12;
  // 标志，最低位为1代表是最后一个包
  uint16_t msg_flag  :4;
  // 回话ID
  uint16_t session_id;
  // 报文在业务中的编号
  uint32_t seq;
  // 消息数据
  // dji_file_list_download_req : 文件列表下载请求
  // dji_file_download_req : 文件下载请求
  // dji_stream_download_req : 流下载请求
  // dji_download_ack : 下载应答
  uint8_t data[1];
} dji_general_transfer_msg_req;

// 文件列表过滤器  - 视频
// 各素材类型比特位如果为 **1** 表示需要过滤出该类型素材，为 **0** 则不过滤出来
typedef struct {
  // 普通录像
  uint32_t video_nomal     :1;
  // 慢动作录像
  uint32_t video_slowmotion:1;
  // 延时视频（拍照合成方式）
  uint32_t video_timelapse :1;
  // Hyperlapse
  uint32_t video_hyperlapse:1;
  // HDR录像
  uint32_t video_hdr       :1;
  // 循环录像
  uint32_t video_loop      :1;
  // 保留位
  uint32_t reserved        :26;
} dji_file_list_filter_video;

// 文件列表过滤器  - 照片
// 各素材类型比特位如果为 **1** 表示需要过滤出该类型素材，为 **0** 则不过滤出来
typedef struct {
  // 普通照片
  uint32_t photo_normal  :1;
  // HDR 照片
  uint32_t photo_hdr     :1;
  // AEB 照片
  uint32_t photo_aeb     :1;
  // 定时拍
  uint32_t photo_interval:1;
  // Burst 照片
  uint32_t photo_burst   :1;
  // 全景照片
  uint32_t photo_pano    :1;
  // 保留位
  uint32_t reserved      :26;
} dji_file_list_filter_photo;

// **文件列表下载过滤器**
// <br/>
// 过滤器组合规则：
// 1. **like** 过滤器作用于后续所有过滤器的结果，进行**逻辑与**运算；
// 2. **video**、**photo** 等 过滤器内部通过**逻辑或**运算取值，各过滤器结果取并集
// ```
// like && ((video) U (photo))
// 如：
// like = 1
// video = 1
// photo = 3
// 即表示过滤：所有(普通录像、普通照片、HDR 照片)中 被 like 的文件。
// ```
typedef struct {
  // 媒体文件是否被喜欢加星
  uint8_t like;    // enum-type: DJI_FILE_LIST_FILTER_LIKE
  // 视频素材过滤器，按位或取值
  dji_file_list_filter_video video;
  // 照片素材过滤器，按位或取值
  dji_file_list_filter_photo photo;
} dji_download_file_list_filter;

// 文件index  定义
typedef struct {
  // 文件index 信息
  uint32_t index:30;
  // 逻辑分区信息， 表示逻辑分区0 - 逻辑分区3
  uint32_t drive:2;
} dji_file_index_t;

// 文件列表下载请求
typedef struct {
  // 起始文件Index（UINT32，LSB在前，MSB在后）。
  // 注：=1表示从第一个文件开始
  dji_file_index_t index;
  // 文件个数（UINT16，LSB在前，MSB在后）
  // 注：=0xFFFF表示传之后所有的列表信息
  uint16_t count;
  // 请求下载信息的文件子类型
  uint8_t type;    // enum-type: DJI_DOWNLOAD_FILE_LIST_TYPE
  // **type == DJI_MEDIA_GROUP 时生效**
  // <br/><br/>
  // filter_enable 为 `0` 时，表示不启用过滤器，从所有素材中进行文件列表拉取。<br/>
  // filter_enable 为 `1` 时，表示启用过滤器，根据 filters 过滤规则进行文件过滤。
  uint8_t filter_enable;
  // **文件列表筛选过滤器**
  // <br/>
  // filter_enable 为 `1` 时生效
  // download_file_list_filter
  dji_download_file_list_filter filters;
} dji_file_list_download_req;

// 消息数据 – 文件下载请求（由App向相机发出的文件下载请求）
typedef struct {
  // 起始文件Index（UINT32，LSB在前，MSB在后）。
  dji_file_index_t index;
  // 请求下载的文件个数（UINT16，LSB在前，MSB在后）。
  // 注：
  // 当为0xffff时为传之后所有的文件
  // 当前只支持1
  uint16_t count;
  // 请求下载的文件子类型
  uint8_t type;    // enum-type: DJI_DOWNLOAD_FILE_SUBTYPE
  // 请求下载的文件子索引
  // 当此字段为0xFF时，使用扩展文件子索引字段
  uint8_t sub_index;
  // 请求下载的文件数据偏移（UINT32，LSB在前，MSB在后）。
  uint32_t offset;
  // 请求下载的文件数据大小（UINT32，LSB在前，MSB在后）
  uint32_t size;
  // 扩展文件子索引
  uint32_t ext_sub_index;
  // 扩展分段文件子索引
  uint16_t seg_sub_index;
} dji_file_download_req;

// 流数据传输
typedef struct {
  // 流文件Index
  dji_file_index_t index;
  // 请求传输的流起始时间，单位ms
  uint32_t offset;
  union {
    // 请求传输的流时长，单位ms， -1表示传输至本文件流结束，-2表示跨文件一直传输直到后续的所有文件流结束
    int32_t duration;
    // 请求传输的文件个数， -1表示传输之后的所有文件，目前只有video_type=DJI_TRANS_VIDEO_STREAM_MJPEG时使用
    int32_t count;
  };
  // 请求传输的流子类型：视频码流格式
  uint8_t video_type:4;    // enum-type: DJI_TRANS_VIDEO_STREAM_FORMAT
  // 请求传输的流子类型：音频开关及码流格式。0表示不需要音频，1表示原片码流格式的音频
  uint8_t audio_type:4;
  // 请求传输的流子索引（用于扩展）
  uint8_t subindex;
  // 播放速率控制。
  // [7] – 加速/减速标志
  // [6:0] – 变速倍数
  uint8_t speed_ctrl;
  // 视频分辨率
  uint8_t resolution;    // enum-type: DJI_CAMERA_VIDEO_RESOLUTION
  // 视频码率
  uint16_t bitrate;
} dji_stream_download_req;

// 丢失分片描述
typedef struct {
  // 起始分片序号
  uint32_t seq;
  // 分片个数
  uint32_t cnt;
} dji_loss_desc;

// 文件下载应答消息
typedef struct {
  // 期望收到的分片序号，表示已收到expect_seq之前的所有分片。
  uint32_t expect_seq;
  // 丢包信息个数
  uint8_t loss_nr;
  // 丢包信息
  dji_loss_desc loss_desc[1];
} dji_download_ack;

typedef struct {
  // Return code
  uint8_t ret_code;
} dji_general_transfer_msg_rsp;
/////////////(0x00,0x26) 相关结构体定义////////////

/*! (0x00,0x27) 相关结构体定义*/

// 文件列表扩展数据——照片类型和ID
typedef struct {
// 照片类型
  uint8_t type;    // enum-type: DJI_EXT_P_TID_TYPE
// 照片ID，同一组照片使用相同的照片ID，普通单拍的情况下设置成0
  uint16_t id;
} dji_ext_p_tid;

// 文件列表扩展数据——视频UUID
typedef struct {
// 视频UUID
  uint32_t uuid;
} dji_ext_video_uuid;

// 文件列表扩展数据——星标
typedef struct {
// 星标数值
  uint8_t star;
} dji_ext_star;

// 文件列表扩展数据内容——视频类型信息
typedef struct {
// 视频类型
  uint8_t type;    // enum-type: DJI_EXT_V_TID_TYPE
// 视频子类型
// 当视频类型 = 变速视频时：
// 0 = 慢速视频
// 1 = 快速视频
// 其他 = 保留
// 当视频类型 = 一键短片时：
// 0 = 保留
// 1 = 环绕
// 2 = 斜飞
// 3 = 螺旋
// 4 = 火箭
// 其他 = 保留
// 当视频类型 = hyperlapse视频时：
// 0 = hyperlapse with  interval
// 1 = hyperlapse with speed ratio
// 其他 = 保留
  uint8_t subtype;
// 编码格式
// 0 = H.264
// 1 = H.265
// 其他 = 保留
  uint8_t encode_format;
// 变速视频播放倍速（仅对变速视频有效）
// 对于慢速视频：
// 0 = default
// 1 = 1倍速
// 2 = 1/2倍速
// N = 1/N倍速
// 对于快速视频：
// 0 = default
// 1 = 1倍速
// 2 = 2倍速
// N = N倍速
  uint8_t speed;
// 保留
  uint8_t reserved;
} dji_ext_v_tid;

// 文件列表扩展数据内容—合成照片Group信息，用于相机端使用子文件夹方式存放原照片的情况
typedef struct {
// 照片子类型
// 对于全景照片：
// 0 = 保留
// 1 = 360 Panorama
// 2 = Ball Panorama
// 3 = Vertical Panorama
// 4 = 180 Panorama(云台横拍状态)
// 5 = Wide Panorama(云台横拍状态)
// 6 = 180 Panorama(云台竖拍状态)
// 7 = Wide Panorama(云台竖拍状态)
// 其他 = 保留
// 对于OSMO（X3X5系列）对于全景照片：
// 0 = 保留
// 1 = 360 Panorama
// 2 = Ball Panorama
// 3 = Self Panorama
// 4 = Manual Panorama
// 5 =180 Panorama
// 6 = Vertical Panorama
// 7 = Sectorial Panorama
// 其他 = 保留
  uint8_t subtype;
// 原片张数(0~9999)
  uint16_t amount:14;
// 机内合成结果，APP合成忽略此字段
// 0 = 保留
// 1 = 成功
// 2 = 失败
// 3 = 中止
  uint16_t result:2;
} dji_ext_fusion;

// 文件列表扩展数据——文件同步，用于合成的全景图从APP反向下载到相机的同步指示
typedef struct {
// 1 - 已同步
// 其他 - 保留
  uint8_t sync;
} dji_ext_file_sync;

// 文件列表扩展数据——原片信息
typedef struct {
// 是否有原片
// 0 -- 不存在
// 1 -- 存在
  uint8_t exist_flag:2;
// 保留
  uint8_t reserved  :6;
} dji_ext_origin;

// 文件扩展数据——exif信息
typedef struct {
// exposure_time是否有效
  uint8_t enable_exposure_time           :1;
// fnumber是否有效
  uint8_t enable_fnumber                 :1;
// exposure_program是否有效
  uint8_t enable_exposure_program        :1;
// iso是否有效
  uint8_t enable_iso                     :1;
// exposure_compensation是否有效
  uint8_t enable_exposure_compensation   :1;
// metering_mode是否有效
  uint8_t enable_metering_mode           :1;
// light_source是否有效
  uint8_t enable_light_source            :1;
// focal_length_35mm_format是否有效
  uint8_t enable_focal_length_35mm_format:1;
// exposure_time分子
  uint32_t exposure_time_num;
// exposure_time分母
  uint32_t exposure_time_den;
// fnumber分子
  uint32_t fnumber_num;
// fnumber分母
  uint32_t fnumber_den;
// exposure_program
  uint16_t exposure_program;
// iso
  uint16_t iso;
// exposure_compensation分子
  int32_t exposure_compensation_num;
// exposure_compensation分母
  int32_t exposure_compensation_den;
// metering_mode
  uint16_t metering_mode;
// light_source
  uint16_t light_source;
// focal_length_35mm_format
  uint16_t focal_length_35mm_format;
} dji_ext_exif;

// 文件列表扩展数据——视频杂项信息
typedef struct {
// 分段视频子索引
  uint16_t seg_idx;
// 视频时长（毫秒单位）
  uint32_t video_time_in_ms;
  union
  {
// 当视频类型为1 = 延时视频，或视频类型为5（hyperlapse）且视频子类型是0（hyperlapse with interval）时,  表示延时视频帧间隔（单位：100ms）
    uint32_t video_timelapse_interval_100ms;
// 当视频类型为5（hyperlapse），且视频子类型是1（hyperlapse with speed ratio）时，表示hyperlapse视频倍速值
    uint32_t video_hyperlapse_speed_ratio;
// 其他视频类型时， 此字段保留
    uint32_t reserved;
  };
} dji_ext_misc_video;

// 描述照片或者视频文件的来源，设置拍照或录像要保存的流请参考capture_recording_streams
typedef struct
{
  // 文件来源
  uint8_t file_source;	// enum-type: DJI_DCF_EXT_FILE_SOURCE
  // 如果是相机原始照片，这里表明相机来源，不是原始照片时，值为0(DJI_DEFAULT_CAM)
  uint8_t source_camera;	// enum-type: DJI_STREAM_SOURCE_CAMERA
} dji_dcf_ext_file_source;
// 飞机起飞时间

typedef struct
{
  // 保留位
  uint32_t reserved:5;
  // 分
  uint32_t minute  :6;
  // 时
  uint32_t hour    :5;
  // 日
  uint32_t day     :5;
  // 月
  uint32_t month   :4;
  // 年（从1980年开始）
  uint32_t year    :7;
} dji_departure_time;

// 自定义DCF文件信息
typedef struct {
// 用户标示信息
  uint8_t customKey[8];
// 自定义dcf文件来源
  dji_dcf_ext_file_source dcf_ext_file_source;
// 起飞时间
  dji_departure_time departure_time;
// 自定义dcf文件所在目录Index
  uint8_t dcf_directory_index;
// 自定义dcf文件文件Index
  uint32_t dcf_file_index;
// 自定义dcf文件所在集合Id
  uint32_t dcf_file_setId;
} dji_ext_customize_dcf_info;

// 媒体文件文件名信息
typedef struct {
// 文件名长度
  uint8_t file_name_len;
// 文件名字符串（不包括空字符）
  uint8_t file_name[1];
} dji_ext_file_name;

// 文件MD5值信息
typedef struct {
// MD5值校验，用于DJI Pilot 校验PSDK 自定义控件配置文件。DJI Pilot 会在本地缓存PSDK 自定义控件配置文件，如果MD5不一致才会重新拉取配置文件。
  uint8_t file_md5[16];
} dji_ext_file_md5;

// 扩展数据描述符
typedef struct {
  // 扩展信息数据类型
  uint8_t id;    // enum-type: DJI_FILE_LIST_EXT_TYPE
  // 扩展信息数据内容
  // ext_video_uuid : 视频UUID
  // ext_p_tid : 照片类型和ID
  // ext_star : 星标
  // ext_v_tid : 视频类型信息
  // ext_fusion : 合成照片Group信息
  // ext_file_sync : 文件同步
  // ext_origin : 原片信息
  // ext_exif : EXIF信息
  // ext_misc_video : 视频杂项信息
  // ext_customize_dcf_info : 自定义DCF格式文件信息
  // ext_file_name : 文件名信息
  // ext_file_md5 : 文件MD5值校验
  uint8_t data[1];
} dji_ext_info_descriptor;

// 文件列表扩展信息
typedef struct {
  // 扩展信息项，注意新增扩展信息项时必须从尾部开始添加
  dji_ext_info_descriptor ext_info_item[1];
  // 扩展数据结束标记。固定为'\0'
  uint8_t eof;
} dji_file_list_ext_info;

// file create time
typedef struct {
  // 双秒（实际值 = 双秒 x 2）
  uint32_t second:5;
  // 分
  uint32_t minute:6;
  // 时
  uint32_t hour  :5;
  // 日
  uint32_t day   :5;
  // 月
  uint32_t month :4;
  // 年（从1980年开始）
  uint32_t year  :7;
} dji_file_create_time;

// 文件列表信息描述符
typedef struct {
  // 文件的创建时间
  dji_file_create_time create_time;
  // 文件大小
  uint32_t size;
  // 文件索引
  uint32_t index;
  union {
    struct video_attribute_type {
      // 视频时长，单位（秒）
      uint32_t attribute_video_duration  :16;
      // 视频帧率
      uint32_t
          attribute_video_framerate :6;// enum-type: DJI_CAMERA_VIDEO_FRAME_RATE
      // 视频旋转方向
      uint32_t attribute_video_rotation  :2;// enum-type: DJI_MULTIMEDIA_ROTATE
      // 视频分辨率
      uint32_t
          attribute_video_resolution:8;// enum-type: DJI_CAMERA_VIDEO_RESOLUTION
    } video_attribute;
    struct photo_attribute_type {
      // 保留位
      uint32_t attribute_photo_reserved:22;
      // 照片旋转方向
      uint32_t attribute_photo_rotation:2;	// enum-type: DJI_MULTIMEDIA_ROTATE
      // 照片宽高比
      uint32_t attribute_photo_ratio   :8;	// enum-type: DJI_CAMERA_PHOTO_RATIO
    } photo_attribute;
  } attribute;
  // 文件类型
  uint8_t type;    // enum-type: DJI_CAMERA_FILE_TYPE
  // 扩展数据长度
  uint8_t ext_size;
  // 扩展数据内容，可以包含多项数据，每项数据包括数据类型（1字节）和数据内容（N字节）。
  // 在最后一个扩展数据末尾添加1字节的’\0’结尾。
  // 不同类型的文件的扩展数据长度保持一致，长度不足时在末端补’\0’。
  dji_file_list_ext_info ext_data;
} dji_list_info_descriptor;

// 文件列表应答
typedef struct {
  // 文件总数
  uint32_t amount;
  // 文件列表信息长度，包括文件总数字段
  uint32_t len;
  // 文件列表信息描述符
  dji_list_info_descriptor list_info[1];
} dji_file_list_download_resp;

// camera发给app的stream数据包的头信息。
typedef struct {
  // stream的格式
  uint8_t format :6;    // enum-type: DJI_TRANS_STREAM_FORMAT
  // 版本号
  uint8_t version:2;
  // 该数据所在的流文件Index
  dji_file_index_t index;
  // 时间戳，单位us
  int64_t timestamp;
  // 数据的长度，不包含本头信息
  uint32_t length;
  // 数据
  uint8_t data[1];
} dji_multi_stream_download_rsp;

// 0x00 0x27 文件下载返回结构体定义
typedef struct {
  union {
    // 文件列表应答数据
    dji_file_list_download_resp file_list_download_rsp;
    // 多类型流传输应答数据，单一码流无header信息（H264流）
    dji_multi_stream_download_rsp multi_stream_download_rsp;
    // 错误码
    uint32_t abort_rsp;    // enum-type: DJI_DOWNLOAD_FILE_ABORT_ERROR_CODE
  };
} dji_download_file_ack_data;

// 文件下载请求的ack，由相机向App发出
typedef struct {
  // 报文头长度
  uint8_t header_length:6;
  // 传输协议版本号
  uint8_t version      :2;
  // 传输协议类型
  uint8_t func_id:5;    // enum-type: DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE
  // 子业务ID
  uint8_t task_id:3;    // enum-type: DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE
  // 报文总长度
  uint16_t msg_length:12;
  // 标志，最低位为1代表是最后一个包
  uint16_t msg_flag  :4;
  // 回话ID
  uint16_t session_id;
  // 报文在业务中的编号
  uint32_t seq;
  // 消息数据
  // dji_download_file_ack_data : 返回数据定义
  uint8_t data[1];
} dji_general_transfer_msg_ack;
/////////////(0x00,0x26) 相关结构体定义////////////

typedef struct {
  uint32_t reserve;
  uint32_t size;
  uint32_t index;
  uint8_t is_next_valid; //not used ,set to 0
  uint8_t file_data[1];
} dji_file_data_download_resp;

#pragma pack()

#endif //_DJI_FILE_INTERNAL_DEFINE_HPP_
