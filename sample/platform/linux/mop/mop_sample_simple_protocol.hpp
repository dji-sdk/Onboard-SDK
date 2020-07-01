//
// Created by dji on 4/21/20.
//

#ifndef ONBOARDSDK_INTERNAL_ONBOARD_SDK_SAMPLE_PLATFORM_LINUX_MOP_MOP_SAMPLE_SIMPLE_PROTOCOL_HPP_
#define ONBOARDSDK_INTERNAL_ONBOARD_SDK_SAMPLE_PLATFORM_LINUX_MOP_MOP_SAMPLE_SIMPLE_PROTOCOL_HPP_

#define UTIL_OFFSETOF(type, member)                ((size_t) & ((type *)0 )-> member)

typedef enum {
    FILE_TRANSFOR_CMD_REQUEST = 0x50,
    FILE_TRANSFOR_CMD_ACK = 0x51,
    FILE_TRANSFOR_CMD_RESULT = 0x52,
    FILE_TRANSFOR_CMD_FILE_INFO = 0x60,
    FILE_TRANSFOR_CMD_FILE_DOWNLOAD_REQ = 0x61,
    FILE_TRANSFOR_CMD_FILE_DATA = 0x62,
    FILE_TRANSFOR_CMD_STOP = 0x62,
} T_MopChannel_FileTransforCmd;

typedef enum {
    FILE_TRANSFOR_SUBCMD_REQUEST_UPLOAD = 0x00,
    FILE_TRANSFOR_SUBCMD_REQUEST_DOWNLOAD = 0x01,
} T_MopChannel_FileTransforRequestSubCmd;

typedef enum {
    FILE_TRANSFOR_SUBCMD_ACK_OK = 0x00,
    FILE_TRANSFOR_SUBCMD_ACK_REJECTED = 0x01,
} T_MopChannel_FileTransforAckSubCmd;

typedef enum {
    FILE_TRANSFOR_SUBCMD_RESULT_OK = 0x00,
    FILE_TRANSFOR_SUBCMD_RESULT_FAILED = 0x01,
} T_MopChannel_FileTransforResultSubCmd;

typedef enum {
    FILE_TRANSFOR_SUBCMD_FILE_INFO_DEFAULT = 0xFF,
} T_MopChannel_FileTransforFileInfoSubCmd;

typedef enum {
    FILE_TRANSFOR_SUBCMD_DOWNLOAD_REQUEST = 0xFF,
} T_MopChannel_FileTransforFileDownloadRequestSubCmd;

typedef enum {
    FILE_TRANSFOR_SUBCMD_FILE_DATA_NORMAL = 0x00,
    FILE_TRANSFOR_SUBCMD_FILE_DATA_END = 0x01,
} T_MopChannel_FileTransforFileDataSubCmd;

typedef enum {
    FILE_TRANSFOR_SUBCMD_STOP_UPLOAD = 0x00,
    FILE_TRANSFOR_SUBCMD_STOP_DOWNLOAD = 0x01,
} T_MopChannel_FileTransforStopSubCmd;

typedef enum CmdEnum : uint8_t {
  CMD_REQUEST = 0x50,
  CMD_ACK = 0x51,
  CMD_RESULT = 0x52,

  CMD_FILEINFO = 0x60,
  CMD_DL_FILENAME = 0x61,
  CMD_FILEDATA = 0x62,
} CmdEnum;

enum ReqSubCmd : uint8_t {
  REQ_UPLOAD = 0x00,
  REQ_DOWNLOAD = 0x01,
} ReqSubCmd;

enum AckSubCmd : uint8_t {
  ACK_OK = 0x00,
  ACK_REJECT = 0x01,
} AckSubCmd;

enum RetSubCmd : uint8_t {
  RET_OK = 0x00,
  RET_FAIL = 0x01,
} RetSubCmd;

enum FileDataSubCmd : uint8_t {
  NORMAL_PACK = 0x00,
  END_PACK = 0x01,
} FileDataSubCmd;

#pragma pack(1)
typedef struct fileInfo {
  bool isExist;
  uint32_t fileLength;
  char fileName[32];
  uint8_t md5Buf[16];
} fileInfo;

typedef struct targetFileName {
  char fileName[32];
} targetFileName;

typedef struct sampleProtocolStruct {
  uint8_t cmd;
  uint8_t subcmd;
  uint16_t seq;
  uint32_t dataLen;
  union dataType {
    fileInfo info;
    targetFileName targetFile;
    uint8_t fileData[0];
  } data;
} sampleProtocolStruct;
#pragma pack()

#define SIMPLE_CMD_PACK_LEN  (uint32_t)(4)
#define RAW_DATA_HEADER_LEN  (uint32_t)(SIMPLE_CMD_PACK_LEN + sizeof(uint32_t))
#define FILEINFO_PACK_LEN    (uint32_t)(RAW_DATA_HEADER_LEN + sizeof(fileInfo))


#endif //ONBOARDSDK_INTERNAL_ONBOARD_SDK_SAMPLE_PLATFORM_LINUX_MOP_MOP_SAMPLE_SIMPLE_PROTOCOL_HPP_
