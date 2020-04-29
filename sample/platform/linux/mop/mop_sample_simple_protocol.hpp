//
// Created by dji on 4/21/20.
//

#ifndef ONBOARDSDK_INTERNAL_ONBOARD_SDK_SAMPLE_PLATFORM_LINUX_MOP_MOP_SAMPLE_SIMPLE_PROTOCOL_HPP_
#define ONBOARDSDK_INTERNAL_ONBOARD_SDK_SAMPLE_PLATFORM_LINUX_MOP_MOP_SAMPLE_SIMPLE_PROTOCOL_HPP_

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
