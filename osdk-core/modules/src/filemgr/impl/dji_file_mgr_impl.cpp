/** @file dji_file_mgr_impl.cpp
 *  @version 4.0
 *  @date July 2020
 *
 *  @brief Implementation for file manager
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

#include "dji_file_mgr_impl.hpp"
#include "dji_linker.hpp"
#include "dji_linker.hpp"
#include "osdk_device_id.h"
#include "dji_command.hpp"
#include "osdk_command.h"
#include "osdk_protocol.h"
#include "dji_internal_command.hpp"
#include "dji_log.hpp"

using namespace DJI;
using namespace DJI::OSDK;

#define V1_HEADR_AND_CRC_LEN (11 + 2)

E_OsdkStat downloadFileAckCB(struct _CommandHandle *cmdHandle,
                                      const T_CmdInfo *cmdInfo,
                                      const uint8_t *cmdData,
                                      void *userData) {
  if ((!cmdInfo) || (!userData)){
    DERROR("Recv Info is a null value");
    return OSDK_STAT_ERR;
  }

  T_ProtocolOps V1_ops;
  /*! 1.Get the v1 ops first, the data in the usbmc packet is a v1 packet */
  if (OsdkProtocol_getProtocolOps(PROTOCOL_V1, &V1_ops) != OSDK_STAT_OK) {
    DERROR("Get V1 ops failed");
    return OSDK_STAT_SYS_ERR;
  }

  uint32_t usedDataCnt = 0;
  while (usedDataCnt < cmdInfo->dataLen) {
    /*! 2.Get V1 data length*/
    uint8_t buffer[1024] = {0};
    T_CmdInfo V1_info = {0};
    uint32_t fullV1FrameLen = 0;
    if (V1_ops.GetFrameLen((char *) cmdData + usedDataCnt, &fullV1FrameLen)
        != OSDK_STAT_OK) {
      DERROR("Get V1 packet data length failed");
      return OSDK_STAT_SYS_ERR;
    }

    /*! 3.Check V1 and data length */
    if ((usedDataCnt + fullV1FrameLen) > cmdInfo->dataLen) {
      DERROR("Next V1 packet data length too big : %d > %d ",
             (usedDataCnt + fullV1FrameLen), cmdInfo->dataLen);
      return OSDK_STAT_SYS_ERR;
    }
    if ((fullV1FrameLen - V1_HEADR_AND_CRC_LEN) > sizeof(buffer)) {
      DERROR("Received V1 packet data length too big : %d",
             (fullV1FrameLen - V1_HEADR_AND_CRC_LEN));
      return OSDK_STAT_SYS_ERR;
    }

    /*! 4.Do V1 packet unpacking */
    if (V1_ops.Unpack(NULL, (uint8_t *) (cmdData + usedDataCnt), &V1_info, buffer)
        == OSDK_STAT_OK) {
      FileMgrImpl *fileMgrImpl = (FileMgrImpl *) userData;
      fileMgrImpl->HandlePushPack((dji_general_transfer_msg_ack *) buffer);
      usedDataCnt += (V1_info.dataLen + V1_HEADR_AND_CRC_LEN);
    } else {
      DERROR("V1 unpack failed in downloading.");
      return OSDK_STAT_SYS_ERR;
    }
    //printf("usedDataCnt=%d cmdInfo->dataLen=%d\n", usedDataCnt, cmdInfo->dataLen);
  }

  return OSDK_STAT_OK;
}

void FileMgrImpl::printFileDownloadStatus() {
    uint32_t lossPackCnt = 0;
    uint32_t recvPackCnt = 0;
    for (auto &msg : fileDataHandler->range_handler_->GetNoAckRanges()) {
      lossPackCnt += msg.length;
    }
    recvPackCnt = fileDataHandler->range_handler_->GetLastNotReceiveSeq() - lossPackCnt;
    DSTATUS("\033[0;32m[Complete rate : %0.1f%%] (recv:%dpacks loss:%dpacks) \033[0m",
            (recvPackCnt * 800 * 100.0f / fileDataHandler->mmap_file_buffer_->fdAddrSize),
            recvPackCnt, lossPackCnt);
}

void FileMgrImpl::fileListMonitorTask(void *arg) {
  DSTATUS("OSDK download monitor task created.");
  if(arg) {
    uint32_t curTimeMs = 0;
    uint32_t preTimeMs = 0;
    uint32_t pollTimeMsInterval = 500;
    uint32_t taskTimeOutMs = 6000;
    FileMgrImpl *impl = (FileMgrImpl *)arg;
    OsdkOsal_GetTimeMs(&curTimeMs);
    impl->fileListHandler->updateTimeMs = curTimeMs;
    for (;;)
    {
      uint32_t refreshTimeMs = impl->fileListHandler->updateTimeMs;
      OsdkOsal_GetTimeMs(&curTimeMs);

      /*! Task timeout */
      if (curTimeMs - refreshTimeMs >= taskTimeOutMs) {
        DSTATUS("curTimeMs:%d refreshTimeMs:%d", curTimeMs, refreshTimeMs);
        DERROR("downloadMonitorTask timeout!! device type : %d index: %d", impl->type, impl->index);

          if (impl->fileListHandler->downloadState == RECVING_FILE_LIST) {
            impl->SendAbortPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST);
            auto cb = impl->fileListHandler->reqCB;
            void *udata = impl->fileListHandler->reqCBUserData;
              FilePackage defaultPack;
              defaultPack.type = FileType::UNKNOWN;
              defaultPack.media.clear();
              if(cb) cb(OSDK_STAT_ERR, defaultPack, udata);
            DSTATUS("Finish req filelist task cause of timeout, reset downloadState to be DOWNLOAD_IDLE");
            impl->fileListHandler->downloadState = DOWNLOAD_IDLE;
          }
      }

      if (impl->fileListHandler->downloadState == DOWNLOAD_IDLE) return;

      OsdkOsal_TaskSleepMs(10);
    }
  } else {
    DERROR("task run failed because of the invalid"
           " FileMgrImpl ptr. Please recheck this task params.");
  }
}


void FileMgrImpl::fileDataMonitorTask(void *arg) {
  DSTATUS("OSDK download filedata monitor task created.");
  if(arg) {
    uint32_t curTimeMs = 0;
    uint32_t preTimeMs = 0;
    uint32_t pollTimeMsInterval = 500;
    uint32_t taskTimeOutMs = 6000;
    FileMgrImpl *impl = (FileMgrImpl *)arg;
    OsdkOsal_GetTimeMs(&curTimeMs);
    impl->fileDataHandler->updateTimeMs = curTimeMs;
    for (;;)
    {

      uint32_t refreshTimeMs = impl->fileDataHandler->updateTimeMs;
      OsdkOsal_GetTimeMs(&curTimeMs);

      /*! Task timeout */
      if (curTimeMs - refreshTimeMs >= taskTimeOutMs) {
        DSTATUS("curTimeMs:%d refreshTimeMs:%d", curTimeMs, refreshTimeMs);
        DERROR("downloadMonitorTask timeout!! device type : %d index: %d", impl->type, impl->index);

        if (impl->fileDataHandler->downloadState == RECVING_FILE_DATA) {
          impl->SendAbortPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE);
          auto cb = impl->fileDataHandler->reqCB;
          void *udata = impl->fileDataHandler->reqCBUserData;
          if (cb) cb(OSDK_STAT_ERR, udata);
          DSTATUS("Finish req filedata task cause of timeout, reset downloadState to be DOWNLOAD_IDLE");
          impl->fileDataHandler->downloadState = DOWNLOAD_IDLE;
        }
      } else if (curTimeMs - refreshTimeMs >= (taskTimeOutMs * 2 / 3)) {
        DSTATUS("The second time to wake up the pushing");
        impl->SendReqFileDataPack(impl->fileDataHandler->curTargetFileIndex);
      } else if (curTimeMs - refreshTimeMs >= (taskTimeOutMs * 1 / 3)) {
        DSTATUS("The first time to wake up the pushing");
        impl->SendReqFileDataPack(impl->fileDataHandler->curTargetFileIndex);
      }

      if (impl->fileDataHandler->downloadState == DOWNLOAD_IDLE) return;

      if (curTimeMs - preTimeMs >=  pollTimeMsInterval)
      {
        /*! Here to send the miss ack packs*/
        if (impl->fileDataHandler->downloadState == RECVING_FILE_DATA) {
          impl->printFileDownloadStatus();
          impl->SendMissedAckPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE);
        }
        preTimeMs = curTimeMs;
      }

      /*! TODO: with out sleep 100ms, the time will get the same as last time. */
      OsdkOsal_TaskSleepMs(10);
    }
  } else {
    DERROR("task run failed because of the invalid"
           " FileMgrImpl ptr. Please recheck this task params.");
  }
}

FileMgrImpl::FileMgrImpl(Linker *linker, E_OSDKCommandDeiveType type,
                         uint8_t index) : linker(linker),
                                          type(type),
                                          index(index) {
  fileListHandler = new DownloadListHandler();
  fileDataHandler = new DownloadDataHandler();
  localSenderId = OSDK_COMMAND_DEVICE_ID(OSDK_COMMAND_DEVICE_TYPE_APP, 0);
  static bool registerCBFlag = false;
  if (!registerCBFlag) {
    registerCBFlag = true;
    static T_RecvCmdItem bulkCmdList[] = {
        PROT_CMD_ITEM(0, 0, V1ProtocolCMD::Common::downloadFileAck[0], V1ProtocolCMD::Common::downloadFileAck[1], MASK_HOST_DEVICE_SET_ID, this, downloadFileAckCB),
    };
    T_RecvCmdHandle recvCmdHandle;
    recvCmdHandle.cmdList = bulkCmdList;
    recvCmdHandle.cmdCount = sizeof(bulkCmdList) / sizeof(T_RecvCmdItem);
    recvCmdHandle.protoType = PROTOCOL_USBMC;
    if (!linker->registerCmdHandler(&recvCmdHandle)) {
      DERROR("register download file callback handler failed, exiting.");
    } else {
      DSTATUS("register download file callback handler successfully.");
    }
  }
}

FileMgrImpl::~FileMgrImpl(){
  if (fileListHandler) {
    delete fileListHandler;
  }
  if (fileDataHandler) {
    delete fileDataHandler;
  }
}


ErrorCode::ErrorCodeType FileMgrImpl::SendReqFileListPack() {
  uint8_t reqBuf[1024] = {0};
  dji_general_transfer_msg_req
      *setting = (dji_general_transfer_msg_req *) reqBuf;
  setting->version = 1;
  setting->header_length = 10;
  setting->task_id = DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST;
  setting->func_id = DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_REQ;
  setting->msg_flag = 0;
  setting->session_id = 0;
  setting->seq = 0;

  dji_file_list_download_req reqData = {0};
  reqData.index.drive = 0;
  reqData.index.index = 1;
  reqData.count = 0xffff;
  reqData.type = DJI_MEDIA;
  uint32_t reqDataLen =
      sizeof(reqData) - sizeof(reqData.filter_enable)
          - sizeof(reqData.filters);
  memcpy(setting->data, &reqData, reqDataLen);
  setting->msg_length = sizeof(dji_general_transfer_msg_req) + reqDataLen - 1;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024] = {0};

  cmdInfo.cmdSet = V1ProtocolCMD::Common::downloadFile[0];
  cmdInfo.cmdId = V1ProtocolCMD::Common::downloadFile[1];
  cmdInfo.dataLen = setting->msg_length;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(type, index);
  cmdInfo.sender = localSenderId; //linker->getLocalSenderId();

//    printf("-------------->request data :\n");
//    for (int i = 0; i < cmdInfo.dataLen; i++) {
//      printf("%02X ", ((uint8_t *) setting)[i]);
//    }
//    printf("\n");

  E_OsdkStat linkAck =
      linker->sendSync(&cmdInfo, (uint8_t *) setting, &ackInfo, ackData,
                       2 * 1000 / 4, 4);

  /*! @TODO fix H20T route */
  if (localSenderId != linker->getLocalSenderId()) return ErrorCode::SysCommonErr::Success;

  ErrorCode::ErrorCodeType ret = ErrorCode::getLinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  return ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                 ErrorCode::CameraCommon, ackData[0]);
}

ErrorCode::ErrorCodeType FileMgrImpl::SendReqFileDataPack(int fileIndex) {
  uint8_t reqBuf[1024] = {0};
  dji_general_transfer_msg_req
      *setting = (dji_general_transfer_msg_req *) reqBuf;
  setting->version = 1;
  setting->header_length = 10;
  setting->task_id = DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE;
  setting->func_id = DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_REQ;
  setting->msg_flag = 0;
  setting->session_id = 0;
  setting->seq = 0;

  dji_file_download_req reqData = {0};
  reqData.index.drive = 0;
  reqData.index.index = fileIndex;
  reqData.count = 1;
  reqData.type = DJI_MEDIA;
  reqData.sub_index = 0;
  reqData.offset = 0;
  reqData.size = (uint32_t) (-1);
  uint32_t reqDataLen = sizeof(reqData) - sizeof(reqData.ext_sub_index)
      - sizeof(reqData.seg_sub_index);
  memcpy(setting->data, &reqData, reqDataLen);
  setting->msg_length = sizeof(dji_general_transfer_msg_req) + reqDataLen - 1;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024] = {0};

  cmdInfo.cmdSet = V1ProtocolCMD::Common::downloadFile[0];
  cmdInfo.cmdId = V1ProtocolCMD::Common::downloadFile[1];
  cmdInfo.dataLen = setting->msg_length;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_FINISH_ACK;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(type, index);
  cmdInfo.sender = localSenderId; //linker->getLocalSenderId();

//    printf("-------------->request data :\n");
//    for (int i = 0; i < cmdInfo.dataLen; i++) {
//      printf("%02X ", ((uint8_t *) setting)[i]);
//    }
//    printf("\n");

  E_OsdkStat linkAck =
      linker->sendSync(&cmdInfo, (uint8_t *) setting, &ackInfo, ackData,
                       1000, 1);

  /*! @TODO fix H20T route */
  if (localSenderId != linker->getLocalSenderId()) return ErrorCode::SysCommonErr::Success;

  ErrorCode::ErrorCodeType ret = ErrorCode::getLinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  return ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                 ErrorCode::CameraCommon, ackData[0]);
}

ErrorCode::ErrorCodeType FileMgrImpl::startReqFileList(FileMgr::FileListReqCBType cb, void* userData) {
  //SendAbortPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST);
  if (fileListHandler->downloadState == DOWNLOAD_IDLE) {
    fileListHandler->downloadState = RECVING_FILE_LIST;
    if (fileListHandler->download_buffer_) {
      fileListHandler->download_buffer_->Clear();
      fileListHandler->download_buffer_->InitBufferQueue(5000, 0); //CSDK 5000
    } else return ErrorCode::SysCommonErr::AllocMemoryFailed;

    if (fileListHandler->range_handler_) fileListHandler->range_handler_->DeInit();
    else return ErrorCode::SysCommonErr::AllocMemoryFailed;

    /*! Create file list req task*/
    OsdkOsal_TaskCreate(&reqFileListHandle,
                        (void *(*)(void *)) (&fileListMonitorTask),
                        OSDK_TASK_STACK_SIZE_DEFAULT, this);

    fileListHandler->reqCB = cb;
    fileListHandler->reqCBUserData = userData;

    return SendReqFileListPack();
  } else {
    DERROR("Current state cannot support to do downloading ...");
    return ErrorCode::CameraCommonErr::InvalidState;
  }
}

ErrorCode::ErrorCodeType FileMgrImpl::startReqFileData(int fileIndex, std::string localPath, FileMgr::FileDataReqCBType cb, void* userData) {
  if (fileDataHandler->downloadState == DOWNLOAD_IDLE) {
    fileDataHandler->downloadState = RECVING_FILE_DATA;

    fileDataHandler->downloadPath = localPath;
    fileDataHandler->mmap_file_buffer_->currentLogFilePath = localPath;
    DSTATUS("currentLogFilePath = %s", localPath.c_str());

    fileDataHandler->reqCB = cb;
    fileDataHandler->reqCBUserData = userData;
    fileDataHandler->curTargetFileIndex = fileIndex;

    if (fileDataHandler->range_handler_)delete (fileDataHandler->range_handler_);
    fileDataHandler->range_handler_ = new CommonDataRangeHandler();
    if (!fileDataHandler->range_handler_) return ErrorCode::SysCommonErr::AllocMemoryFailed;

    /*! Create file data req task*/
    OsdkOsal_TaskCreate(&reqFileDataHandle,
                        (void *(*)(void *)) (&fileDataMonitorTask),
                        OSDK_TASK_STACK_SIZE_DEFAULT, this);

    return SendReqFileDataPack(fileIndex);
  } else {
    DERROR("Current state cannot support to do downloading ...");
    return ErrorCode::CameraCommonErr::InvalidState;
  }
}

/**
* 催促包只有3次，代表远程（eg.相机）已经发送完毕
* 如果共计10个包出现  1 2 3 4 gap 6 7 empty 的情况
* 直接从4重发
*/
void FileMgrImpl::OnReceiveUrgePack(dji_general_transfer_msg_ack *rsp) {
  DSTATUS("[FileTransferHandler] OnReceiveUrgePack");
  if (rsp) {
    CommonDataRangeHandler *range_handler_;
    if (rsp->task_id == DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST)
      range_handler_ = fileListHandler->range_handler_;
    else if (rsp->task_id == DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE)
      range_handler_ = fileDataHandler->range_handler_;
    else return;

    if (range_handler_->GetNoAckRanges().size() == 0)
      SendAbortPack((DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE) rsp->task_id);
    else
      DSTATUS("range_handler_->GetNoAckRanges().size() = %d", range_handler_->GetNoAckRanges().size());
      SendMissedAckPack((DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE) rsp->task_id);
  }
}

void FileMgrImpl::OnReceiveAbortPack(dji_general_transfer_msg_ack *rsp) {
  uint32_t curMs = 0;
  OsdkOsal_GetTimeMs(&curMs);
  DSTATUS("[%d][FileMgr] The request is aborted by the camera: session_id = %d",curMs,
          (int) rsp->session_id);
}

typedef enum ParsingStateEnum {
  PARSING_TOTAL_HEADER,
  PARSING_DATA_HEADER,
  PARSING_FILEINFO,
  PARSING_FILEDATA = PARSING_FILEINFO,
  PARSE_FINISH,
} ParsingFileListStateEnum, ParsingFileDataStateEnum;

#include "dji_file_mgr_internal_define.hpp"
FileMgrImpl::ConsumeDataBuffer FileMgrImpl::ConsumeChunk(DataPointer data_pointer, size_t &chunk_index, size_t consumSize) {
  ConsumeDataBuffer ret = {{0}, 0};
  if (consumSize > sizeof(ret.data))
    consumSize = sizeof(ret.data);
  if (consumSize > data_pointer.length - chunk_index)
    consumSize = data_pointer.length - chunk_index;
  memcpy(ret.data, (uint8_t *)data_pointer.data + chunk_index, consumSize);
  ret.index = consumSize;
  chunk_index += consumSize;
  return ret;
}

FilePackage FileMgrImpl::parseFileList(std::list<DataPointer> fullDataList) {
  static size_t chunk_index = 0;
  FilePackage pack;
  pack.type = FileType::UNKNOWN;
  //pack.common.clear();
  pack.media.clear();
  if (fullDataList.size() == 0) return pack;

  ParsingFileListStateEnum parsingState = PARSING_TOTAL_HEADER;
  while (fullDataList.size() != 0) {
    auto dataPtr = fullDataList.front();
    switch (parsingState) {
      case PARSING_TOTAL_HEADER: {
        chunk_index = 0;
        uint32_t consumeBytes = sizeof(dji_general_transfer_msg_ack) - sizeof(uint8_t);
        auto buffer = ConsumeChunk(dataPtr, chunk_index, consumeBytes);
        if (buffer.index == consumeBytes) {
          auto data = (dji_general_transfer_msg_ack *)(buffer.data);
          DSTATUS("Unpack datapack seq(%d)", data->seq);
          parsingState = (data->seq == 0) ? PARSING_DATA_HEADER : PARSING_FILEINFO;
        } else {
          parsingState = PARSE_FINISH;
        }
        break;
      }
      case PARSING_DATA_HEADER: {
        uint32_t consumeBytes = sizeof(dji_file_list_download_resp) - sizeof(dji_list_info_descriptor);
        auto buffer = ConsumeChunk(dataPtr, chunk_index, consumeBytes);
        if (buffer.index == consumeBytes) {
          auto data = (dji_file_list_download_resp *)(buffer.data);
          DSTATUS("###data->amount = %d, data->len = %d", data->amount, data->len);
          parsingState = PARSING_FILEINFO;
        } else {
          parsingState = PARSE_FINISH;
        }
        break;
      }
      case PARSING_FILEINFO:{
        uint32_t consumeBytes = sizeof(dji_list_info_descriptor) - sizeof(dji_file_list_ext_info);
        auto buffer = ConsumeChunk(dataPtr, chunk_index, consumeBytes);
        if (buffer.index == consumeBytes) {
          if (pack.type == FileType::UNKNOWN)pack.type = FileType::MEDIA;
          auto data = (dji_list_info_descriptor *)(buffer.data);
          //DSTATUS("data->index = %d, data->size = %d", data->index, data->size);
          /*! 构建file信息,装入容器 */
          MediaFile file;
          file.valid = true;
          file.date.year = data->create_time.year;
          file.date.month = data->create_time.month;
          file.date.day = data->create_time.day;
          file.date.hour = data->create_time.hour;
          file.date.minute = data->create_time.minute;
          file.date.second = data->create_time.second;
          file.fileIndex = data->index;
          file.fileSize = data->size;
          file.fileType = (MediaFileType)data->type;
          if ((data->type == (uint8_t) MediaFileType::MOV)
              || (data->type == (uint8_t) MediaFileType::MP4)) {
            file.duration = data->attribute.video_attribute.attribute_video_duration;
            file.orientation = (CameraOrientation)data->attribute.video_attribute.attribute_video_rotation;
            file.resolution = (VideoResolution)data->attribute.video_attribute.attribute_video_resolution;
            file.frameRate = (VideoFrameRate)data->attribute.video_attribute.attribute_video_framerate;
          } else if ((data->type == (uint8_t) MediaFileType::JPEG)
              || (data->type == (uint8_t) MediaFileType::DNG)
              || (data->type == (uint8_t) MediaFileType::TIFF)) {
            file.orientation = (CameraOrientation)data->attribute.photo_attribute.attribute_photo_rotation;
            file.photoRatio = (PhotoRatio)data->attribute.photo_attribute.attribute_photo_ratio;
          }
          pack.media.push_back(file);
          /*! 这部分消耗了就算了,目前不解析 */
          if (data->ext_size) {
            auto extBuffer = ConsumeChunk(dataPtr, chunk_index, data->ext_size);
            auto extData = (dji_ext_info_descriptor *) (extBuffer.data);
            DSTATUS("extData->id = %d, %s", extData->id, extData->data);
          }
          parsingState = PARSING_FILEINFO;
        } else {
          parsingState = PARSE_FINISH;
        }
        break;
      }
      case PARSE_FINISH:
        fullDataList.pop_front();
        parsingState = PARSING_TOTAL_HEADER;
        break;
      default:break;
    }
  }
  return pack;
}

#define SIZE_LIMIT 0
bool FileMgrImpl::parseFileData(dji_general_transfer_msg_ack *rsp) {
  if (rsp->seq == 0) {
    /*! 1. 是第一包,parse文件大小 */
    auto resp = (dji_file_data_download_resp *) (rsp->data);
    /*! 2. 文件总大小计算 */
    uint32_t file_size = resp->size - (sizeof(dji_file_data_download_resp) - sizeof(uint8_t));
    fileDataHandler->mmap_file_buffer_->init(fileDataHandler->downloadPath, file_size);
    /*! 3. 本包数据总大小计算 */
    uint32_t data_size = rsp->msg_length;
    data_size -= sizeof(dji_general_transfer_msg_ack) - sizeof(uint8_t);
    data_size -= sizeof(dji_file_data_download_resp) - sizeof(uint8_t);
    fileDataHandler->mmap_file_buffer_->InsertBlock(resp->file_data, data_size, rsp->seq);
  } else {
    /*! 1. 本包数据总大小计算 */
    uint32_t data_size = rsp->msg_length;
    data_size -= sizeof(dji_general_transfer_msg_ack) - sizeof(uint8_t);
    fileDataHandler->mmap_file_buffer_->InsertBlock(rsp->data, data_size, rsp->seq);
  }

#if 0
    size_t chunk_index = 0;
  ParsingFileListStateEnum parsingState = PARSING_TOTAL_HEADER;
  DataPointer dataPtr = {
      .data = rsp,
      .length = rsp->msg_length
  };

  while (chunk_index < dataPtr.length) {
    switch (parsingState) {
      case PARSING_TOTAL_HEADER: {
        chunk_index = 0;
        uint32_t consumeBytes = sizeof(dji_general_transfer_msg_ack) - sizeof(uint8_t);
        auto buffer = ConsumeChunk(dataPtr, chunk_index, consumeBytes);
        if (buffer.index == consumeBytes) {
          auto data = (dji_general_transfer_msg_ack *) (buffer.data);
          parsingState = (data->seq == 0) ? PARSING_DATA_HEADER : PARSING_FILEDATA;
        } else {
          parsingState = PARSE_FINISH;
        }
        break;
      }
      case PARSING_DATA_HEADER: {
        uint32_t consumeBytes = sizeof(dji_file_data_download_resp) - sizeof(uint8_t);
        auto buffer = ConsumeChunk(dataPtr, chunk_index, consumeBytes);
        if (buffer.index == consumeBytes) {
          auto data = (dji_file_data_download_resp *) (buffer.data);
          uint32_t file_size = data->size - (sizeof(dji_file_data_download_resp) - sizeof(uint8_t));
          this->mmap_file_buffer_->init(this->currentLogFilePath, file_size);
#if SIZE_LIMIT
          uint32_t targetFileSize = *(uint32_t *)(buffer.data + 4);
          uint32_t targetFileIndex = *(uint32_t *)(buffer.data + 8);
          DSTATUS("### The size of the current target file (index:%d) : %d", targetFileIndex, targetFileSize);
          if (targetFileSize > 2*1024*1024) {
            DERROR("### The size of the current target file (index:%d) : %d", targetFileIndex, targetFileSize);
            DERROR("### Current download function support only single file which is smaller than"
                   "2MB. It will be optimized in the future.");
            DERROR("### So sorry that this download request will go falied");
            this->closeFile();
            this->downloadState = DOWNLOAD_IDLE;
            return false;
          }
#endif
          parsingState = PARSING_FILEDATA;
        } else {
          parsingState = PARSE_FINISH;
        }
        break;
      }
      case PARSING_FILEDATA: {
        uint32_t consumeBytes = dataPtr.length - chunk_index; // 消耗完
        if (consumeBytes == 0) {
          //DSTATUS("##Parse 完了");
          parsingState = PARSE_FINISH;
          break;
        }
        auto buffer = ConsumeChunk(dataPtr, chunk_index, consumeBytes);
        if (buffer.index == consumeBytes) {
          //DSTATUS("##記錄文件數據, %dBytes", buffer.index);
          if (rsp->seq * 800 + buffer.index <= mmap_file_buffer_->fdAddrSize)
            mmap_file_buffer_->InsertBlock(buffer.data, buffer.index, rsp->seq);
          parsingState = PARSING_FILEDATA;
        } else {
          //DSTATUS("##Parse 完了");
          parsingState = PARSE_FINISH;
        }
        break;
      }
      case PARSE_FINISH:
        parsingState = PARSING_TOTAL_HEADER;
        return true;
        break;
      default:break;
    }
  }
#endif

  return false;
}

void FileMgrImpl::fileListRawDataCB(dji_general_transfer_msg_ack *rsp) {
  if (fileListHandler->downloadState == DOWNLOAD_IDLE) return;
    auto download_buffer_ = fileListHandler->download_buffer_;
    auto range_handler_ = fileListHandler->range_handler_;
    if (download_buffer_ && range_handler_) {
      download_buffer_->InsertBlock((const uint8_t *) rsp, rsp->msg_length, rsp->seq, true);
      range_handler_->AddSeqIndex(rsp->seq, download_buffer_->GetConfirmSeq(), download_buffer_->GetSize());
    }

    /*! refresh the time stamp */
    uint32_t curMs = 0;
    OsdkOsal_GetTimeMs(&curMs);
    fileListHandler->updateTimeMs = curMs;

    /*! 看看是否拿到了最后一个包 */
    /*! 收完了再解包*/
    if ((rsp->msg_flag & 0x01)
    && (range_handler_->GetLastNotReceiveSeq() == rsp->seq + 1)
    && (range_handler_->GetNoAckRanges().size() == 0)) {
      std::list<DataPointer> dataList = download_buffer_->DequeueAllBuffer();
      FilePackage file_package = parseFileList(dataList);

      SendAbortPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST);
      if (fileListHandler->reqCB) {
        fileListHandler->reqCB(OSDK_STAT_OK, file_package, fileListHandler->reqCBUserData);
        fileListHandler->reqCB = NULL;
      }

      DSTATUS("Finish req filelist task, reset downloadState to be DOWNLOAD_IDLE");
      fileListHandler->downloadState = DOWNLOAD_IDLE;
    }
}

void FileMgrImpl::fileDataRawDataCB(dji_general_transfer_msg_ack *rsp) {
  if (fileDataHandler->downloadState == DOWNLOAD_IDLE) return;
  auto range_handler_ = fileDataHandler->range_handler_;
  auto mmap_file_buffer_ = fileDataHandler->mmap_file_buffer_;
  if (range_handler_) {
    range_handler_->AddSeqIndex(rsp->seq, 0, (uint32_t)(-1));
  }

  /*! refresh the time stamp */
  uint32_t curMs = 0;
  OsdkOsal_GetTimeMs(&curMs);
  fileDataHandler->updateTimeMs = curMs;

  /*! do data parsing, 边收边解包 */
  parseFileData(rsp);

  /*! 看看是否拿到了最后一个包 */
  if ((rsp->msg_flag & 0x01)
      && (range_handler_->GetLastNotReceiveSeq() == rsp->seq + 1)
      && (range_handler_->GetNoAckRanges().size() == 0)){
    mmap_file_buffer_->deInit();
    SendAbortPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE);
    if (fileDataHandler->reqCB) {
      fileDataHandler->reqCB(OSDK_STAT_OK, fileDataHandler->reqCBUserData);
      fileDataHandler->reqCB = NULL;
    }
    DSTATUS("Finish req filedata task, reset downloadState to be DOWNLOAD_IDLE");
    fileDataHandler->downloadState = DOWNLOAD_IDLE;
  }
}

#define LOG_EVERY_PACK 0
void FileMgrImpl::OnReceiveDataPack(dji_general_transfer_msg_ack *rsp) {
  if (rsp->func_id != DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_DATA) return;

  static uint32_t lastSeq = 0;
  if (rsp->seq == 0) DSTATUS("[First pack] get the first pack");
  else if (rsp->seq != lastSeq + 1) DSTATUS("[Skip packs]------------------->skip seq : lastSeq = %d, rsp->seq = %d", lastSeq, rsp->seq);
  lastSeq = rsp->seq;

#if LOG_EVERY_PACK
  DSTATUS(
      "rsp : version(%d) headerlen(%d) taskid(%d) funcid(%d) msgflag(%d) msglen(%d) sessid(%d) seq(%d)",
      rsp->version, rsp->header_length, rsp->task_id, rsp->func_id,
      rsp->msg_flag, rsp->msg_length, rsp->session_id, rsp->seq);
#endif

  if (rsp->task_id == DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST) {
    fileListRawDataCB(rsp);
  } else if (rsp->task_id == DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE) {
    fileDataRawDataCB(rsp);
  }
}

void ParseFilelistData(dji_general_transfer_msg_ack *rsp) {

}

void FileMgrImpl::HandlePushPack(dji_general_transfer_msg_ack *rsp) {
  if (!rsp) {
    DERROR("Download ack param invalid");
    return;
  }
  DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE func_type = (DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE)rsp->func_id;
  switch (func_type) {
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_PUSH:
      OnReceiveUrgePack(rsp);
      break;
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ABORT:
      OnReceiveAbortPack(rsp);
      break;
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_DATA:
      OnReceiveDataPack(rsp);
      break;
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_REQ:
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ACK:
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_DEL:
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_PAUSE:
    case DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_RESUME:
      DERROR("Now support on this Function ID(%d)", func_type);
      break;
  }
}

ErrorCode::ErrorCodeType FileMgrImpl::SendAbortPack(
    DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE taskId) {
  DSTATUS("SendAbortPack");
  uint8_t reqBuf[1024] = {0};
  dji_general_transfer_msg_req
      *setting = (dji_general_transfer_msg_req *) reqBuf;
  setting->version = 1;
  setting->header_length = 10;
  setting->task_id = taskId;
  setting->func_id = DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ABORT;
  setting->msg_flag = 0;
  setting->session_id = 0;
  setting->seq = 0;
/*
  uint32_t abortReason = TransAbortReasonForce;
  uint32_t reqDataLen = sizeof(abortReason);
  memcpy(setting->data, &abortReason, reqDataLen);
  */
  setting->msg_length = sizeof(dji_general_transfer_msg_req) - 1;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];

  cmdInfo.cmdSet = V1ProtocolCMD::Common::downloadFile[0];
  cmdInfo.cmdId = V1ProtocolCMD::Common::downloadFile[1];
  cmdInfo.dataLen = setting->msg_length;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_NO_NEED;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(type, index);
  cmdInfo.sender = localSenderId; //linker->getLocalSenderId();
//  printf("-------------->request data :\n");
//  for (int i = 0; i < cmdInfo.dataLen; i++) {
//    printf("%02X ", ((uint8_t *) setting)[i]);
//  }
//  printf("\n");
  if (OSDK_STAT_OK != linker->send(&cmdInfo, (uint8_t *) setting))
  DERROR("Send error");
  return ErrorCode::SysCommonErr::Success;
}


ErrorCode::ErrorCodeType FileMgrImpl::SendACKPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE taskId, dji_download_ack *ack) {
  uint8_t reqBuf[1024] = {0};
  dji_general_transfer_msg_req
      *setting = (dji_general_transfer_msg_req *) reqBuf;
  setting->version = 1;
  setting->header_length = 10;
  setting->task_id = taskId;
  setting->func_id = DJI_GENERAL_DOWNLOAD_FILE_FUNC_TYPE_ACK;
  setting->msg_flag = 0;
  setting->session_id = 0;
  setting->seq = 0;

  uint32_t reqDataLen = sizeof(dji_download_ack) - sizeof(dji_loss_desc) + ack->loss_nr * sizeof(dji_loss_desc);
  memcpy(setting->data, ack, reqDataLen);
  setting->msg_length = sizeof(dji_general_transfer_msg_req) + reqDataLen - 1;

  T_CmdInfo cmdInfo = {0};
  T_CmdInfo ackInfo = {0};
  uint8_t ackData[1024];

  cmdInfo.cmdSet = V1ProtocolCMD::Common::downloadFile[0];
  cmdInfo.cmdId = V1ProtocolCMD::Common::downloadFile[1];
  cmdInfo.dataLen = setting->msg_length;
  cmdInfo.needAck = OSDK_COMMAND_NEED_ACK_NO_NEED;
  cmdInfo.packetType = OSDK_COMMAND_PACKET_TYPE_REQUEST;
  cmdInfo.addr = GEN_ADDR(0, ADDR_V1_COMMAND_INDEX);
  cmdInfo.receiver = OSDK_COMMAND_DEVICE_ID(type, index);
  cmdInfo.sender = localSenderId; //linker->getLocalSenderId();

//  printf("-------------->request data :\n");
//  for (int i = 0; i < cmdInfo.dataLen; i++) {
//    printf("%02X ", ((uint8_t *) setting)[i]);
//  }
//  printf("\n");

  if (OSDK_STAT_OK != linker->send(&cmdInfo, (uint8_t *) setting))
  DERROR("Send error");
  /*
  ErrorCode::ErrorCodeType ret = ErrorCode::getLinkerErrorCode(linkAck);
  if (ret != ErrorCode::SysCommonErr::Success) return ret;
  return ErrorCode::getErrorCode(ErrorCode::CameraModule,
                                 ErrorCode::CameraCommon, ackData[0]);*/
  return ErrorCode::SysCommonErr::Success;
}

ErrorCode::ErrorCodeType FileMgrImpl::SendMissedAckPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE taskId) {
  CommonDataRangeHandler *range_handler_;
  if (taskId == DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_LIST)
    range_handler_ = fileListHandler->range_handler_;
  else if (taskId == DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE_FILE)
    range_handler_ = fileDataHandler->range_handler_;
  else return ErrorCode::SysCommonErr::ReqNotSupported;

  std::vector<Range> ranges = range_handler_->GetNoAckRanges();
  if(ranges.empty()) {
    uint8_t buf[1024] = {0};
    dji_download_ack *ack = (dji_download_ack *)buf;
    ack->expect_seq = range_handler_->GetLastNotReceiveSeq();
    ack->loss_nr = 0;
    DSTATUS("[Confirming ...]---------------ack->expect_seq = %d ack->loss_nr = %d", ack->expect_seq, ack->loss_nr);
    SendACKPack(taskId, ack);
    return OSDK_STAT_OK;
  }

  auto rangeCnt = ranges.size() > 1 ? 1 : ranges.size();
  uint8_t buf[1024] = {0};
  dji_download_ack *ack = (dji_download_ack *)buf;
  ack->expect_seq = ranges[0].seq_num;
  ack->loss_nr = rangeCnt;
  DSTATUS("[ReqMissingPack ...]---------------ack->expect_seq = %d ack->loss_nr = %d", ack->expect_seq, ack->loss_nr);
  for (int i = 0; i < rangeCnt; i++) {
    auto range = ranges[i];
    ack->loss_desc[i].seq = range.seq_num;
    ack->loss_desc[i].cnt = range.length;
    DSTATUS("[MissPack]---------------loss[%d] range.seq_num = %d, range.length = %d", i, range.seq_num, range.length);
  }
  return SendACKPack(taskId, ack);
}

DownloadListHandler::DownloadListHandler() : reqCB(nullptr), reqCBUserData(nullptr) {
  range_handler_ = new CommonDataRangeHandler();
  download_buffer_ = new DownloadBufferQueue();
  downloadState = DOWNLOAD_IDLE;
}

DownloadListHandler::~DownloadListHandler() {
  if (range_handler_) delete range_handler_;
  if (download_buffer_) delete download_buffer_;
}

DownloadDataHandler::DownloadDataHandler() : reqCB(nullptr), reqCBUserData(nullptr) {
  range_handler_ = new CommonDataRangeHandler();
  mmap_file_buffer_ = new MmapFileBuffer();
  downloadState = DOWNLOAD_IDLE;
}

DownloadDataHandler::~DownloadDataHandler() {
  if (range_handler_) delete range_handler_;
  if (mmap_file_buffer_) delete mmap_file_buffer_;
}