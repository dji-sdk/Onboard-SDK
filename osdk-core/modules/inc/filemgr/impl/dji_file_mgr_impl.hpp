/** @file dji_file_mgr_impl.hpp
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

#ifndef DJI_FILE_MGR_IMPL_HPP
#define DJI_FILE_MGR_IMPL_HPP

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory>
#include <atomic>
#include "dji_error.hpp"
#include "osdk_command.h"
#include "dji_file_mgr_internal_define.hpp"
#include "commondatarangehandler.h"
#include "downloadbufferqueue.h"
#include "dji_file_mgr_define.hpp"
#include "dji_file_mgr.hpp"
#include "mmap_file_buffer.hpp"

#if 0
#include "commondatarangehandler.h"
#include "downloadbufferqueue.h"
#endif

namespace DJI {
namespace OSDK {

typedef enum DownloadStateEnum : int {
  DOWNLOAD_IDLE,
  RECVING_FILE_LIST,
  RECVING_FILE_DATA,
  MAX_INDEX_CNT,
} DownloadStateEnum;

// Forward Declaration
class Linker;

class DownloadListHandler {
 public:
  DownloadListHandler();
  ~DownloadListHandler();
 public:
  CommonDataRangeHandler *range_handler_;
  DownloadBufferQueue *download_buffer_;
  FileMgr::FileListReqCBType reqCB;
  void* reqCBUserData;
  std::atomic<int> downloadState;
  std::atomic<uint32_t> updateTimeMs;
};

class DownloadDataHandler {
 public:
  DownloadDataHandler();
  ~DownloadDataHandler();
 public:
  CommonDataRangeHandler *range_handler_;
  MmapFileBuffer *mmap_file_buffer_;
  FileMgr::FileDataReqCBType reqCB;
  void* reqCBUserData;
  std::atomic<uint32_t> updateTimeMs;
  std::string downloadPath;
  std::atomic<int> downloadState;
  std::atomic<int> curTargetFileIndex;
};

class FileMgrImpl {
 public:
  FileMgrImpl(Linker *linker, E_OSDKCommandDeiveType type, uint8_t index);
  ~FileMgrImpl();

  ErrorCode::ErrorCodeType startReqFileList(FileMgr::FileListReqCBType cb, void* userData);
  ErrorCode::ErrorCodeType startReqFileData(int fileIndex, std::string localPath, FileMgr::FileDataReqCBType cb, void* userData);

  void HandlePushPack(dji_general_transfer_msg_ack *rsp);
  ErrorCode::ErrorCodeType SendReqFileListPack();
  ErrorCode::ErrorCodeType SendReqFileDataPack(int fileIndex) ;

 private:
  ErrorCode::ErrorCodeType SendAbortPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE taskId);
  ErrorCode::ErrorCodeType SendACKPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE taskId, dji_download_ack *ack);
  ErrorCode::ErrorCodeType SendMissedAckPack(DJI_GENERAL_DOWNLOAD_FILE_TASK_TYPE taskId);

 private:
  DownloadListHandler *fileListHandler;
  DownloadDataHandler *fileDataHandler;

  Linker *linker;
  E_OSDKCommandDeiveType type;
  uint8_t index;

 private:
  //typedef void (*FileDataReqCBType)(E_OsdkStat ret_code, dji_general_transfer_msg_ack* ackData);
  //static void internalFileDataReqCB(E_OsdkStat ret_code, void *userData);
  typedef struct ConsumeDataBuffer {
    uint8_t data[1024];
    uint16_t index;
  } ConsumeDataBuffer;
  ConsumeDataBuffer ConsumeChunk(DataPointer data_pointer, size_t &chunk_index, size_t consumSize);
  FilePackage parseFileList(std::list<DataPointer> fullDataList);
  bool parseFileData(dji_general_transfer_msg_ack *rsp);

 private:
  void OnReceiveAbortPack(dji_general_transfer_msg_ack *rsp);
  void OnReceiveUrgePack(dji_general_transfer_msg_ack *rsp);
  void OnReceiveDataPack(dji_general_transfer_msg_ack *rsp);

  void fileListRawDataCB(dji_general_transfer_msg_ack *rsp);
  void fileDataRawDataCB(dji_general_transfer_msg_ack *rsp);

  uint16_t createNextReqSessionId() {return reqSessionId++;};
  uint16_t getCurReqSessionId() {return reqSessionId;};
  static std::atomic<uint16_t> reqSessionId;
  T_OsdkTaskHandle reqFileListHandle;
  T_OsdkTaskHandle reqFileDataHandle;
  static void fileListMonitorTask(void *arg);
  static void fileDataMonitorTask(void *arg);
  void printFileDownloadStatus();
  //只是用于测试
 private:
  uint8_t localSenderId;
  FILE *fp = NULL;
  int fd;
  void *fdAddr;
  uint64_t fdAddrSize;
  uint64_t fdAddrIndex;
  uint64_t tempSize = 0;

};

}
}

#endif  // DJI_FILE_MGR_IMPL_HPP
