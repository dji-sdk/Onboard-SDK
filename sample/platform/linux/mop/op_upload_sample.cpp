/*! @file op_upload_sample.cpp
 *  @version 4.0
 *  @date March 6 2020
 *
 *  @brief Sample to show how to upload file from OSDK to PSDK
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

#include "dji_linux_helpers.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <dirent.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "mop_sample_simple_protocol.hpp"

using namespace DJI::OSDK;

#define TEST_OP_PIPELINE_ID 49153
#define READ_ONCE_BUFFER_SIZE 100*1024
#define SEND_ONCE_BUFFER_SIZE 100*1024
#define TEST_SEND_FILE_NAME "/home/dji/telemetryLogFile.txt"
#define EXPORT_SHORT_FILE_NAME "telemetryLogFile.txt"
#define TEST_FILE_MD5_ARRAY {0xa1, 0xd0, 0xc4, 0x50, 0xe7, 0x49, 0xd0, 0x99, \
                             0xb6, 0x95, 0x69, 0x0a, 0x46, 0xa5, 0x76, 0xbb}

#define ASSERT_MOP_RET(ret) \
  if (ret != MOP_PASSED) { \
  ErrorCode::printErrorCodeMsg(ret); \
  throw std::runtime_error("MOP result failed"); \
}

typedef enum OPUploadSampleState {
  UPLOAD_TASK_IDLE,
  REQUEST_FILE_UPLOAD,
  RECV_FILE_UPLOAD_ACK,
  SEND_FILE_INFOMATION,
  RECV_FILE_INFO_ACK,
  SEND_FILE_RAW_DATA,
  RECV_FINAL_RESULT,
  UPLOAD_TASK_FINISH,
} OPUploadSampleState;
bool sampleFinish = false;

static uint32_t get_file_size(const char *path)
{
  unsigned long filesize = -1;
  struct stat statbuff;
  if(stat(path, &statbuff) < 0){
    return filesize;
  }else{
    filesize = statbuff.st_size;
  }
  return filesize;
}

static void OPUploadFileTask(MopPipeline *OP_Pipeline) {
  if (!OP_Pipeline)std::runtime_error("Error param");
  MopErrCode mopRet;
  OPUploadSampleState uploadState = REQUEST_FILE_UPLOAD;
  uint8_t recvBuf[READ_ONCE_BUFFER_SIZE] = {0};
  while(uploadState != UPLOAD_TASK_FINISH) {
    switch (uploadState) {
      case REQUEST_FILE_UPLOAD: {
        /*! Step 1 : send file upload request */
        DSTATUS("Step 1 : Send file upload request ...");
        sampleProtocolStruct req = {.cmd = CMD_REQUEST, .subcmd = REQ_UPLOAD};
        MopPipeline::DataPackType reqPack = {.data = (uint8_t *) &req, .length = SIMPLE_CMD_PACK_LEN};
        mopRet = OP_Pipeline->sendData(reqPack, &reqPack.length);
        ASSERT_MOP_RET(mopRet)
        uploadState = RECV_FILE_UPLOAD_ACK;
        break;
      }
      case RECV_FILE_UPLOAD_ACK: {
        /*! Step 2 : receive the ack of the file upload request */
        MopPipeline::DataPackType readPack = {(uint8_t *) recvBuf, READ_ONCE_BUFFER_SIZE};
        mopRet = OP_Pipeline->recvData(readPack, &readPack.length);
        ASSERT_MOP_RET(mopRet)
        sampleProtocolStruct *ackData = (sampleProtocolStruct *) readPack.data;
        if ((readPack.length >= SIMPLE_CMD_PACK_LEN) && (ackData->cmd == CMD_ACK) && (ackData->subcmd == ACK_OK)) {
          DSTATUS("Step 2 : Request to upload file OK");
        } else {
          throw std::runtime_error("Step 2 : Request to upload file failed!");
        }
        uploadState = SEND_FILE_INFOMATION;
        break;
      }
      case SEND_FILE_INFOMATION: {
        /*! Step 3 : send file information */
        DSTATUS("Step 3 : Send file information ...");
        sampleProtocolStruct req = {
            .cmd = CMD_FILEINFO,
            .subcmd = 0xFF,
            .seq = 0,
            .dataLen = sizeof(req.data.info)
        };
        req.data.info.isExist = true;
        req.data.info.fileLength = get_file_size(TEST_SEND_FILE_NAME);
        sprintf(req.data.info.fileName, EXPORT_SHORT_FILE_NAME);
        uint8_t fileMd5[] = TEST_FILE_MD5_ARRAY;
        memcpy(req.data.info.md5Buf, fileMd5, sizeof(req.data.info.md5Buf));

        MopPipeline::DataPackType fileInfoPack = {.data = (uint8_t *) &req, .length = RAW_DATA_HEADER_LEN + req.dataLen};
        mopRet = OP_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
        ASSERT_MOP_RET(mopRet)
        uploadState = RECV_FILE_INFO_ACK;
        break;
      }
      case RECV_FILE_INFO_ACK: {
        /*! Step 4 : receive the ack of the file info pack pushing */
        MopPipeline::DataPackType readPack = {(uint8_t *) recvBuf, READ_ONCE_BUFFER_SIZE};
        mopRet = OP_Pipeline->recvData(readPack, &readPack.length);
        ASSERT_MOP_RET(mopRet)
        sampleProtocolStruct *ackData = (sampleProtocolStruct *) readPack.data;
        if ((readPack.length >= SIMPLE_CMD_PACK_LEN) && (ackData->cmd == CMD_ACK) && (ackData->subcmd == ACK_OK)) {
          DSTATUS("Step 4 : File info update OK");
        } else {
          throw std::runtime_error("Step 4 : File info update failed!");
        }
        uploadState = SEND_FILE_RAW_DATA;
        break;
      }
      case SEND_FILE_RAW_DATA: {
        /*! Step 5 : send file raw data */
        DSTATUS("Step 5 : Send file raw data.");
        sampleProtocolStruct *filePushingData = (sampleProtocolStruct *) recvBuf;
        FILE *fp = fopen(TEST_SEND_FILE_NAME, "r");
        uint32_t cnt = 0;
        auto targetFileSize = get_file_size(TEST_SEND_FILE_NAME);
        if (fp == NULL) throw std::runtime_error("open file error");
        int totalSize = 0;
        do {
          int ret = fread((uint8_t *) filePushingData->data.fileData, 1, SEND_ONCE_BUFFER_SIZE - RAW_DATA_HEADER_LEN, fp);
          totalSize += ret;
          DSTATUS("total read size : %d; pdf read bytes ret = %d", totalSize, ret);
          filePushingData->cmd = CMD_FILEDATA;
          filePushingData->subcmd = (totalSize >= targetFileSize) ? END_PACK : NORMAL_PACK;
          filePushingData->dataLen = ret;
          MopPipeline::DataPackType fileDataPacket = {(uint8_t *) filePushingData, RAW_DATA_HEADER_LEN + filePushingData->dataLen};

          DSTATUS("Do sendData to PSDK, size : %d", fileDataPacket.length);
          mopRet = OP_Pipeline->sendData(fileDataPacket, &fileDataPacket.length);
          if (mopRet != MOP_PASSED) {
            DERROR("mop send error,stat:%lld, writePacket.length = %d", mopRet,
                   fileDataPacket.length);
            break;
          } else {
            DSTATUS("mop send success,stat:%lld, writePacket.length = %d", mopRet,
                   fileDataPacket.length);
          }
          cnt++;
          DSTATUS("send cnt %d!", cnt);
        } while (totalSize < targetFileSize);
        uploadState = RECV_FINAL_RESULT;
        break;
      }
      case RECV_FINAL_RESULT: {
        DSTATUS("Step 6 : Wait for file upload result .");

        /*! Step 6 : get result */
        MopPipeline::DataPackType readPack = {(uint8_t *) recvBuf, READ_ONCE_BUFFER_SIZE};
        mopRet = OP_Pipeline->recvData(readPack, &readPack.length);
        ASSERT_MOP_RET(mopRet)
        sampleProtocolStruct *ackData = (sampleProtocolStruct *)readPack.data;
        if ((readPack.length >= SIMPLE_CMD_PACK_LEN) && (ackData->cmd == CMD_RESULT) && (ackData->subcmd == RET_OK)) {
          DSTATUS("Step 6 : File upload Successfully");
        } else {
          throw std::runtime_error("Step 6 : File upload result falied.");
        }
        uploadState = UPLOAD_TASK_FINISH;
        break;
      }
    }
  }

};

static void* MopClientTask(void *arg)
{
  Vehicle *vehicle = (Vehicle *)arg;

  /*! main psdk device init */
  ErrorCode::ErrorCodeType ret = vehicle->psdkManager->initPSDKModule(
      PAYLOAD_INDEX_0, "Main_psdk_device");
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Init PSDK module Main_psdk_device failed.");
    ErrorCode::printErrorCodeMsg(ret);
  }

  /*! get the mop client */
  MopClient *mopClient = NULL;
  ret = vehicle->psdkManager->getMopClient(PAYLOAD_INDEX_0, mopClient);
  if (ret != ErrorCode::SysCommonErr::Success) {
    DERROR("Get MOP client object for_psdk_device failed.");
    ErrorCode::printErrorCodeMsg(ret);
    return NULL;
  }

  if (!mopClient) {
    DERROR("Get MOP client object is a null value.");
    return NULL;
  }

  /*! connect pipeline */
  MopPipeline *OP_Pipeline = NULL;
  if ((mopClient->connect(TEST_OP_PIPELINE_ID, RELIABLE, OP_Pipeline)
      != MOP_PASSED) || (OP_Pipeline == NULL)) {
    DERROR("MOP Pipeline connect failed");
    return NULL;
  } else {
    DSTATUS("Connect to mop pipeline id(%d) successfully", TEST_OP_PIPELINE_ID);
  }

  /*! OSDK upload file to PSDK */
  OPUploadFileTask(OP_Pipeline);

  /*! Disconnect pipeline */
  if (mopClient->disconnect(TEST_OP_PIPELINE_ID) != MOP_PASSED) {
    DERROR("MOP Pipeline disconnect pipeline(%d) failed", TEST_OP_PIPELINE_ID);
  } else {
    DSTATUS("Disconnect mop pipeline id(%d) successfully", TEST_OP_PIPELINE_ID);
  }
  sampleFinish = true;
}

int main(int argc, char** argv)
{
  int result;
  pthread_t clientTask;
  pthread_t serverTask;

  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();

  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }

  result = pthread_create(&clientTask, NULL, MopClientTask, vehicle);
  if(result != 0) {
    DERROR("Client task create failed!");
  } else {
    DSTATUS("Client task create success!");
  }

  //press q to exit
  while(!sampleFinish) {
    OsdkOsal_TaskSleepMs(1000);
  }
  DSTATUS("Sample exit.");
}
