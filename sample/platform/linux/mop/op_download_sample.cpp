/*! @file op_download_sample.cpp
 *  @version 4.0
 *  @date March 6 2020
 *
 *  @brief Sample to show how to download file from PSDK to OSDK
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
#define TEST_RECV_FILE_NAME "test.mp4"

#define ASSERT_MOP_RET(ret) \
  if (ret != MOP_PASSED) { \
  ErrorCode::printErrorCodeMsg(ret); \
  throw std::runtime_error("MOP result failed"); \
}

typedef enum OPDownloadSampleState {
  DOWNLOAD_TASK_IDLE,
  REQUEST_FILE_DOWNLOAD,
  RECV_FILE_DOWNLOAD_ACK,
  SEND_FILE_NAME,
  RECV_FILE_INFO_DATA,
  RECV_FILE_RAW_DATA,
  MD5_RESULT_CHECK,
  DOWNLOAD_TASK_FINISH,
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

int writeStreamData(const char *fileName, const uint8_t *data, uint32_t len) {
  FILE *fp = NULL;
  size_t size = 0;

  fp = fopen(fileName, "a+");
  if(fp == NULL) {
    DERROR("fopen failed!\n");
    return -1;
  }
  size = fwrite(data, 1, len, fp);
  if(size != len) {
    return -1;
  }

  fflush(fp);
  if(fp) {
    fclose(fp);
  }
  return 0;
}

#include <openssl/md5.h>
static void OPDownloadFileTask(MopPipeline *OP_Pipeline) {
  if (!OP_Pipeline)std::runtime_error("Error param");
  fileInfo downloadFileInfo = {0};
  MopErrCode mopRet;
  OPUploadSampleState uploadState = REQUEST_FILE_DOWNLOAD;
  uint8_t recvBuf[READ_ONCE_BUFFER_SIZE] = {0};

  /*! MD5 prepare*/
  MD5_CTX ctx;
  unsigned char md5_out[16];

  while(uploadState != DOWNLOAD_TASK_FINISH) {
    switch (uploadState) {
      case REQUEST_FILE_DOWNLOAD: {
        /*! Step 1 : send file download request */
        DSTATUS("Step 1 : Send file download request ...");
        MD5_Init(&ctx);
        sampleProtocolStruct req = {.cmd = CMD_REQUEST, .subcmd = REQ_DOWNLOAD};
        MopPipeline::DataPackType reqPack = {.data = (uint8_t *) &req, .length = SIMPLE_CMD_PACK_LEN};
        mopRet = OP_Pipeline->sendData(reqPack, &reqPack.length);
        ASSERT_MOP_RET(mopRet)
        uploadState = RECV_FILE_DOWNLOAD_ACK;
        break;
      }
      case RECV_FILE_DOWNLOAD_ACK: {
        /*! Step 2 : receive the ack of the file download request */
        MopPipeline::DataPackType readPack = {(uint8_t *) recvBuf, READ_ONCE_BUFFER_SIZE};
        mopRet = OP_Pipeline->recvData(readPack, &readPack.length);
        ASSERT_MOP_RET(mopRet)
        sampleProtocolStruct *ackData = (sampleProtocolStruct *)readPack.data;
        if ((readPack.length >= SIMPLE_CMD_PACK_LEN) && (ackData->cmd == CMD_ACK) && (ackData->subcmd == ACK_OK)) {
          DSTATUS("Step 2 : Request to download file OK");
        } else {
          throw std::runtime_error("Step 2 : Request to download file failed!");
        }
        uploadState = SEND_FILE_NAME;
        break;
      }
      case SEND_FILE_NAME: {
        /*! Step 3 : send target download file name */
        DSTATUS("Step 3 : Send target download  file name ...");
        sampleProtocolStruct req = {
            .cmd = CMD_DL_FILENAME,
            .subcmd = 0xFF,
            .seq = 0,
            .dataLen = sizeof(req.data.targetFile)
        };
        req.data.info.isExist = true;
        sprintf(req.data.targetFile.fileName, TEST_RECV_FILE_NAME);

        MopPipeline::DataPackType fileInfoPack = {.data = (uint8_t *) &req, .length = 6 + req.dataLen};
        mopRet = OP_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
        ASSERT_MOP_RET(mopRet)
        uploadState = RECV_FILE_INFO_DATA;
        break;
      }
      case RECV_FILE_INFO_DATA: {
        /*! Step 4 : receive the info of the target download file */
        MopPipeline::DataPackType readPack = {(uint8_t *) recvBuf, READ_ONCE_BUFFER_SIZE};
        mopRet = OP_Pipeline->recvData(readPack, &readPack.length);
        ASSERT_MOP_RET(mopRet)
        DSTATUS("Step 4 : recv data length : %d", readPack.length);
        sampleProtocolStruct *ackData = (sampleProtocolStruct *)readPack.data;
        if ((readPack.length >= FILEINFO_PACK_LEN) && (ackData->cmd == CMD_FILEINFO)) {
          DSTATUS("Step 4 : File info get success");
        } else {
          throw std::runtime_error("Step 4 : File info get failed!");
        }
        sampleProtocolStruct *data = (sampleProtocolStruct *)readPack.data;
        downloadFileInfo = data->data.info;
        DSTATUS("Step 4 : get file name : %s", downloadFileInfo.fileName);
        DSTATUS("Step 4 : get file size : %d", downloadFileInfo.fileLength);
        DSTATUS("Step 4 : get file exit : %s", downloadFileInfo.isExist ? "Yes" : "No");
        char md5Char[100] = {0};
        for (int i = 0; i < sizeof(downloadFileInfo.md5Buf); i++)
          sprintf(md5Char, "%s%02X", md5Char, downloadFileInfo.md5Buf[i]);
        DSTATUS("Step 4 : get file md5  : %s", md5Char);
        uploadState = RECV_FILE_RAW_DATA;
        break;
      }
      case RECV_FILE_RAW_DATA: {
        /*! Step 5 : recv file raw data */
        DSTATUS("Step 5 : Recv file raw data.");
        FILE *fp = fopen(TEST_RECV_FILE_NAME, "w+");
        uint32_t cnt = 0;
        bool recvEndPack = false;
        if (fp == NULL) throw std::runtime_error("open file error");

        while (1) {
          memset(recvBuf, 0, READ_ONCE_BUFFER_SIZE);
          MopPipeline::DataPackType readPacket = {(uint8_t *)recvBuf, READ_ONCE_BUFFER_SIZE};
          mopRet = OP_Pipeline->recvData(readPacket, &readPacket.length);
          if (mopRet != MOP_PASSED) {
            DERROR("recv whole file data failed!, realLen = %d\n", readPacket.length);
            //break;
          } else {
            DSTATUS("recv whole file data success!, realLen = %d\n", readPacket.length);
            sampleProtocolStruct *fileData = (sampleProtocolStruct *)readPacket.data;
            DSTATUS("fileData->dataLen = %d", fileData->dataLen);
            fwrite(fileData->data.fileData, 1, fileData->dataLen, fp);
            MD5_Update(&ctx, fileData->data.fileData, fileData->dataLen);
            cnt++;
            DSTATUS("recv cnt %d!\n", cnt);
            if (fileData->subcmd == END_PACK) break;
          }
        }
        MD5_Final(md5_out, &ctx);
        DSTATUS("Step 5 : Recv file raw data finish.");
        uploadState = MD5_RESULT_CHECK;
        break;
      }
      case MD5_RESULT_CHECK: {
        DSTATUS("Step 6 : Check file md5 value.");
        sampleProtocolStruct req = {
            .cmd = CMD_RESULT,
            .subcmd = RET_OK,
        };
        char md5Char[100] = {0};
        for (int i = 0; i < sizeof(downloadFileInfo.md5Buf); i++)
          sprintf(md5Char, "%s%02X", md5Char, downloadFileInfo.md5Buf[i]);
        DSTATUS("Step 6 : get file md5  : %s", md5Char);
        memset(md5Char, 0 ,sizeof(md5Char));
        for (int i = 0; i < sizeof(md5_out); i++)
          sprintf(md5Char, "%s%02X", md5Char, md5_out[i]);
        DSTATUS("Step 6 : cal file md5  : %s", md5Char);
        if (memcmp(md5_out, downloadFileInfo.md5Buf, sizeof(md5_out)) == 0) {
          DSTATUS("Step 6 : MD5 compare success.");
          req.subcmd = RET_OK;
        } else {
          DERROR("Step 6 : Error MD5");
          req.subcmd = RET_FAIL;
        }

        MopPipeline::DataPackType fileInfoPack = {.data = (uint8_t *) &req, .length = SIMPLE_CMD_PACK_LEN};
        mopRet = OP_Pipeline->sendData(fileInfoPack, &fileInfoPack.length);
        ASSERT_MOP_RET(mopRet)
        uploadState = DOWNLOAD_TASK_FINISH;
        break;
      }
    }
  }
}

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
  OPDownloadFileTask(OP_Pipeline);

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
    DERROR("Client task create failed!\n");
  } else {
    DSTATUS("Client task create success!\n");
  }

  //press q to exit
  while(!sampleFinish) {
    OsdkOsal_TaskSleepMs(1000);
  }
  DSTATUS("Sample exit.");
}
