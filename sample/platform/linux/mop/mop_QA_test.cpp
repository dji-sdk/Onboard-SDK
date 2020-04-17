/*! @file mop_client_main.cpp
 *  @version 4.0
 *  @date March 6 2020
 *
 *  @brief Sample to show how to use mop as a client.
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


using namespace DJI::OSDK;

#define TEST_OP_PIPELINE_ID 15
#define READ_ONCE_BUFFER_SIZE 100*1024
#define SEND_ONCE_BUFFER_SIZE 100*1024
#define TEST_SEND_FILE_NAME "/home/dji/M210_Manual.pdf"


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

static int non_block_get_char()
{
  fd_set rfds;
  struct timeval tv;
  int ch = 0;

  FD_ZERO(&rfds);
  FD_SET(0, &rfds);
  tv.tv_sec = 0;
  tv.tv_usec = 10;

  if (select(1, &rfds, NULL, NULL, &tv) > 0)
  {
    ch = getchar();
    DSTATUS("getchar: %c \n", ch);
  }

  return ch;
}

int writeStreamData(const char *fileName, const uint8_t *data, uint32_t len) {
  FILE *fp = NULL;
  size_t size = 0;

  fp = fopen(fileName, "a+");
  if(fp == NULL) {
    printf("fopen failed!\n");
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

static bool send_finish = false;
static bool recv_finish = false;

using namespace DJI::OSDK;

static void* MopRecvTask(void *arg)
{
  MopPipeline *handle;
  MopErrCode mopRet;
  void *recvBuf = NULL;
  int cnt = 0;

  handle = (MopPipeline *)arg;
  if(handle == NULL) {
    printf("recv task param check failed!\n");
    recv_finish = true;
    return NULL;
  }

  recvBuf = malloc(READ_ONCE_BUFFER_SIZE);
  if (recvBuf == NULL) {
    printf("malloc recv buffer error\n");
    recv_finish = true;
    return NULL;
  }

  struct timeval tv;
  struct tm tm;
  gettimeofday(&tv, NULL);
  localtime_r(&tv.tv_sec, &tm);
  char logFileName[128] = {0};
  sprintf(logFileName, "mop_recv_file_%d-%d-%d_%d-%d-%d", tm.tm_year + 1900,
          tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

  while (1) {
    memset(recvBuf, 0, READ_ONCE_BUFFER_SIZE);
    MopPipeline::DataPackType readPacket = {(uint8_t *)recvBuf, READ_ONCE_BUFFER_SIZE};
    mopRet = handle->recvData(readPacket, &readPacket.length);
    if (mopRet != MOP_PASSED) {
      printf("recv whole file data failed!, realLen = %d\n", readPacket.length);
      //break;
    } else {
      printf("recv whole file data success!, realLen = %d\n", readPacket.length);
    }
    writeStreamData(logFileName, readPacket.data, readPacket.length);
    cnt++;
    printf("recv cnt %d!\n", cnt);
    if (non_block_get_char() == 'q') break;
  }
  printf("mop channel recv task end!\n");

  if(recvBuf != NULL)
    free(recvBuf);
  recv_finish = true;
  return NULL;
}

static void* MopSendTask(void *arg)
{
  MopPipeline *handle;
  MopErrCode mopRet;
  void* addr = NULL;
  int cnt = 0;

  handle = (MopPipeline *)arg;
  if(handle == NULL) {
    printf("send task param check failed!\n");
    send_finish = true;
    return NULL;
  }

  struct timeval tv;
  struct tm tm;
  gettimeofday(&tv, NULL);
  localtime_r(&tv.tv_sec, &tm);
  char logFileName[128] = {0};
  sprintf(logFileName, "mop_send_file_%d-%d-%d_%d-%d-%d", tm.tm_year + 1900,
          tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

  auto targetFileSize = get_file_size(TEST_SEND_FILE_NAME);
  FILE *fp = fopen(TEST_SEND_FILE_NAME, "r");
  if (fp == NULL) {
    printf("open %s error\n ", TEST_SEND_FILE_NAME);
    return NULL;
  }
  printf("targetFileSize = %d", targetFileSize);
  addr = malloc(SEND_ONCE_BUFFER_SIZE);

  int totalSize = 0;
  do {
    int ret = fread((uint8_t *) addr, 1, SEND_ONCE_BUFFER_SIZE, fp);
    totalSize += ret;
    printf("total read size : %d\n", totalSize);
    printf("----------- pdf read bytes ret = %d\n", ret);

    MopPipeline::DataPackType writePacket = {(uint8_t *) addr, (uint32_t) ret};

    printf("Do sendData to PSDK, size : %d\n", writePacket.length);
    mopRet = handle->sendData(writePacket, &writePacket.length);
    if (mopRet != MOP_PASSED) {
      printf("mop send error,stat:%lld, writePacket.length = %d\n", mopRet,
             writePacket.length);
      break;
    } else {
      printf("mop send success,stat:%lld, writePacket.length = %d\n", mopRet,
             writePacket.length);
    }
    cnt++;
    printf("send cnt %d!\n", cnt);

  } while (totalSize < targetFileSize);

  printf("mop channel send task end!\n");
  free(addr);
  fclose(fp);

  send_finish = true;

  return NULL;
}

int main(int argc, char** argv)
{
  pthread_t recvTask;
  pthread_t sendTask;
  MopErrCode mopRet;
  int result = -1;

  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();

  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }

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
    return -1;
  }

  if (!mopClient) {
    DERROR("Get MOP client object is a null value.");
    return -1;
  }

  /*! connect pipeline */
  MopPipeline *OP_Pipeline = NULL;
  if ((mopClient->connect(TEST_OP_PIPELINE_ID, RELIABLE, OP_Pipeline)
      != MOP_PASSED) || (OP_Pipeline == NULL)) {
    DERROR("MOP Pipeline connect failed");
    return -1;
  } else {
    DSTATUS("Connect to mop pipeline id(%d) successfully", TEST_OP_PIPELINE_ID);
  }

  result = pthread_create(&sendTask, NULL, MopSendTask, (void *)OP_Pipeline);
  if(result != 0) {
    printf("send task create failed!\n");
    send_finish = true;
  }

  result = pthread_create(&recvTask, NULL, MopRecvTask, (void *)OP_Pipeline);
  if(result != 0) {
    printf("recv task create failed!\n");
    recv_finish = true;
  }

  while(!send_finish || !recv_finish) {
    sleep(1);
  }
  /*! Disconnect pipeline */
  if (mopClient->disconnect(TEST_OP_PIPELINE_ID) != MOP_PASSED) {
    DERROR("MOP Pipeline disconnect pipeline(%d) failed", TEST_OP_PIPELINE_ID);
  } else {
    DSTATUS("Disconnect mop pipeline id(%d) successfully", TEST_OP_PIPELINE_ID);
  }
}
