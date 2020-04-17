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
#include <openssl/md5.h>
#include <fcntl.h>
#include <pthread.h>

#define TEST_OP_PIPELINE_ID                    14
#define MOP_VERIFY_FILE_LEN                    (177438017)

//"f32510ba39f6417fdc6c2e898a864e46"
static unsigned char FILE_MD5[] = {0xf3, 0x25, 0x10, 0xba, 0x39, 0xf6, 0x41, 0x7f,
                                   0xdc, 0x6c, 0x2e, 0x89, 0x8a, 0x86, 0x4e, 0x46};

static bool send_finish = false;
static bool recv_finish = false;

using namespace DJI::OSDK;

bool verifyFile(void* buf, uint32_t len, unsigned char* md5_in) {
  MD5_CTX ctx;
  int i = 0;
  unsigned char md5_out[16];
  MD5_Init(&ctx);
  MD5_Update(&ctx, buf, len);
  MD5_Final(md5_out, &ctx);
  if(memcmp(md5_out, md5_in, 16) == 0) {
    return true;
  } else {
    printf("md5 verify failed!\n");
    for(i = 0; i < 16; i++) {
        printf("%02X", md5_out[i]);
    }
    printf("\n");
    return false;
  }
  return true;
}

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

    recvBuf = malloc(MOP_VERIFY_FILE_LEN);
    if (recvBuf == NULL) {
        printf("malloc recv buffer error\n");
        recv_finish = true;
        return NULL;
    }

    MopPipeline::DataPackType readPacket = {(uint8_t *)recvBuf, MOP_VERIFY_FILE_LEN};

    while (1) {
        memset(recvBuf, 0, MOP_VERIFY_FILE_LEN);
        mopRet = handle->recvData(readPacket, &readPacket.length);
        if (mopRet != MOP_PASSED) {
            printf("recv verify file failed!, realLen = %d\n", readPacket.length);
            break;
        }
        if(verifyFile(recvBuf, MOP_VERIFY_FILE_LEN, FILE_MD5) != true) {
            printf("verify file failed!\n");
            break;
        }
        cnt++;
        printf("recv cnt %d!\n", cnt);
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
    int fd = -1;
    void* addr = NULL;
    int cnt = 0;

    handle = (MopPipeline *)arg;
    if(handle == NULL) {
        printf("send task param check failed!\n");
        send_finish = true;
        return NULL;
    }

    fd = open("/home/dji/Downloads/verify.dat", O_RDONLY, 0666);
    if(fd == -1) {
        printf("open file failed!\n");
        send_finish = true;
        return NULL;
    }

    addr = mmap(NULL, MOP_VERIFY_FILE_LEN, PROT_READ, MAP_SHARED, fd, 0);
    if(addr == NULL) {
        printf("mmap file failed!");
        close(fd);
        send_finish = true;
        return NULL;
    }

    MopPipeline::DataPackType writePacket = {(uint8_t *)addr, MOP_VERIFY_FILE_LEN};

    while (1) {
        mopRet = handle->sendData(writePacket, &writePacket.length);
        if (mopRet != MOP_PASSED) {
            printf("mop send error,stat:%d", mopRet);
            break;
        }
        cnt++;
        printf("send cnt %d!\n", cnt);
        if(cnt == 10) {
          break;
        }
        //5 secends for verify
        sleep(5);
    }
   printf("mop channel send task end!\n");

  if(addr != NULL)
    munmap(addr, MOP_VERIFY_FILE_LEN);
  if(fd != -1)
    close(fd);
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
