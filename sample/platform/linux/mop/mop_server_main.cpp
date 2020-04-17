/*! @file mop_server_main.cpp
 *  @version 4.0
 *  @date March 6 2020
 *
 *  @brief Sample to show how to use mop as a server.
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

using namespace DJI::OSDK;

#define TEST_MO_PIPELINE_ID 20

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


int readStreamData(const char *fileName, uint32_t pos, uint8_t *data, uint32_t len) {
  FILE *fp = NULL;
  size_t size = 0;

  fp = fopen(fileName, "r+");
  if(fp == NULL) {
    printf("fopen failed!\n");
    return -1;
  }

  if(fseek(fp, pos, SEEK_SET)) {
    printf("fseek failed!\n");
    return -1;
  }

  size = fread(data, 1, len, fp);

  fclose(fp);

  return size;
}

void *recvTask(void *arg) {
  uint64_t recvBytesCnt = 0;
  if (arg) {
    uint8_t readBuffer[1024];
    MopPipeline::DataPackType readPacket = {readBuffer, sizeof(readBuffer)};
    DSTATUS("MOP server recv thread created!");
    MopPipeline *p = (MopPipeline *) arg;
    while (1) {
      DSTATUS("Loop...");
      MopErrCode ret = p->recvData(readPacket, &readPacket.length);
      if (ret == MOP_PASSED) {
        recvBytesCnt += readPacket.length;
        DSTATUS("Receive %d bytes from MSDK (total %d)", readPacket.length, recvBytesCnt);
        if (readPacket.length >= 3)
          DSTATUS("data[0]=%d data[1]=%d data[2]=%d ..",
                  readPacket.length, readPacket.data[0], readPacket.data[1],
                  readPacket.data[2]);
          writeStreamData("./output", readPacket.data, readPacket.length);
      } else {
        DERROR("Receive bytes from MSDK failed (%d)", ret);
      }
    }
  }
}

int main(int argc, char** argv)
{
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  T_OsdkTaskHandle recvTaskHandle;

  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }

  MopPipeline *MO_Pipeline = NULL;

  /*! Do accepting */
  if (vehicle->mopServer->accept((PipelineID)TEST_MO_PIPELINE_ID, UNRELIABLE, MO_Pipeline)
      != MOP_PASSED) {
    DERROR("MOP Pipeline accept failed");
    return -1;
  } else {
    DSTATUS("Accept to mop pipeline id(%d) successfully", TEST_MO_PIPELINE_ID);
  }
  OsdkOsal_TaskSleepMs(10*1000);
  int ret = 0;

  OsdkOsal_TaskCreate(&recvTaskHandle, (void *(*)(void *)) recvTask,
                      OSDK_TASK_STACK_SIZE_DEFAULT, MO_Pipeline);

  while (1) {
    /*! Writing data to MSDK */
    static uint64_t file_pos = 0;
    uint8_t writeBuffer[1024] = {0};
    static uint8_t c = 0;
    static uint64_t times = 0;
  
    int readBytes = readStreamData("./mop_server_sample", file_pos, writeBuffer, sizeof(writeBuffer));
    if (readBytes > 0) {
      file_pos += readBytes;
      DSTATUS("readBytes ...   %d", readBytes);
      DSTATUS("file_pos  ...   %d", file_pos);
      OsdkOsal_TaskSleepMs(500);
    } else {
      if (readBytes == 0) {
        DSTATUS("Read Finished ...");
      } else {
        DSTATUS("Read file failed ...");
      }
      OsdkOsal_TaskSleepMs(1000);
      continue;
    }

    times++;
    if ((times % 1) == 0)c++;
    MopPipeline::DataPackType writePacket = {writeBuffer, (uint32_t)readBytes};

    ret = MO_Pipeline->sendData(writePacket, &writePacket.length);
    if (ret == MOP_PASSED) {
      DSTATUS("-----Send %d bytes to MSDK :", writePacket.length);
      if (writePacket.length >= 3)
        DSTATUS("-----data[0]=%d data[1]=%d data[2]=%d ..",
                writePacket.length, writePacket.data[0], writePacket.data[1],
                writePacket.data[2]);
    } else {
      DERROR("Send bytes to MSDK failed (%d)", ret);
    }
    DSTATUS("mark .............................    2333");
    OsdkOsal_TaskSleepMs(500);
    int keyBoardInput = non_block_get_char();
    if (keyBoardInput == 'q') {
      OsdkOsal_TaskSleepMs(1000);
      DSTATUS("Got key board input : q, exit the test.");
      OsdkOsal_TaskSleepMs(2000);
      break;
    }
  }

  /*! close pipeline */
  if (vehicle->mopServer->close(TEST_MO_PIPELINE_ID) != MOP_PASSED) {
    DERROR("MOP Pipeline disconnect pipeline(%d) failed", TEST_MO_PIPELINE_ID);
  } else {
    DSTATUS("Disconnect mop pipeline id(%d) successfully", TEST_MO_PIPELINE_ID);
  }
}
