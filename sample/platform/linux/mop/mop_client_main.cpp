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

using namespace DJI::OSDK;

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

int main(int argc, char** argv)
{
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();

  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }

  MopPipeline *O2PWrOnlyPipeline = NULL;
  MopPipeline *P2ORdOnlyPipeline = NULL;

  if ((vehicle->mopClient->connect(P2O_PIPELINE, UNRELIABLE, SLOT_0,
                                   P2ORdOnlyPipeline) != MOP_PASSED)
      || (P2ORdOnlyPipeline == NULL)) {
    DERROR("MOP Pipeline connect failed");
    return -1;
  } else {
    DSTATUS("Connect to mop pipeline id(%d) successfully", P2O_PIPELINE);
  }

  if ((vehicle->mopClient->connect(O2P_PIPELINE, UNRELIABLE, SLOT_0,
                                   O2PWrOnlyPipeline) != MOP_PASSED)
      || (O2PWrOnlyPipeline == NULL)) {
    DERROR("MOP Pipeline connect failed");
    return -1;
  } else {
    DSTATUS("Connect to mop pipeline id(%d) successfully", O2P_PIPELINE);
  }

  /*! Define the buffer and the log file name */
  uint8_t readBuffer[1024];
  uint8_t writeBuffer[1024];
  MopErrCode ret;
  uint32_t transTimes = 0;
  struct timeval tv;
  struct tm tm;
  gettimeofday(&tv, NULL);
  localtime_r(&tv.tv_sec, &tm);
  char logFileName[128] = {0};
  sprintf(logFileName, "mop_file_%d-%d-%d_%d::%d::%d", tm.tm_year + 1900,
          tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

  while (1) {
    /*! Reading data from PSDK */
    MopPipeline::DataPackType readPacket = {readBuffer, sizeof(readBuffer)};
    ret = P2ORdOnlyPipeline->recvData(readPacket, &readPacket.length);
    if (ret == MOP_PASSED) {
      DSTATUS("Receive %d bytes from PSDK :", readPacket.length);
      if (readPacket.length >= 3)
        DSTATUS("data[0]=%d data[1]=%d data[2]=%d ..",
                readPacket.length, readPacket.data[0], readPacket.data[1],
                readPacket.data[2]);
      writeStreamData(logFileName, readPacket.data, readPacket.length);
      transTimes++;
    } else {
      DERROR("Receive bytes from PSDK failed (%d)", ret);
    }

    /*! Writing data to PSDK */
    if ((transTimes % 100) == 0) {
      /*! transTimes/100 means 100k Bytes */
      memset(writeBuffer, (uint8_t)(transTimes/100), sizeof(writeBuffer));
      MopPipeline::DataPackType writePacket = {writeBuffer, sizeof(writeBuffer)};
      //sprintf((char *)writeBuffer, "Success transing time : %d", transTimes);
      //ret = O2PWrOnlyPipeline->sendData(writePacket, &writePacket.length);
      ret = P2ORdOnlyPipeline->sendData(writePacket, &writePacket.length);
      if (ret == MOP_PASSED) {
        DSTATUS("-----Send %d bytes to PSDK :", writePacket.length);
        if (writePacket.length >= 3)
          DSTATUS("-----data[0]=%d data[1]=%d data[2]=%d ..",
                  writePacket.length, writePacket.data[0], writePacket.data[1],
                  writePacket.data[2]);
      } else {
        DERROR("Send bytes to PSDK failed (%d)", ret);
      }
    }
  }

  /*! Disconnect pipeline */
  if (vehicle->mopClient->disconnect(P2O_PIPELINE) != MOP_PASSED) {
    DERROR("MOP Pipeline disconnect pipeline(%d) failed", P2O_PIPELINE);
  } else {
    DSTATUS("Disconnect mop pipeline id(%d) successfully", P2O_PIPELINE);
  }

  if (vehicle->mopClient->disconnect(O2P_PIPELINE) != MOP_PASSED) {
    DERROR("MOP Pipeline disconnect pipeline(%d) failed", O2P_PIPELINE);
  } else {
    DSTATUS("Disconnect mop pipeline id(%d) successfully", O2P_PIPELINE);
  }
}
