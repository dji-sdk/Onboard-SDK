/*! @file advanced-sensing/main.cpp
 *  @version 3.4
 *  @date Sep 15 2017
 *
 *  @brief
 *  Logging API usage in a Linux environment.
 *  Shows example usage of various advanced sensing APIs and controls.
 *
 *  @Copyright (c) 2017 DJI
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

#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

using namespace DJI;
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

void liveViewSampleCb(uint8_t* buf, int bufLen, void* userData) {
  if (userData) {
    const char *filename = (const char *) userData;
    writeStreamData(filename, buf, bufLen);
  } else {
  DERROR("userData is a null value (should be a file name to log h264 stream).");
  }
}


int
main(int argc, char** argv) {
  // Setup OSDK.
  bool enableAdvancedSensing = true;
  LinuxSetup linuxEnvironment(argc, argv, enableAdvancedSensing);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  if (vehicle == nullptr) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  /*! sample loop start */
  char inputChar = 0;
  while (true) {
    std::cout << std::endl;
    std::cout
        << "| Available commands:                                            |"
        << std::endl
        << "| [a] Start getting FPV H264 stream sample                       |"
        << std::endl
        << "| [b] Start getting main camera H264 stream sample               |"
        << std::endl
        << "| [c] Start getting vice camera H264 stream sample               |"
        << std::endl
        << "| [d] Start getting top camera H264 stream sample                |"
        << std::endl;

    std::cin >> inputChar;

    struct timeval tv;
    struct tm tm;
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);

    char h264FileName[50] = {0};

    switch (inputChar) {
      case 'a':
        sprintf(h264FileName, "FPV_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
                tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_FPV,
                                           liveViewSampleCb,
                                           (void *) h264FileName);
        break;
      case 'b':
        sprintf(h264FileName, "MainCam_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
                tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_NO_1,
                                           liveViewSampleCb,
                                           (void *) h264FileName);
        break;
      case 'c':
        sprintf(h264FileName, "ViceCam_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
                tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_NO_2,
                                           liveViewSampleCb,
                                           (void *) h264FileName);
        break;
      case 'd':
        sprintf(h264FileName, "TopCam_%d-%d-%d_%d-%d-%d.h264", tm.tm_year + 1900,
                tm.tm_mon, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
        vehicle->advancedSensing->startH264Stream(LiveView::OSDK_CAMERA_POSITION_NO_3,
                                           liveViewSampleCb,
                                           (void *) h264FileName);
        break;
    }

    DSTATUS("Wait 10 second to record stream");
    sleep(10);

    switch (inputChar) {
      case 'a':
        vehicle->advancedSensing->stopH264Stream(LiveView::OSDK_CAMERA_POSITION_FPV);
        break;
      case 'b':
        vehicle->advancedSensing->stopH264Stream(LiveView::OSDK_CAMERA_POSITION_NO_1);
        break;
      case 'c':
        vehicle->advancedSensing->stopH264Stream(LiveView::OSDK_CAMERA_POSITION_NO_2);
        break;
      case 'd':
        vehicle->advancedSensing->stopH264Stream(LiveView::OSDK_CAMERA_POSITION_NO_3);
        break;

    }
  }
}
