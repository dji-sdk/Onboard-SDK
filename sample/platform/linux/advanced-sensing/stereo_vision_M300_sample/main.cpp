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

#include "dji_linux_helpers.hpp"
#include "dji_vehicle.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace DJI::OSDK;

#define IMAGE_FILE_PATH                "./image"
#define IMAGE_FILE_PATH_LEN            (64)

int writePictureData(const uint8_t *data, uint32_t len) {
  DIR *dirp = NULL;
  FILE *fp = NULL;
  char fileName[IMAGE_FILE_PATH_LEN] = {0};
  size_t size = 0;
  static uint32_t index = 0;

  dirp = opendir(IMAGE_FILE_PATH);
  if(dirp == NULL && mkdir(IMAGE_FILE_PATH, S_IRWXU | S_IRGRP | S_IROTH | S_IXOTH)) {
    return -1;
  }
  closedir(dirp);
  
  snprintf(fileName, IMAGE_FILE_PATH_LEN, "%s/%d.raw", IMAGE_FILE_PATH, index);
  
  fp = fopen(fileName, "w+");
  if(fp == NULL) {
    return -1;
  }
  size = fwrite(data, 1, len, fp);
  if(size != len) {
    return -1;
  }
  
  if(fp) {
    fclose(fp);
  }
  index++;
  return 0;
}

void sampleCB (Perception::ImageInfoType info, uint8_t *imageRawBuffer, int bufferLen, void *userData) {
  DSTATUS("A perception image is received");
  DSTATUS("Image index            : %d", info.rawInfo.index);
  DSTATUS("Image camera direction : %d", info.rawInfo.direction);
  DSTATUS("Image width            : %d", info.rawInfo.width);
  DSTATUS("Image height           : %d", info.rawInfo.height);
  DSTATUS("Image camera data      : %d", info.dataType);
  writePictureData(imageRawBuffer, bufferLen);
}

int
main(int argc, char** argv)
{

  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();

  if (vehicle == NULL) {
      std::cout << "Vehicle not initialized, exiting. \n";
      return -1;
  }

  DSTATUS("This sample only support for M300");
  char inputChar;

  while(1) {
	  // Display interactive prompt
    std::cout
        << "| Aailable commands:                                              |"
        << std::endl;
    std::cout
        << "| [l] subscribe leftward perception camera images for 10 seconds  |"
        << std::endl;
    std::cout
        << "| [r] subscribe rightward perception camera images for 10 seconds |"
        << std::endl;
    std::cout
        << "| [f] subscribe frontward perception camera images for 10 seconds |"
        << std::endl;
    std::cout
        << "| [b] subscribe backward perception camera images for 10 seconds  |"
        << std::endl;
    std::cout
        << "| [u] subscribe upward perception camera images for 10 seconds    |"
        << std::endl;
    std::cout
        << "| [d] subscribe downward perception camera images for 10 seconds  |"
        << std::endl;
    std::cout
        << "| [q] quit                                                        |"
        << std::endl;

    std::cin >> inputChar;
    switch (inputChar)
    {
      case 'l': {
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_LEFT, sampleCB, NULL);
        sleep(10);
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_LEFT, sampleCB, NULL);
      }
        break;
      case 'r': {
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_RIGHT, sampleCB, NULL);
        sleep(10);
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_RIGHT, sampleCB, NULL);
      }
        break;
      case 'f': {
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_FRONT, sampleCB, NULL);
        sleep(10);
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_FRONT, sampleCB, NULL);
      }
        break;
      case 'b': {
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_REAR, sampleCB, NULL);
        sleep(10);
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_REAR, sampleCB, NULL);
      }
        break;
      case 'u': {
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_UP, sampleCB, NULL);
        sleep(10);
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_UP, sampleCB, NULL);
      }
        break;
      case 'd': {
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_DOWN, sampleCB, NULL);
        sleep(10);
        vehicle->advancedSensing->subscribePerceptionImage(Perception::RECTIFY_DOWN, sampleCB, NULL);
      }
        break;
      case 'q': {
        return 0;
      }
        break;
      default:
        break;
    }
  }

  return 0;
}
