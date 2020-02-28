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
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include "dji_vehicle.hpp"

using namespace DJI::OSDK;

#define IMAGE_FILE_PATH                "./image"
#define IMAGE_FILE_PATH_LEN            (64)
#define IMAGE_INFO_LEN                 (sizeof(DUSS_MSG_OSDK_IMAGE_INFO_t))
#define IMAGE_MAX_DIRECTION_NUM        (6)

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

void PerceptionImageCB(Perception::ImageInfoType info, uint8_t *imageRawBuffer,
                       int bufferLen, void *userData) {
  DSTATUS("image info : dataId(%d) seq(%d) timestamp(%d) datatype(%d)", info.dataId, info.sequence,
          info.timeStamp, info.dataType);
  DSTATUS("image info : index(%d) h(%d) w(%d) dir(%d) bpp(%d) bufferlen(%d)",
          info.rawInfo.index, info.rawInfo.height, info.rawInfo.width,
          info.rawInfo.direction, info.rawInfo.bpp, bufferLen);
  if (imageRawBuffer) {
    writePictureData(imageRawBuffer, bufferLen);
  }
}

void PerceptionCamParamCB(Perception::CamParamPacketType pack,
                          void *userData) {
  DSTATUS("stereo cam parameters : timestamp(%d) dirNum(%d)", pack.timeStamp, pack.directionNum);
  if ((pack.directionNum > 0) && (pack.directionNum <= IMAGE_MAX_DIRECTION_NUM))
    for (int i = 0 ; i < pack.directionNum; i++) {
      DSTATUS("dir[%d] parameters :", pack.cameraParam[i].direction);
      DSTATUS("\tleftIntrinsics \t= {%f, %f, %f, %f, %f, %f, %f, %f, %f }",
              pack.cameraParam[i].leftIntrinsics[0],
              pack.cameraParam[i].leftIntrinsics[1],
              pack.cameraParam[i].leftIntrinsics[2],
              pack.cameraParam[i].leftIntrinsics[3],
              pack.cameraParam[i].leftIntrinsics[4],
              pack.cameraParam[i].leftIntrinsics[5],
              pack.cameraParam[i].leftIntrinsics[6],
              pack.cameraParam[i].leftIntrinsics[7],
              pack.cameraParam[i].leftIntrinsics[8]);
      DSTATUS("\trightIntrinsics \t= {%f, %f, %f, %f, %f, %f, %f, %f, %f }",
              pack.cameraParam[i].rightIntrinsic[0],
              pack.cameraParam[i].rightIntrinsic[1],
              pack.cameraParam[i].rightIntrinsic[2],
              pack.cameraParam[i].rightIntrinsic[3],
              pack.cameraParam[i].rightIntrinsic[4],
              pack.cameraParam[i].rightIntrinsic[5],
              pack.cameraParam[i].rightIntrinsic[6],
              pack.cameraParam[i].rightIntrinsic[7],
              pack.cameraParam[i].rightIntrinsic[8]);
      DSTATUS("\trotaionLeftInRight \t= {%f, %f, %f, %f, %f, %f, %f, %f, %f }",
              pack.cameraParam[i].rotaionLeftInRight[0],
              pack.cameraParam[i].rotaionLeftInRight[1],
              pack.cameraParam[i].rotaionLeftInRight[2],
              pack.cameraParam[i].rotaionLeftInRight[3],
              pack.cameraParam[i].rotaionLeftInRight[4],
              pack.cameraParam[i].rotaionLeftInRight[5],
              pack.cameraParam[i].rotaionLeftInRight[6],
              pack.cameraParam[i].rotaionLeftInRight[7],
              pack.cameraParam[i].rotaionLeftInRight[8]);
      DSTATUS("\ttranslationLeftInRight \t= {%f, %f, %f }",
              pack.cameraParam[i].translationLeftInRight[0],
              pack.cameraParam[i].translationLeftInRight[1],
              pack.cameraParam[i].translationLeftInRight[2]);
    }
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

  DSTATUS("This sample only supports the M300. This is the beta version,"
          "and the APIs wil be optimized in the future.");

  char inputChar;

  while(1) {
    // Display interactive prompt
    std::cout
        << "| Available commands:                                            |"
        << std::endl;
    std::cout
        << "| [a] Front stereo camera advanced sensing function test         |"
        << std::endl;
    std::cout
        << "| [b] Upward stereo camera advanced sensing function test        |"
        << std::endl;
    std::cout
        << "| [q] quit                                                       |"
        << std::endl;

    std::cin >> inputChar;
    switch (inputChar)
    {
      case 'a': {
        DSTATUS("Do stereo camera parameters subscription");
        vehicle->advancedSensing->setStereoCamParamsObserver(PerceptionCamParamCB, NULL);
        vehicle->advancedSensing->triggerStereoCamParamsPushing();
        DSTATUS("Do stereo camera imagines subscription");
        vehicle->advancedSensing->subscribePerceptionImage(
            Perception::DirectionType::RECTIFY_FRONT, PerceptionImageCB, NULL);
        sleep(5);
        DSTATUS("Do stereo camera imagines unsubscription");
        vehicle->advancedSensing->unsubscribePerceptionImage(
            Perception::DirectionType::RECTIFY_FRONT);
      }
        break;
      case 'b': {
        DSTATUS("Do stereo camera parameters subscription");
        vehicle->advancedSensing->setStereoCamParamsObserver(PerceptionCamParamCB, NULL);
        vehicle->advancedSensing->triggerStereoCamParamsPushing();
        DSTATUS("Do stereo camera imagines subscription");
        vehicle->advancedSensing->subscribePerceptionImage(
            Perception::DirectionType::RECTIFY_UP, PerceptionImageCB, NULL);
        sleep(5);
        DSTATUS("Do stereo camera imagines unsubscription");
        vehicle->advancedSensing->unsubscribePerceptionImage(
            Perception::DirectionType::RECTIFY_UP);
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
