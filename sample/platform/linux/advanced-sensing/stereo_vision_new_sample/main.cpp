/*! @file advanced-sensing/main.cpp
 *  @version 4.0.0
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

#ifdef OPEN_CV_INSTALLED
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#else
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
#endif

typedef struct StereoImagePacketType {
  Perception::ImageInfoType info;
  uint8_t *imageRawBuffer;
  T_OsdkMutexHandle mutex;
  bool gotData;
} StereoImagePacketType;

void PerceptionImageCB(Perception::ImageInfoType info, uint8_t *imageRawBuffer,
                       int bufferLen, void *userData) {
  DSTATUS("image info : dataId(%d) seq(%d) timestamp(%llu) datatype(%d) index(%d) h(%d) w(%d) dir(%d) "
          "bpp(%d) bufferlen(%d)", info.dataId, info.sequence, info.timeStamp, info.dataType,
          info.rawInfo.index, info.rawInfo.height, info.rawInfo.width, info.rawInfo.direction,
          info.rawInfo.bpp, bufferLen);
  if (imageRawBuffer && userData) {
    StereoImagePacketType *pack = (StereoImagePacketType*)userData;
    OsdkOsal_MutexLock(pack->mutex);
    pack->info = info;
    if (pack->imageRawBuffer) OsdkOsal_Free(pack->imageRawBuffer);
    pack->imageRawBuffer = (uint8_t *)OsdkOsal_Malloc(bufferLen);
    memcpy(pack->imageRawBuffer, imageRawBuffer, bufferLen);
    pack->gotData = true;
    OsdkOsal_MutexUnlock(pack->mutex);
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
              pack.cameraParam[i].rightIntrinsics[0],
              pack.cameraParam[i].rightIntrinsics[1],
              pack.cameraParam[i].rightIntrinsics[2],
              pack.cameraParam[i].rightIntrinsics[3],
              pack.cameraParam[i].rightIntrinsics[4],
              pack.cameraParam[i].rightIntrinsics[5],
              pack.cameraParam[i].rightIntrinsics[6],
              pack.cameraParam[i].rightIntrinsics[7],
              pack.cameraParam[i].rightIntrinsics[8]);
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

static void *stereoImagesDisplayTask(void *arg) {
  while (1) {
    OsdkOsal_TaskSleepMs(1);
    if (arg) {
      StereoImagePacketType *pack = (StereoImagePacketType*)arg;

#ifdef OPEN_CV_INSTALLED
      char name[20] = {0};
      /*! Get data here */
      OsdkOsal_MutexLock(pack->mutex);
      if (!pack->gotData) {
        OsdkOsal_MutexUnlock(pack->mutex);
        continue;
      }
      cv::Mat cv_img_stereo = cv::Mat(pack->info.rawInfo.height, pack->info.rawInfo.width, CV_8U);
      int copySize = pack->info.rawInfo.height * pack->info.rawInfo.width;
      if (pack->imageRawBuffer) {
        memcpy(cv_img_stereo.data, pack->imageRawBuffer, copySize);
        OsdkOsal_Free(pack->imageRawBuffer);
        pack->imageRawBuffer = NULL;
      }
      sprintf(name, "Image_dataType : %d", pack->info.dataType);
      pack->gotData = false;
      OsdkOsal_MutexUnlock(pack->mutex);
      /*! Using Opencv display here */
      cv::imshow(name, cv_img_stereo);
      cv::waitKey(1);
#else
      OsdkOsal_MutexLock(pack->mutex);
      if (!pack->gotData) {
        OsdkOsal_MutexUnlock(pack->mutex);
        continue;
      }
      int copySize = pack->info.rawInfo.height * pack->info.rawInfo.width;
      uint8_t *imageRawBuffer = (uint8_t *)OsdkOsal_Malloc(copySize);
      memcpy(imageRawBuffer, pack->imageRawBuffer, copySize);
      OsdkOsal_MutexUnlock(pack->mutex);

      DSTATUS("Save images at local path.");
      writePictureData(imageRawBuffer, copySize);
      OsdkOsal_Free(imageRawBuffer);

#endif


    }
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

  /*! Image Data Container */
  StereoImagePacketType   stereoImagePacket = {
    .info           = { 0 },
    .imageRawBuffer = NULL,
    .mutex          = NULL,
    .gotData        = false };

  /*! Display Thread */
  static T_OsdkTaskHandle stereoImageThread;
  OsdkOsal_MutexCreate(&stereoImagePacket.mutex);
  OsdkOsal_TaskCreate(&stereoImageThread, stereoImagesDisplayTask,
                      OSDK_TASK_STACK_SIZE_DEFAULT, &stereoImagePacket);

  char inputChar;

  while(1) {
    // Display interactive prompt
    std::cout
        << "| Available commands:                                            |"
        << std::endl;
    std::cout
        << "| [a] Subscribe down stereo camera pair images                   |"
        << std::endl;
    std::cout
        << "| [b] Subscribe front stereo camera pair images                  |"
        << std::endl;
    std::cout
        << "| [c] Subscribe rear stereo camera pair images                   |"
        << std::endl;
    std::cout
        << "| [d] Subscribe up stereo camera pair images                     |"
        << std::endl;
    std::cout
        << "| [e] Subscribe left stereo camera pair images                   |"
        << std::endl;
    std::cout
        << "| [f] Subscribe right stereo camera pair images                  |"
        << std::endl;
    std::cout
        << "| [g] get the parameters of all the stereo cameras               |"
        << std::endl;
    std::cout
        << "| [q] quit                                                       |"
        << std::endl;

    Perception::DirectionType direction = (Perception::DirectionType)0xFF;
    std::cin >> inputChar;
    switch (inputChar) {
      case 'a':
        DSTATUS("Subscribe down stereo camera pair images.");
        direction = Perception::DirectionType::RECTIFY_DOWN;
        break;
      case 'b':
        DSTATUS("Subscribe front stereo camera pair images.");
        direction = Perception::DirectionType::RECTIFY_FRONT;
        break;
      case 'c':
        DSTATUS("Subscribe rear stereo camera pair images.");
        direction = Perception::DirectionType::RECTIFY_REAR;
        break;
      case 'd':
        DSTATUS("Subscribe up stereo camera pair images.");
        direction = Perception::DirectionType::RECTIFY_UP;
        break;
      case 'e':
        DSTATUS("Subscribe left stereo camera pair images.");
        direction = Perception::DirectionType::RECTIFY_LEFT;
        break;
      case 'f':
        DSTATUS("Subscribe right stereo camera pair images.");
        direction = Perception::DirectionType::RECTIFY_RIGHT;
        break;
      case 'g':
        DSTATUS("Do stereo camera parameters subscription");
        vehicle->advancedSensing->setStereoCamParamsObserver(PerceptionCamParamCB, NULL);
        vehicle->advancedSensing->triggerStereoCamParamsPushing();
        break;
      case 'q':return 0;
      default:break;
    }

    if (direction != (Perception::DirectionType)0xFF) {
      OsdkOsal_TaskSleepMs(1000);
      DSTATUS("Do stereo camera imagines subscription");
      vehicle->advancedSensing->subscribePerceptionImage(direction, PerceptionImageCB, &stereoImagePacket);
      OsdkOsal_TaskSleepMs(10000);
      vehicle->advancedSensing->unsubscribePerceptionImage(direction);
      OsdkOsal_TaskSleepMs(1000);
    }
  }
  OsdkOsal_MutexDestroy(stereoImagePacket.mutex);
  return 0;
}
