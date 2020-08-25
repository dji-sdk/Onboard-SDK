/*! @file payloads/download_sample.cpp
 *  @version 4.0.0
 *  @date July 29 2019
 *
 *  @brief
 *  Show a example of downloading filelist and files from camera.
 *
 *  @Copyright (c) 2019 DJI
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

#include <dji_linux_helpers.hpp>

FilePackage cur_file_list;
void fileListReqCB(E_OsdkStat ret_code, const FilePackage file_list, void* udata) {
  DSTATUS("\033[1;32;40m##[%s] : ret = %d \033[0m", udata, ret_code);
  if (ret_code == OSDK_STAT_OK) {
    cur_file_list = file_list;
    DSTATUS("file_list.type = %d", file_list.type);
    DSTATUS("file_list.media.size() = %d", file_list.media.size());
    for (auto &file : file_list.media) {
      if ((file.fileSize > 0) && (file.valid))
      printMediaFileMsg(file);
    }
  }
}

bool fileDataDownloadFinished = false;
void fileDataReqCB(E_OsdkStat ret_code, void *udata) {
  if (ret_code == OSDK_STAT_OK) {
    DSTATUS("\033[1;32;40m##Download file [%s] successfully. \033[0m", udata);
  } else {
    DERROR("\033[1;31;40m##Download file data failed. \033[0m");
  }
  fileDataDownloadFinished = true;
}

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char **argv) {
  /*! Setup the OSDK: Read config file, create vehicle, activate. */
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle *vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    DERROR("Vehicle not initialized, exiting.");
    return -1;
  }


  /*! sample loop start */
  while (true) {
    std::cout << std::endl;
    std::cout
        << "| [a] Download main camera filelist                              |"
        << std::endl
        << "| [b] Download main camera filedata from case a                  |"
        << std::endl
        << "| [q] Quit                                                       |"
        << std::endl;
    char inputChar = 0;
    std::cin >> inputChar;

    switch (inputChar) {
      case 'a': {
        ErrorCode::ErrorCodeType ret;
        DSTATUS("Play back mode setting......");
        vehicle->cameraManager->setModeSync(PAYLOAD_INDEX_0,
                                            CameraModule::WorkMode::PLAYBACK,
                                            2);
        DSTATUS("Get liveview right......");
         ret = vehicle->cameraManager->obtainDownloadRightSync(PAYLOAD_INDEX_0,
                                                            true, 2);
        ErrorCode::printErrorCodeMsg(ret);
        DSTATUS("Try to download file list  .......");
        ret = vehicle->cameraManager->startReqFileList(
          PAYLOAD_INDEX_0,
          fileListReqCB,
          (void*)("Download main camera file list"));
        ErrorCode::printErrorCodeMsg(ret);
        break;
      }
      case 'b': {
        ErrorCode::ErrorCodeType ret;
        DSTATUS("Download file number : %d", cur_file_list.media.size());
        uint32_t downloadCnt = cur_file_list.media.size();
        if (downloadCnt > 4) downloadCnt = 4;
        DSTATUS("Now try to download %d media files from main camera.", downloadCnt);
        for (uint32_t i = 0; i < downloadCnt; i++) {
          fileDataDownloadFinished = false;
          DSTATUS("playback mode......");
          vehicle->cameraManager->setModeSync(PAYLOAD_INDEX_0,
                                              CameraModule::WorkMode::PLAYBACK,
                                              2);
          DSTATUS("Get liveview right......");
          ret = vehicle->cameraManager->obtainDownloadRightSync(
            PAYLOAD_INDEX_0, true, 2);
          ErrorCode::printErrorCodeMsg(ret);

          DSTATUS("Try to download file list  .......");
          char pathBuffer[100] = {0};
          MediaFile targetFile = cur_file_list.media[i];
          sprintf(pathBuffer, "./%s", targetFile.fileName.c_str());
          std::string localPath(pathBuffer);

          DSTATUS("targetFile.fileIndex = %d, localPath = %s", targetFile.fileIndex, localPath.c_str());
          ret = vehicle->cameraManager->startReqFileData(
            PAYLOAD_INDEX_0,
            targetFile.fileIndex,
            localPath,
            fileDataReqCB,
            (void*)(localPath.c_str()));
          ErrorCode::printErrorCodeMsg(ret);
          while (fileDataDownloadFinished == false) {
            OsdkOsal_TaskSleepMs(1000);
          }
          DSTATUS("Prepare to do next downloading ...");
          OsdkOsal_TaskSleepMs(1000);
        }
        break;
      }
      case 'q':
        DSTATUS("Quit now ...");
        return 0;
      default:
        break;
    }
    DSTATUS("Sample end ...");
    sleep(2);
    inputChar = 0;
  }
  return 0;
}
