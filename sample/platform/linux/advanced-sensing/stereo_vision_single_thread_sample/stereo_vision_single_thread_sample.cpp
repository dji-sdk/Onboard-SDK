/*! @file advanced_sensing_sample.cpp
 *  @version 3.3.2
 *  @date Aug 25 2017
 *
 *  @brief
 *  AdvancedSensing API usage in a Linux environment.
 *  Provides an example callback function to subscribe to stereo images
 *  (Optional) With OpenCV installed, user can visualize the images
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

#include "stereo_vision_single_thread_sample.hpp"

using namespace DJI::OSDK;

int
main(int argc, char** argv)
{
  // Setup OSDK.
  bool enableAdvancedSensing = true;
  LinuxSetup linuxEnvironment(argc, argv, enableAdvancedSensing);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Initialize variables
  int functionTimeout = 1;
  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::cout
    << std::endl
    << " AdvancedSensing API works in subscription mechanism              \n"
    << " Please remember to unsubscribe when the program terminates       \n"
    << " If any error messages occur, please reboot the aircraft          \n"
    << std::endl
    << "| Available commands:                                            |\n"
    << "| [a] Subscribe to 240p stereo images                            |\n"
    << "| [b] Subscribe to VGA front stereo images                       |\n"
    << "| [c] Unsubscribe to 240p stereo images                          |\n"
    << "| [d] Unsubscribe to VGA front stereo images                     |\n"
    << std::endl;
  char inputChar = ' ';
  std::cin >> inputChar;

  bool is240p = false, isVGA = false;

  AdvancedSensing::ImageSelection image_select;
  switch (inputChar)
  {
    case 'a':
    {
      memset(&image_select, 0, sizeof(AdvancedSensing::ImageSelection));
      image_select.front_left = 1;
      image_select.front_right = 1;
      image_select.down_front = 1;
      image_select.down_back = 1;
      vehicle->advancedSensing->subscribeStereoImages(&image_select, &stereoImg240pCallback, NULL);
      is240p = true;
    }
      break;
    case 'b':
    {
      vehicle->advancedSensing->subscribeFrontStereoVGA(AdvancedSensingProtocol::FREQ_20HZ, &stereoImgVGACallback, NULL);
      isVGA = true;
    }
      break;
    case 'c':
    {
      vehicle->advancedSensing->unsubscribeStereoImages();
    }
      break;
    case 'd':
    {
      vehicle->advancedSensing->unsubscribeVGAImages();
    }
      break;
    default:
      break;
  }

  DSTATUS("AdvancedSensing sample sleeps 5 seconds here");
  sleep(5);

  if (is240p){
    vehicle->advancedSensing->unsubscribeStereoImages();
    DSTATUS("Unsubscribe 240p stereo images");
    vehicle->advancedSensing->unsubscribeStereoImages();
  }
  if (isVGA){
    DSTATUS("Unsubscribe VGA images");
    vehicle->advancedSensing->unsubscribeVGAImages();
  }

  sleep(1);
  DSTATUS("waited 1 second for the image subscription to stop completely\n");

  return 0;
}

//! @note This callback is running on reading thread.
//! If you would like to run computation inside this
//! callback, note that images are coming at 20 fps,
//! there's only 50ms window for reading + processing.
//! Please make sure your computation does not exceed
//! this limit to prevent dropping frame.
//! Check out multi_thread sample to perform computation on
//! another thread
void stereoImg240pCallback(Vehicle *vehiclePtr, RecvContainer recvFrame, UserData userData)
{
  DSTATUS("sample stereoCallback receive an image at frame: %d and time stamp: %d",
          recvFrame.recvData.stereoImgData->frame_index,
          recvFrame.recvData.stereoImgData->time_stamp);

#ifdef OPEN_CV_INSTALLED
  cv::Mat cv_img_stereo[recvFrame.recvData.stereoImgData->num_imgs];

  for (int i = 0; i < recvFrame.recvData.stereoImgData->num_imgs; ++i)
  {
    cv_img_stereo[i] = cv::Mat(240, 320, CV_8U);

    memcpy(cv_img_stereo[i].data, recvFrame.recvData.stereoImgData->img_vec[i].image,
           sizeof(char)*ACK::IMG_240P_SIZE);

    cv::imshow(recvFrame.recvData.stereoImgData->img_vec[i].name,
               cv_img_stereo[i]);
    cv::waitKey(1);
  }
#endif
}

//! @note This callback is running on reading thread.
//! If you would like to run computation inside this
//! callback, note that images are coming at 10/20 fps,
//! there's only 100/50 ms window for reading + processing.
//! Please make sure your computation does not exceed
//! this limit to prevent dropping frame.
//! Check out multi_thread sample to perform computation on
//! another thread
void stereoImgVGACallback(Vehicle *vehiclePtr, RecvContainer recvFrame, UserData userData)
{
  DSTATUS("sample VGACallback receive an image at frame: %d and time stamp: %d",
          recvFrame.recvData.stereoVGAImgData->frame_index,
          recvFrame.recvData.stereoVGAImgData->time_stamp);

#ifdef OPEN_CV_INSTALLED
  // VGA always comes in pair
  cv::Mat cv_img_stereo[2];
  for (int i = 0; i < 2; ++i) {
    cv_img_stereo[i] = cv::Mat(480, 640, CV_8U);
  }

  memcpy(cv_img_stereo[0].data, recvFrame.recvData.stereoVGAImgData->img_vec[0], sizeof(char)*480*640);
  memcpy(cv_img_stereo[1].data, recvFrame.recvData.stereoVGAImgData->img_vec[1], sizeof(char)*480*640);

  cv::Mat img_to_show;
  cv::hconcat(cv_img_stereo[0], cv_img_stereo[1], img_to_show);
  cv::resize(img_to_show, img_to_show,
             cv::Size(640*2, 480), (0, 0), (0, 0), cv::INTER_LINEAR);

  cv::imshow("cv_img_stereo", img_to_show);
  cv::waitKey(1);
#endif
}
