/*! @file advanced_sensing_multi_thread_sample.cpp
 *  @version 3.4
 *  @date Oct 10 2017
 *
 *  @brief
 *  AdvancedSensing API usage in a Linux environment.
 *  Provides an example callback function to subscribe to stereo images
 *  and an example class to process images in a separate thread
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

#include "stereo_vision_multi_thread_sample.hpp"

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

  image_process_container_ptr = new ImageProcessContainer(vehicle);

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
      // register a callback for "image processing" thread
      image_process_container_ptr->setCallbackHandler(&ImageProcessContainer::displayStereo240pCallback,
                                                      image_process_container_ptr);
      // register a callback for "image reading" thread
      vehicle->advancedSensing->subscribeStereoImages(&image_select, &storeStereoImg240pCallback, NULL);
      is240p = true;
    }
      break;
    case 'b':
    {
      // register a callback for "image processing" thread
      image_process_container_ptr->setCallbackHandler(&ImageProcessContainer::displayStereoVGACallback,
                                                      image_process_container_ptr);
      // register a callback for "image reading" thread
      vehicle->advancedSensing->subscribeFrontStereoVGA(AdvancedSensingProtocol::FREQ_20HZ, &storeStereoImgVGACallback, NULL);
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
//! Another thread is created inside ImageProcessContainer.
//! Whenever the images are copied, another callback on
//! processing thread will perform the image computation
void storeStereoImg240pCallback(Vehicle *vehiclePtr, RecvContainer recvFrame, UserData userData)
{
  DSTATUS("sample stereoCallback receive an image at frame: %d and time stamp: %d",
          recvFrame.recvData.stereoImgData->frame_index,
          recvFrame.recvData.stereoImgData->time_stamp);

  // copy the image data to image process container
  // a boolean is flipped to inform the polling
  // mechanism in image process thread
  image_process_container_ptr->copy240pImg(recvFrame.recvData.stereoImgData);
}

//! @note This callback is running on reading thread.
//! Another thread is created inside ImageProcessContainer.
//! Whenever the images are copied, another callback on
//! processing thread will perform the image computation
void storeStereoImgVGACallback(Vehicle *vehiclePtr, RecvContainer recvFrame, UserData userData)
{
  DSTATUS("sample VGACallback receive an image at frame: %d and time stamp: %d",
          recvFrame.recvData.stereoVGAImgData->frame_index,
          recvFrame.recvData.stereoVGAImgData->time_stamp);

  // copy the image data to image process container
  // a boolean is flipped to inform the polling
  // mechanism in image process thread
  image_process_container_ptr->copyVGAImg(recvFrame.recvData.stereoVGAImgData);
}
