/*
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

#include "stereo_vision_depth_perception_sample.hpp"

using namespace DJI::OSDK;

int
main(int argc, char** argv)
{
  if(argc >= 3){
    DSTATUS("Input yaml file: %s\n", argv[2]);
  } else{
    DERROR("Please specify a yaml file with camera parameters\n");
    DERROR("Ex: ./stereo-vision-depth-perception-sample UserConfig.txt m210_stereo_param.yaml\n");
    return -1;
  }

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
    << "| [a] Display rectified stereo images                            |\n"
    << "| [b] Display disparity map                                      |\n"
    << "| [c] Display filtered disparity map                             |\n"
    << "| [d] Display point cloud                                        |\n"
    << "| [e] Unsubscribe to VGA front stereo images                     |\n"
    << std::endl;
  char inputChar = ' ';
  std::cin >> inputChar;

  std::string yaml_file_path = argv[2];
  M210_STEREO::Config::setParamFile(yaml_file_path);

  image_process_container_ptr = new StereoProcessContainer(vehicle);

  switch (inputChar)
  {
    case 'a':
    {
      // register a callback for "image processing" thread
      image_process_container_ptr->setCallbackHandler(&StereoProcessContainer::displayStereoRectImgCallback,
                                                      dynamic_cast<StereoProcessContainer*>(image_process_container_ptr));
      // register a callback for "image reading" thread
      vehicle->advancedSensing->subscribeFrontStereoVGA(AdvancedSensingProtocol::FREQ_20HZ, &storeStereoImgVGACallback, NULL);
    }
      break;
    case 'b':
    {
      // register a callback for "image processing" thread
      image_process_container_ptr->setCallbackHandler(&StereoProcessContainer::displayStereoDisparityCallback,
                                                      dynamic_cast<StereoProcessContainer*>(image_process_container_ptr));
      // register a callback for "image reading" thread
      vehicle->advancedSensing->subscribeFrontStereoVGA(AdvancedSensingProtocol::FREQ_20HZ, &storeStereoImgVGACallback, NULL);
    }
      break;
    case 'c':
    {
      // register a callback for "image processing" thread
      image_process_container_ptr->setCallbackHandler(&StereoProcessContainer::displayStereoFilteredDisparityCallback,
                                                      dynamic_cast<StereoProcessContainer*>(image_process_container_ptr));
      // register a callback for "image reading" thread
      vehicle->advancedSensing->subscribeFrontStereoVGA(AdvancedSensingProtocol::FREQ_20HZ, &storeStereoImgVGACallback, NULL);
    }
      break;
    case 'd':
    {
      // register a callback for "image processing" thread
      image_process_container_ptr->setCallbackHandler(&StereoProcessContainer::displayStereoPtCloudCallback,
                                                      dynamic_cast<StereoProcessContainer*>(image_process_container_ptr));
      // register a callback for "image reading" thread
      vehicle->advancedSensing->subscribeFrontStereoVGA(AdvancedSensingProtocol::FREQ_20HZ, &storeStereoImgVGACallback, NULL);
    }
      break;
    case 'e':
    {
      vehicle->advancedSensing->unsubscribeVGAImages();
    }
      break;
    default:
      break;
  }

  DSTATUS("AdvancedSensing sample sleeps 15 seconds here");
  sleep(15);

  DSTATUS("Unsubscribe VGA images");
  vehicle->advancedSensing->unsubscribeVGAImages();

  sleep(1);
  DSTATUS("waited 1 second for the image subscription to stop completely\n");

  return 0;
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
