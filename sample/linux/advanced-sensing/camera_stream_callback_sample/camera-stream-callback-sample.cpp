/*! @file camera_stream_poll_sample.cpp
 *  @version 3.6
 *  @date Feb 1st 2018
 *
 *  @brief
 *  AdvancedSensing, Camera Stream API usage in a Linux environment.
 *  This sample shows how to use a callback function to process (view) the FPV or
 *  main camera image.
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
 * */


#include "dji_vehicle.hpp"
#include <iostream>
#include "dji_linux_helpers.hpp"

#ifdef OPEN_CV_INSTALLED
  #include "opencv2/opencv.hpp"
  #include "opencv2/highgui/highgui.hpp"
#endif

using namespace DJI::OSDK;
using namespace cv;
using namespace std;

void show_rgb(CameraRGBImage img, void *p)
{
  string name = string(reinterpret_cast<char *>(p));
  cout << "#### Got image from:\t" << name << endl;
#ifdef OPEN_CV_INSTALLED
  Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(), img.width*3);
  cvtColor(mat, mat, COLOR_RGB2BGR);
  imshow(name,mat);
  cv::waitKey(1);
#endif
}

int main(int argc, char** argv)
{
  bool f = false;
  bool m = false;
  char c = 0;
  cout << "Please enter the type of camera stream you want to view\n"
       << "m: Main Camera\n"
       << "f: FPV  Camera" << endl; 
  cin >> c;

  switch(c)
  {
  case 'm':
    m=true; break;
  case 'f':
    f=true; break;
  default:
    cout << "No camera selected";
    return 1;
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

  char fpvName[] = "FPV_CAM";
  char mainName[] = "MAIN_CAM";

  bool camResult = false;
  if(f)
  {
    camResult = vehicle->advancedSensing->startFPVCameraStream(&show_rgb, &fpvName);
  }
  else if(m)
  {
    camResult = vehicle->advancedSensing->startMainCameraStream(&show_rgb, &mainName);
  }

  if(!camResult)
  {
    cout << "Failed to open selected camera" << endl;
    return 1;
  }

  CameraRGBImage camImg;

  // main thread just sleep
  // callback function will be called whenever a new image is ready
  sleep(10);

  if(f)
  {
    vehicle->advancedSensing->stopFPVCameraStream();
  }
  else if(m)
  {
    vehicle->advancedSensing->stopMainCameraStream();
  }

  return 0;
}
