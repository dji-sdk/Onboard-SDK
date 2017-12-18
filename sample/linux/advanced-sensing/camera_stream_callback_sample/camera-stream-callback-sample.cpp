/*! @file camera_stream_poll_sample.cpp
 *  @version 3.5
 *  @date Dec 06 2017
 *
 *  @brief
 *  AdvancedSensing, Camera Stream API usage in a Linux environment.
 *  This sample shows how to use a callback function to process (view) the FPV or
 *  main camera image.
 *  (Optional) With OpenCV installed, user can visualize the images
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
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
