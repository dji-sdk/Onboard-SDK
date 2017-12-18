/*! @file camera_stream_poll_sample.cpp
 *  @version 3.5
 *  @date Dec 06 2017
 *
 *  @brief
 *  AdvancedSensing, Camera Stream API usage in a Linux environment.
 *  This sample shows how to poll new images from FPV camera or/and
 *  main camera from the main thread.
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

void show_rgb(CameraRGBImage img, char* name)
{
  cout << "#### Got image from:\t" << string(name) << endl;
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
       << "f: FPV  Camera\n" 
       << "b: both Cameras" << endl;
  cin >> c;

  switch(c)
  {
  case 'm':
    m=true; break;
  case 'f':
    f=true; break;
  case 'b':
    f=true; m=true; break;
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

  bool fpvCamResult = false;
  bool mainCamResult = false;

  if(f)
  {
    fpvCamResult = vehicle->advancedSensing->startFPVCameraStream();
    if(!fpvCamResult)
    {
      cout << "Failed to open FPV Camera" << endl;
    }
  }
  
  if(m)
  {
    mainCamResult = vehicle->advancedSensing->startMainCameraStream();
    if(!mainCamResult)
    {
      cout << "Failed to open Main Camera" << endl;
    }
  }  

  if((!fpvCamResult) && (!mainCamResult))
  {
    cout << "Failed to Open Either Cameras, exiting" << endl;
    return 1;
  }
  
  CameraRGBImage fpvImg;
  CameraRGBImage mainImg;
  char fpvName[] = "FPV_CAM";
  char mainName[] = "MAIN_CAM";
  for(int i=0; i<1000; i++)
  {
    if(f && fpvCamResult && vehicle->advancedSensing->newFPVCameraImageIsReady())
    {
      if(vehicle->advancedSensing->getFPVCameraImage(fpvImg))
      {
        show_rgb(fpvImg, fpvName);
      }
      else
      {
        cout << "Time out" << endl;
      }
    }

    if(m && mainCamResult && vehicle->advancedSensing->newMainCameraImageReady())
    {

      if(vehicle->advancedSensing->getMainCameraImage(mainImg))
      {
        show_rgb(mainImg, mainName);
      }
      else
      {
        cout << "Time out" << endl;
      }
    }
    usleep(2e4);
  }

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
