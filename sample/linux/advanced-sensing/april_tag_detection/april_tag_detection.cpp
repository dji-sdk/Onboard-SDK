/*! @file camera_stream_poll_sample.cpp
 *  @version 3.6
 *  @date Feb 1st 2018
 *
 *  @brief
 *  AdvancedSensing, Camera Stream API usage in a Linux environment.
 *  This sample shows how to poll new images from FPV camera or/and
 *  main camera from the main thread.
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
#include <thread>
#include "dji_linux_helpers.hpp"
#include "AprilTag_TX2.h"

#ifdef OPEN_CV_INSTALLED
  #include "opencv2/opencv.hpp"
  #include "opencv2/highgui/highgui.hpp"
#endif

using namespace DJI::OSDK;
using namespace cv;
using namespace std;

AprilTag *landing_april_hanler;

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

void april_tag_process(CameraRGBImage main_img)
{
  // One processing for every 100
  static int count = 0;
  if (count < 100)
  {
    count++;
    return;
  }
  count = 0;

  cv::Mat mat(main_img.height, main_img.width, CV_8UC3, main_img.rawData.data(), main_img.width * 3);
  cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

  landing_april_hanler->loadImages(mat);
  if (!landing_april_hanler->pos_vec.empty())
  {
    for (unsigned int idx = 0; idx < landing_april_hanler->pos_vec.size(); idx++)
    {
      // Add a copy construction function of this struct
      std::cout << " ID: " << landing_april_hanler->pos_vec.at(idx).mark_id << std::endl;
      std::cout << " X: " << landing_april_hanler->pos_vec.at(idx).x;
      std::cout << " Y: " << landing_april_hanler->pos_vec.at(idx).y;
      std::cout << " Z: " << landing_april_hanler->pos_vec.at(idx).z << std::endl;
      std::cout << " Yaw: " << landing_april_hanler->pos_vec.at(idx).yaw;
      std::cout << " Pitch: " << landing_april_hanler->pos_vec.at(idx).pitch;
      std::cout << " Roll: " << landing_april_hanler->pos_vec.at(idx).roll << std::endl;
    }
  }

  if (landing_april_hanler->m_draw)
  {
    //cv::waitKey should be called here...
    std::string win_name = "Main";
    cv::namedWindow(win_name, 1);
    cv::imshow(win_name, mat);
    cv::waitKey(1);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

int main(int argc, char** argv)
{
  bool m = false;
  char c = 0;
  cout << "Please enter the type of camera stream you want to view\n"
       << "m: Main Camera\n"
       << "f: FPV  Camera\n" 
       << "b: both Cameras" << endl;
  cin >> c;

  // Setup April Tag handler
  landing_april_hanler = new AprilTag();
  if (landing_april_hanler != nullptr)
  {
    DSTATUS("April Tag processing init...\n");
    landing_april_hanler->parseOptions("/home/tao/jav_config/landmark.cfg");
    landing_april_hanler->setup();
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

  bool mainCamResult = false;

  mainCamResult = vehicle->advancedSensing->startMainCameraStream();
  if(!mainCamResult)
  {
    cout << "Failed to open Main Camera" << endl;
  }

  if(!mainCamResult)
  {
    cout << "Failed to Open Cameras, exiting" << endl;
    return 1;
  }

  CameraRGBImage mainImg;
  char mainName[] = "MAIN_CAM";
  for(int i=0; i<10000; i++)
  {
    if(mainCamResult && vehicle->advancedSensing->newMainCameraImageReady())
    {

      if(vehicle->advancedSensing->getMainCameraImage(mainImg))
      {
        //show_rgb(mainImg, mainName);
        april_tag_process(mainImg);
      }
      else
      {
        cout << "Time out" << endl;
      }
    }
    usleep(2e4);
  }

  vehicle->advancedSensing->stopMainCameraStream();
  return 0;
}
