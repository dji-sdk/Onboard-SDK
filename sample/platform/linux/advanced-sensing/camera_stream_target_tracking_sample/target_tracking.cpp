/*
 * @Copyright (c) 2017 DJI
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
 */

#include "dji_vehicle.hpp"
#include <iostream>
#include "dji_linux_helpers.hpp"
#include "tracking_utility.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "kcftracker.hpp"

#include <cstdio>
#include <chrono>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

using namespace DJI::OSDK;
using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  // Setup OSDK.
  bool enableAdvancedSensing = true;
  LinuxSetup linuxEnvironment(argc, argv, enableAdvancedSensing);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  const char *acm_dev = linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
  vehicle->advancedSensing->setAcmDevicePath(acm_dev);
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Initialize variables
  int functionTimeout = 1;
  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  bool mainCamResult = vehicle->advancedSensing->startMainCameraStream();
  if(!mainCamResult)
  {
    cout << "Failed to Open Camera" << endl;
    return 1;
  }

  CameraRGBImage mainImg;
  const char winName[]="My Camera";
  char message1[100];
  char message2[100];
  Rect roi(0,0,0,0);

  KCFTracker *tracker = NULL;
  TrackingUtility tu;

  cv::namedWindow(winName,1);
  cv::setMouseCallback(winName,TrackingUtility::mouseCallback, (void*)&tu);

  while(1)
  {
    char c = cv::waitKey(10);
    if(c==27)
    {
      if(tracker != NULL)
      {
        delete tracker;
        tracker = NULL;
      }
      break; // Quit if ESC is pressed
    }

    tu.getKey(c); //Internal states will be updated based on key pressed.

    if(vehicle->advancedSensing->newMainCameraImageReady())
    {
      int dx = 0;
      int dy = 0;
      int yawRate = 0;
      int pitchRate = 0;
      timer trackerStartTime, trackerFinishTime;
      duration trackerTimeDiff;

      vehicle->advancedSensing->getMainCameraImage(mainImg);
      Mat frame(mainImg.height, mainImg.width, CV_8UC3, mainImg.rawData.data(), mainImg.width*3);


      switch(tu.getState())
      {
      case TrackingUtility::STATE_IDLE:
        roi = tu.getROI();
        sprintf(message2, "Please select ROI and press g");
        break;

      case TrackingUtility::STATE_INIT:
        cout << "g pressed, initialize tracker" << endl;
        sprintf(message2, "g pressed, initialize tracker");
        roi = tu.getROI();
        tracker = new KCFTracker(true, true, false, false);
        tracker->init(roi, frame);
        tu.startTracker();
        break;

      case TrackingUtility::STATE_ONGOING:
        trackerStartTime  = std::chrono::high_resolution_clock::now();
        roi = tracker->update(frame);
        trackerFinishTime = std::chrono::high_resolution_clock::now();
        trackerTimeDiff = trackerFinishTime - trackerStartTime;
        sprintf(message2, "Tracking: bounding box update time = %.2f ms\n", trackerTimeDiff.count()*1000.0);

        // send gimbal speed command
        dx = (int)(roi.x + roi.width/2  - mainImg.width/2);
        dy = (int)(roi.y + roi.height/2 - mainImg.height/2);

        yawRate   = dx;
        pitchRate = -dy;

        if(abs(dx) < 10)
        {
          yawRate = 0;
        }

        if(abs(dy) < 10)
        {
          pitchRate = 0;
        }

        DJI::OSDK::Gimbal::SpeedData gimbalSpeed;
        gimbalSpeed.roll     = 0;
        gimbalSpeed.pitch    = pitchRate;
        gimbalSpeed.yaw      = yawRate;
        gimbalSpeed.gimbal_control_authority = 1;

        vehicle->gimbal->setSpeed(&gimbalSpeed);

        break;

      case TrackingUtility::STATE_STOP:
        cout << "s pressed, stop tracker" << endl;
        sprintf(message2, "s pressed, stop tracker");
        delete tracker;
        tracker = NULL;
        tu.stopTracker();
        roi = tu.getROI();
        break;

      default:
        break;
      }

      dx = roi.x + roi.width/2  - mainImg.width/2;
      dy = roi.y + roi.height/2 - mainImg.height/2;

      cv::circle(frame, Point(mainImg.width/2, mainImg.height/2), 5, cv::Scalar(255,0,0), 2, 8);
      if(roi.width != 0)
      {
        cv::circle(frame, Point(roi.x + roi.width/2, roi.y + roi.height/2), 3, cv::Scalar(0,0,255), 1, 8);

        cv::line(frame,  Point(mainImg.width/2, mainImg.height/2),
                 Point(roi.x + roi.width/2, roi.y + roi.height/2),
                 cv::Scalar(0,255,255));
      }

      cv::rectangle(frame, roi, cv::Scalar(0,255,0), 1, 8, 0 );
      cvtColor(frame, frame, COLOR_RGB2BGR);
      sprintf(message1,"dx=%04d, dy=%04d",dx, dy);
      putText(frame, message1, Point2f(20,30), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
      putText(frame, message2, Point2f(20,60), FONT_HERSHEY_SIMPLEX, 1,  Scalar(0,255,0));
      cv::imshow(winName, frame);

    }
  }

  vehicle->advancedSensing->stopMainCameraStream();

  if(tracker)
  {
    delete tracker;
  }

  return 0;
}
