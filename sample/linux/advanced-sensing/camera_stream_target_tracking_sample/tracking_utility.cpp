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

#include "tracking_utility.hpp"
#include <iostream>
void TrackingUtility::mouseCallback(int event, int x, int y, int f, void *p)
{
  TrackingUtility *u = reinterpret_cast<TrackingUtility*>(p);
  if(u->state != STATE_IDLE)
  {
    std::cout << "Currently tracking, press s key to stop" << std::endl;
    return;
  }

  switch(event)
  {
    case  CV_EVENT_LBUTTONDOWN  :
      u->mouseClicked = true;
      u->roiSelected = false;
      u->P1 = cv::Point(x,y);
      u->P2 = cv::Point(x,y);
      break;

    case  CV_EVENT_LBUTTONUP    :
      u->P2 = cv::Point(x,y);
      u->mouseClicked=false;
      if(u->P2 != u->P1)
      {
        u->roiSelected = true;
      }
      break;

    case  CV_EVENT_MOUSEMOVE    :
      if(u->mouseClicked)
      {
        u->P2 = cv::Point(x,y);
      }
      break;

    default                     :
      break;
  }

  if(u->mouseClicked)
  {
    u->roi = cv::Rect(u->P1, u->P2);
    printf("Current Region of Interest: %d, %d, %d, %d\n", u->roi.tl().x, u->roi.tl().y, u->roi.br().x, u->roi.br().y);
  }
}

cv::Rect TrackingUtility::getROI()
{
  return roi;
}

TrackingUtility::TrackingState TrackingUtility::getState()
{
  return state;
}

void TrackingUtility::startTracker()
{
  state = STATE_ONGOING;
}

void TrackingUtility::stopTracker()
{
  state = STATE_IDLE;
}

void TrackingUtility::getKey(char c)
{
  switch(c)
  {
    case 'g':
      if( (state == STATE_IDLE) && (roiSelected == true))
      {
        state = STATE_INIT;
      }
      break;

    case 's':
      if( state == STATE_ONGOING )
      {
        state = STATE_STOP;
        roi = cv::Rect(0,0,0,0); //when we press s, should clear bounding box
      }
      break;

    default:
      break;
  }
}