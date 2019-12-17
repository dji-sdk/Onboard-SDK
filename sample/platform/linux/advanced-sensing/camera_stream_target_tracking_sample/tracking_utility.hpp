//
// Created by zyli on 12/11/17.
//

#ifndef ONBOARDSDK_TRACKINGUTILITY_HPP
#define ONBOARDSDK_TRACKINGUTILITY_HPP

#include "opencv2/opencv.hpp"
/*! @brief
 * The TrackingUtility Class handles simple start and stop tracking logic
 */
class TrackingUtility
{
public:
  TrackingUtility()
          : P1(0,0),
            P2(0,0),
            roi(0,0,0,0),
            mouseClicked(false),
            roiSelected(false),
            state(STATE_IDLE)
  {
  }

  typedef enum TrackingState
  {
    STATE_IDLE,
    STATE_INIT,
    STATE_ONGOING,
    STATE_STOP
  } TrackingState;

  static void mouseCallback(int event, int x, int y, int f, void *p);

  cv::Point P1;
  cv::Point P2;
  bool mouseClicked;
  cv::Rect roi;

  /*!
   * start_tracking is set true when you select a region and press 'g'
   * is set to false when you press 's'
   */
  bool roiSelected;
  TrackingState state;
  TrackingState getState();

  void startTracker();
  void stopTracker();

  cv::Rect getROI();
  void getKey(char c);
};


#endif //ONBOARDSDK_TRACKINGUTILITY_HPP
