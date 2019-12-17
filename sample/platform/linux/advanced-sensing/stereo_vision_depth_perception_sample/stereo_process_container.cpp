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

#include "stereo_process_container.hpp"

using namespace M210_STEREO;

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

StereoProcessContainer::StereoProcessContainer(DJI::OSDK::Vehicle *vehicle)
  : ImageProcessContainer(vehicle)
  , is_disp_filterd(false)
{
  if(!this->initStereoFrame())
  {
    DERROR("Failed to init camera parameter and stereo frame\n");
  }
}

StereoProcessContainer::~StereoProcessContainer()
{

}

bool
StereoProcessContainer::initStereoFrame()
{
  camera_left_ptr   = CameraParam::createCameraParam(CameraParam::FRONT_LEFT);
  camera_right_ptr  = CameraParam::createCameraParam(CameraParam::FRONT_RIGHT);

  stereo_frame_ptr = StereoFrame::createStereoFrame(camera_left_ptr, camera_right_ptr);

  return true;
}

void
StereoProcessContainer::displayStereoRectImgCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                                     DJI::OSDK::UserData userData)
{
  StereoProcessContainer* this_ptr = (StereoProcessContainer*)userData;

  //! Read raw images
  this_ptr->stereo_frame_ptr->readStereoImgs(this_ptr->stereoVGAImg);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  this_ptr->visualizeRectImgHelper();

  cv::waitKey(1);

  duration rectify_time_diff = rectify_end - rectify_start;
  DSTATUS("This stereo frame takes %.2f ms to rectify", rectify_time_diff.count()*1000.0);
}

void
StereoProcessContainer::displayStereoDisparityCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                                       DJI::OSDK::UserData userData)
{
  StereoProcessContainer* this_ptr = (StereoProcessContainer*)userData;

  //! Read raw images
  this_ptr->stereo_frame_ptr->readStereoImgs(this_ptr->stereoVGAImg);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  //! Compute disparity
  timer disp_start    = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->computeDisparityMap();
  timer disp_end      = std::chrono::high_resolution_clock::now();

  this_ptr->visualizeRectImgHelper();

  this_ptr->visualizeDisparityMapHelper();

  cv::waitKey(1);

  duration rectify_time_diff = rectify_end - rectify_start;
  duration disp_time_diff = disp_end - disp_start;
  DSTATUS("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity",
          rectify_time_diff.count()*1000.0,
          disp_time_diff.count()*1000.0);
}

void
StereoProcessContainer::displayStereoFilteredDisparityCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                                               DJI::OSDK::UserData userData)
{
#ifdef USE_OPEN_CV_CONTRIB

  StereoProcessContainer* this_ptr = (StereoProcessContainer*)userData;

  //! Read raw images
  this_ptr->stereo_frame_ptr->readStereoImgs(this_ptr->stereoVGAImg);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  //! Compute disparity
  timer disp_start    = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->computeDisparityMap();
  timer disp_end      = std::chrono::high_resolution_clock::now();

  //! Filter disparity map
  timer filter_start = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->filterDisparityMap();
  this_ptr->is_disp_filterd = true;
  timer filter_end  = std::chrono::high_resolution_clock::now();

  this_ptr->visualizeRectImgHelper();

  this_ptr->visualizeDisparityMapHelper();

  cv::waitKey(1);

  duration rectify_time_diff = rectify_end - rectify_start;
  duration disp_time_diff = disp_end - disp_start;
  duration filter_diff = filter_end - filter_start;
  DSTATUS("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity, "
            "%.2f ms to filter",
          rectify_time_diff.count()*1000.0,
          disp_time_diff.count()*1000.0,
          filter_diff.count()*1000.0);
#else
  DSTATUS("OpenCV ximgproc module is not found. It's required for disparity map filtering");
#endif

}

void
StereoProcessContainer::displayStereoPtCloudCallback(DJI::OSDK::Vehicle *vehiclePtr,
                                                     DJI::OSDK::UserData userData)
{
  StereoProcessContainer* this_ptr = (StereoProcessContainer*)userData;

  //! Read raw images
  this_ptr->stereo_frame_ptr->readStereoImgs(this_ptr->stereoVGAImg);

  //! Rectify images
  timer rectify_start = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->rectifyImgs();
  timer rectify_end   = std::chrono::high_resolution_clock::now();

  //! Compute disparity
  timer disp_start    = std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->computeDisparityMap();
  timer disp_end      = std::chrono::high_resolution_clock::now();

  //! Filter disparity map
  timer filter_start= std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->filterDisparityMap();
  this_ptr->is_disp_filterd = true;
  timer filter_end  = std::chrono::high_resolution_clock::now();

  //! Unproject image to 3D point cloud
  timer pt_cloud_start= std::chrono::high_resolution_clock::now();
  this_ptr->stereo_frame_ptr->unprojectPtCloud();
  timer pt_cloud_end  = std::chrono::high_resolution_clock::now();

  cv::viz::WCloud pt_cloud = this_ptr->stereo_frame_ptr->getPtCloud();
  PointCloudViewer::showPointCloud(pt_cloud);
  PointCloudViewer::spinOnce();

  this_ptr->visualizeRectImgHelper();

  this_ptr->visualizeDisparityMapHelper();

  cv::waitKey(1);

  duration rectify_time_diff = rectify_end - rectify_start;
  duration disp_time_diff = disp_end - disp_start;
  duration filter_diff = filter_end - filter_start;
  duration pt_cloud_diff = pt_cloud_end - pt_cloud_start;
  DSTATUS("This stereo frame takes %.2f ms to rectify, %.2f ms to compute disparity, "
            "%.2f ms to filter, %.2f ms to unproject point cloud",
          rectify_time_diff.count()*1000.0,
          disp_time_diff.count()*1000.0,
          filter_diff.count()*1000.0,
          pt_cloud_diff.count()*1000.0);
}

void
StereoProcessContainer::visualizeRectImgHelper()
{
  cv::Mat img_to_show;

  cv::hconcat(stereo_frame_ptr->getRectLeftImg(),
              stereo_frame_ptr->getRectRightImg(),
              img_to_show);

  cv::resize(img_to_show, img_to_show,
             cv::Size(VGA_WIDTH*2, VGA_HEIGHT),
             (0, 0), (0, 0), cv::INTER_LINEAR);

  // draw epipolar lines to visualize rectification
  for(int j = 0; j < img_to_show.rows; j += 24 ){
    line(img_to_show, cv::Point(0, j),
         cv::Point(img_to_show.cols, j),
         cv::Scalar(255, 0, 0, 255), 1, 8);
  }

  cv::imshow("Rectified Stereo Imgs with epipolar lines", img_to_show);
}

void
StereoProcessContainer::visualizeDisparityMapHelper()
{
  cv::Mat raw_disp_map;
#ifdef USE_OPEN_CV_CONTRIB
  if(is_disp_filterd) {
    raw_disp_map = stereo_frame_ptr->getFilteredDispMap().clone();
  } else {
    raw_disp_map = stereo_frame_ptr->getDisparityMap().clone();
  }
#else
  raw_disp_map = stereo_frame_ptr->getDisparityMap().clone();
#endif

  double min_val, max_val;
  cv::minMaxLoc(raw_disp_map, &min_val, &max_val, NULL, NULL);

  cv::Mat scaled_disp_map;
  raw_disp_map.convertTo(scaled_disp_map, CV_8U, 255/(max_val-min_val), -min_val/(max_val-min_val));

  cv::imshow("Scaled disparity map", scaled_disp_map);
}