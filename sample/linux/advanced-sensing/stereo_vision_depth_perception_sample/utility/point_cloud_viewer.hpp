#ifndef ONBOARDSDK_POINT_CLOUD_VIEWER_HPP
#define ONBOARDSDK_POINT_CLOUD_VIEWER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz/vizcore.hpp>

namespace M210_STEREO
{

class PointCloudViewer
{
public:
  ~PointCloudViewer();

  static void showPointCloud(cv::viz::WCloud &cloud);

  static inline void spinOnce() { PointCloudViewer::instancePtr()->viewer_.spinOnce(); };

  static PointCloudViewer& instance();

  static PointCloudViewer* instancePtr();

private:
  PointCloudViewer(const std::string &name);

private:
  static PointCloudViewer* single_instance_;

  cv::viz::Viz3d viewer_;

  cv::Point3d view_pos_;
  cv::Point3d view_point_;
  cv::Point3d view_y_dir_;

  cv::viz::WCoordinateSystem world_coordinate_;
};

} // namespace M210_STEREO

#endif //ONBOARDSDK_POINT_CLOUD_VIEWER_HPP
