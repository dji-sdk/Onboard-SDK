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

#include "point_cloud_viewer.hpp"

using namespace M210_STEREO;

PointCloudViewer* PointCloudViewer::single_instance_ = new PointCloudViewer("Point Cloud Viewer");

PointCloudViewer::PointCloudViewer(const std::string &name)
  : viewer_(name)
  , view_pos_(-1.0, -2.0, -3.0)
  , view_point_(0, 0, 0)
  , view_y_dir_(0, 1, 0)
  , world_coordinate_(.3)
{
  cv::Affine3d view_pose = cv::viz::makeCameraPose(view_pos_,
                                                   view_point_,
                                                   view_y_dir_);
  viewer_.setViewerPose(view_pose);

  world_coordinate_.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);

  viewer_.showWidget("World", world_coordinate_);
  cv::Affine3d M = cv::Affine3d::Identity();
  viewer_.setWidgetPose("World", M );
}

PointCloudViewer::~PointCloudViewer()
{

}

PointCloudViewer&
PointCloudViewer::instance()
{
  return *PointCloudViewer::single_instance_;
}

PointCloudViewer*
PointCloudViewer::instancePtr()
{
  return PointCloudViewer::single_instance_;
}

void
PointCloudViewer::showPointCloud(cv::viz::WCloud &cloud)
{
  PointCloudViewer *this_ptr = PointCloudViewer::instancePtr();

  cloud.setRenderingProperty( cv::viz::POINT_SIZE, 3);
  this_ptr->viewer_.showWidget("Cloud", cloud);
}