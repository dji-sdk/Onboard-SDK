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

#include "stereo_frame.hpp"

using namespace M210_STEREO;
using namespace cv;

StereoFrame::StereoFrame(CameraParam::Ptr left_cam,
                         CameraParam::Ptr right_cam,
                         int num_disp, int block_size)
  : camera_left_ptr_(left_cam)
  , camera_right_ptr_(right_cam)
  , num_disp_(num_disp)
  , block_size_(block_size)
  , color_buffer_(VGA_WIDTH*VGA_HEIGHT)
  , mat_vec3_pt_(VGA_HEIGHT, VGA_WIDTH, Vec3f(0, 0, 0))
  , color_mat_(VGA_HEIGHT, VGA_WIDTH, CV_8UC1, &color_buffer_[0])
  , pt_cloud_(mat_vec3_pt_, color_mat_)
  , raw_disparity_map_(Mat(VGA_HEIGHT, VGA_WIDTH, CV_16SC1))
{
  if(!this->initStereoParam())
  {
    DERROR("Failed to init stereo parameters\n");
  }

  Mat m210_vga_stereo_left  = Mat(VGA_HEIGHT,VGA_WIDTH, CV_8U);
  Mat m210_vga_stereo_right = Mat(VGA_HEIGHT,VGA_WIDTH, CV_8U);

  frame_left_ptr_   = Frame::createFrame(0, 0, m210_vga_stereo_left);
  frame_right_ptr_  = Frame::createFrame(0, 0, m210_vga_stereo_right);
}

StereoFrame::~StereoFrame()
{

}

bool
StereoFrame::initStereoParam()
{
  param_rect_left_ =  Config::get<Mat>("leftRectificationMatrix");
  param_rect_right_ = Config::get<Mat>("rightRectificationMatrix");
  param_proj_left_ =  Config::get<Mat>("leftProjectionMatrix");
  param_proj_right_ = Config::get<Mat>("rightProjectionMatrix");

  principal_x_ = param_proj_left_.at<double>(0, 2);
  principal_y_ = param_proj_left_.at<double>(1, 2);
  fx_ = param_proj_left_.at<double>(0, 0);
  fy_ = param_proj_left_.at<double>(1, 1);
  baseline_x_fx_ = -param_proj_right_.at<double>(0, 3);

  initUndistortRectifyMap(camera_left_ptr_->getIntrinsic(),
                              camera_left_ptr_->getDistortion(),
                              param_rect_left_,
                              param_proj_left_,
                              Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                              rectified_mapping_[0][0], rectified_mapping_[0][1]);
  initUndistortRectifyMap(camera_right_ptr_->getIntrinsic(),
                              camera_right_ptr_->getDistortion(),
                              param_rect_right_,
                              param_proj_right_,
                              Size(VGA_WIDTH, VGA_HEIGHT), CV_32F,
                              rectified_mapping_[1][0], rectified_mapping_[1][1]);

#ifdef USE_GPU
  for (int k = 0; k < 2; ++k) {
    for (int i = 0; i < 2; ++i) {
      cuda_rectified_mapping_[k][i].upload(rectified_mapping_[k][i]);
    }
  }

  block_matcher_ = cuda::createStereoBM(num_disp_, block_size_);
#else
  block_matcher_ = StereoBM::create(num_disp_, block_size_);
#endif

#ifdef USE_OPEN_CV_CONTRIB
  wls_filter_ = ximgproc::createDisparityWLSFilter(block_matcher_); // left_matcher
  wls_filter_->setLambda(8000.0);
  wls_filter_->setSigmaColor(1.5);

  right_matcher_ = ximgproc::createRightMatcher(block_matcher_);
#endif

  return true;
}

StereoFrame::Ptr
StereoFrame::createStereoFrame(CameraParam::Ptr left_cam, CameraParam::Ptr right_cam)
{
  return std::make_shared<StereoFrame>(left_cam, right_cam);
}

void
StereoFrame::readStereoImgs(const DJI::OSDK::ACK::StereoVGAImgData &imgs)
{
  memcpy(frame_left_ptr_->raw_image.data,
         imgs.img_vec[0], sizeof(char)*VGA_HEIGHT*VGA_WIDTH);
  memcpy(frame_right_ptr_->raw_image.data,
         imgs.img_vec[1], sizeof(char)*VGA_HEIGHT*VGA_WIDTH);

  frame_left_ptr_-> id  = imgs.frame_index;
  frame_right_ptr_->id  = imgs.frame_index;
  frame_left_ptr_-> time_stamp  = imgs.time_stamp;
  frame_right_ptr_->time_stamp  = imgs.time_stamp;
}

void
StereoFrame::rectifyImgs()
{
#ifdef USE_GPU
  cuda_rect_src.upload(frame_left_ptr_->getImg());
  cuda::remap(cuda_rect_src, cuda_rectified_img_left_,
                  cuda_rectified_mapping_[0][0],
                  cuda_rectified_mapping_[0][1],
                  INTER_LINEAR);
  cuda_rectified_img_left_.download(rectified_img_left_);

  cuda_rect_src.upload(frame_right_ptr_->getImg());
  cuda::remap(cuda_rect_src, cuda_rectified_img_right_,
                  cuda_rectified_mapping_[1][0],
                  cuda_rectified_mapping_[1][1],
                  INTER_LINEAR);
  cuda_rectified_img_right_.download(rectified_img_right_);
#else
  remap(frame_left_ptr_->getImg(), rectified_img_left_,
            rectified_mapping_[0][0], rectified_mapping_[0][1], INTER_LINEAR);
  remap(frame_right_ptr_->getImg(), rectified_img_right_,
            rectified_mapping_[1][0], rectified_mapping_[1][1], INTER_LINEAR);
#endif
}

void
StereoFrame::computeDisparityMap()
{

#ifdef USE_GPU
  cuda::GpuMat cuda_disp_left;

  // GPU implementation of stereoBM outputs uint8_t, i.e. CV_8U
  block_matcher_->compute(cuda_rectified_img_left_.clone(),
                          cuda_rectified_img_right_.clone(),
                          cuda_disp_left);

  cuda_disp_left.download(raw_disparity_map_);

  raw_disparity_map_.convertTo(disparity_map_8u_, CV_8UC1, 1);

  // convert it from CV_8U to CV_16U for unified
  // calculation in filterDisparityMap() & unprojectPtCloud()
  raw_disparity_map_.convertTo(raw_disparity_map_, CV_16S, 16);
#else
  // CPU implementation of stereoBM outputs short int, i.e. CV_16S
  block_matcher_->compute(rectified_img_left_, rectified_img_right_, raw_disparity_map_);

  raw_disparity_map_.convertTo(disparity_map_8u_, CV_8UC1, 0.0625);
#endif

}

void
StereoFrame::filterDisparityMap()
{
#ifdef USE_OPEN_CV_CONTRIB
  right_matcher_->compute(rectified_img_right_, rectified_img_left_, raw_right_disparity_map_);

  // Only takes CV_16S type cv::Mat
  wls_filter_->filter(raw_disparity_map_,
                      rectified_img_left_,
                      filtered_disparity_map_,
                      raw_right_disparity_map_);

  filtered_disparity_map_.convertTo(filtered_disparity_map_8u_, CV_8UC1, 0.0625);
#endif
}

void
StereoFrame::unprojectPtCloud()
{
  // due to rectification, the image boarder are blank
  // we cut them out
  const int border_size = num_disp_;
  const int trunc_img_width_end = VGA_WIDTH - border_size;
  const int trunc_img_height_end = VGA_HEIGHT - border_size;

  mat_vec3_pt_ = Mat_<Vec3f>(VGA_HEIGHT, VGA_WIDTH, Vec3f(0, 0, 0));

  for(int v = border_size; v < trunc_img_height_end; ++v)
  {
    for(int u = border_size; u < trunc_img_width_end; ++u)
    {
      Vec3f &point = mat_vec3_pt_.at<Vec3f>(v, u);

#ifdef USE_OPEN_CV_CONTRIB
      float disparity = (float)(filtered_disparity_map_.at<short int>(v, u)*0.0625);
#else
      float disparity = (float)(raw_disparity_map_.at<short int>(v, u)*0.0625);
#endif

      // do not consider pts that are farther than 8.6m, i.e. disparity < 6
      if(disparity >= 6)
      {
        point[2] = baseline_x_fx_/disparity;
        point[0] = (u-principal_x_)*point[2]/fx_;
        point[1] = (v-principal_y_)*point[2]/fy_;
      }
      color_buffer_[v*VGA_WIDTH+u] = rectified_img_left_.at<uint8_t>(v, u);
    }
  }

  color_mat_ = cv::Mat(VGA_HEIGHT, VGA_WIDTH, CV_8UC1, &color_buffer_[0]).clone();

  // @note Unfortunately, calling this WCloud constructor costs about the same amount
  // of time as we go through each pixel and unproject the pt cloud. Because there's
  // another nested for-loop inside WCloud implementation
  // Ideally these can be done in one shot but it involves changing openCV implementation
  // TODO maybe opencv projectPoints() is a good alternative
  pt_cloud_ = viz::WCloud(mat_vec3_pt_, color_mat_);
}