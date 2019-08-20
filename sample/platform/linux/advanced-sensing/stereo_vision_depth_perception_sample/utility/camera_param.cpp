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

#include "camera_param.hpp"

using namespace M210_STEREO;

CameraParam::CameraParam(uint8_t location)
  : location_(location)
{
  if(!initIntrinsicParam())
  {
    std::cerr << "Failed to init intrinsic parameter";
  }

  if(!initDistortionParam())
  {
    std::cerr << "Failed to init distortion parameter";
  }
}

CameraParam::~CameraParam()
{

}

CameraParam::Ptr CameraParam::createCameraParam(uint8_t position)
{
  return std::make_shared<CameraParam>(position);
}

bool
CameraParam::initDistortionParam()
{
  if(location_ == FRONT_LEFT)
  {
    param_distortion_ = Config::get<cv::Mat>("leftDistCoeffs");
  }
  else if(location_ == FRONT_RIGHT)
  {
    param_distortion_ = Config::get<cv::Mat>("rightDistCoeffs");
  }
  else
  {
    std::cerr << "Please specify the location of the camera\n";
    return false;
  }
  return true;
}

bool
CameraParam::initIntrinsicParam()
{
  if(location_ == FRONT_LEFT)
  {
    param_intrinsic_ = Config::get<cv::Mat>("leftCameraIntrinsicMatrix");
  }
  else if(location_ == FRONT_RIGHT)
  {
    param_intrinsic_ = Config::get<cv::Mat>("rightCameraIntrinsicMatrix");
  }
  else
  {
    std::cerr << "Please specify the location of the camera\n";
    return false;
  }
  return true;
}
