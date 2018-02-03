#ifndef ONBOARDSDK_CAMERA_H
#define ONBOARDSDK_CAMERA_H

#include <opencv2/opencv.hpp>
#include <memory>
#include "config.hpp"

namespace M210_STEREO
{

class CameraParam
{
public:
  enum POSITION
  {
    FRONT_LEFT  = 0,
    FRONT_RIGHT = 1,
    DOWN_LEFT   = 2,
    DOWN_RIGHT  = 3
  };

  typedef std::shared_ptr<CameraParam> Ptr;

  CameraParam(uint8_t position);
  ~CameraParam();

public:
  static CameraParam::Ptr createCameraParam(uint8_t position);

  inline cv::Mat getIntrinsic() { return this->param_intrinsic_; }

  inline cv::Mat getDistortion() { return this->param_distortion_; }

  inline double getPrincipalX() { return this->param_intrinsic_.at<double>(0, 2); }

  inline double getPrincipalY() { return this->param_intrinsic_.at<double>(1, 2); }

  inline double getFocalX() { return this->param_intrinsic_.at<double>(0, 0); }

  inline double getFocalY() { return this->param_intrinsic_.at<double>(1, 1); }

protected:
  bool initIntrinsicParam();

  bool initDistortionParam();

protected:
  cv::Mat param_intrinsic_;
  cv::Mat param_distortion_;
  float   depth_scale_;
  uint8_t location_;
};

} // namespace M210_STEREO



#endif //ONBOARDSDK_CAMERA_H
