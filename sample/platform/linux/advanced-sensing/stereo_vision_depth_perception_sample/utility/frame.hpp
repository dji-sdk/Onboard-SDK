#ifndef ONBOARDSDK_FRAME_H
#define ONBOARDSDK_FRAME_H


#include <memory>
#include <opencv2/opencv.hpp>

namespace M210_STEREO
{
  static const int VGA_HEIGHT = 480;
  static const int VGA_WIDTH  = 640;

struct Frame
{
  typedef std::shared_ptr<Frame> Ptr;

  Frame(uint64_t id, uint32_t time_stamp, cv::Mat img)
    : id(id)
    , time_stamp(time_stamp)
    , raw_image(img) {};

  static Frame::Ptr createFrame(uint64_t id, uint32_t time_stamp, cv::Mat img)
  {
    return std::make_shared<Frame>(id, time_stamp, img);
  }

  inline cv::Mat getImg() { return this->raw_image; }

  uint64_t  id;
  uint32_t  time_stamp;
  cv::Mat   raw_image;
};

} // namespace M210_STEREO

#endif //ONBOARDSDK_FRAME_H
