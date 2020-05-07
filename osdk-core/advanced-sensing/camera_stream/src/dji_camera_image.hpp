/** @file dji_camera_image.cpp
 *  @version 3.5
 *  @date Dec 2017
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef ADVANCED_SENSING_DJI_CAMERA_IMAGE_HPP
#define ADVANCED_SENSING_DJI_CAMERA_IMAGE_HPP
#include <cstdint>
#include <vector>

/*! @brief Data structure for the image frames from the
 *         FPV camera or main camera
 */
struct CameraRGBImage
{
  // rawData.size should be height x width x 3 x sizeof(char)
  std::vector<uint8_t> rawData;
  int height;
  int width;
};

/*! @brief User callback function called by OSDK (in a dedicated thread)
 *  when a new image frame from camera is received.
 */
typedef void (*CameraImageCallback)(CameraRGBImage pImg, void* userData);

/*! @brief User callback function called by OSDK (in a dedicated thread)
 *  when a H264 frame is received.
 */
typedef void (*H264Callback)(uint8_t* buf, int bufLen, void* userData);

/*! @brief Data structure for the image frames from the
 *         FPV camera or main camera
 */
enum CameraType
{
  FPV_CAMERA  = 0,
  MAIN_CAMERA = 1
};

#endif //ADVANCED_SENSING_DJI_CAMERA_IMAGE_HPP
