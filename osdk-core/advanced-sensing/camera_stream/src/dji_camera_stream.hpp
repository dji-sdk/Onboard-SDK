/** @file dji_camera_stream.hpp
 *  @version 3.5
 *  @date Dec 2017
 *
 *  @brief The high level APIs to interact with the camera
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJICAMERASTREAM_H
#define DJICAMERASTREAM_H

#include <string>
#include "dji_camera_image.hpp"
class DJICameraStreamLink;
class DJICameraStreamDecoder;

class DJICameraStream
{
public:
  DJICameraStream(CameraType camType = FPV_CAMERA);
  ~DJICameraStream();

  bool newImageIsReady();

  /*!
   * @param timeoutMilliSec: 0 for infinity waiting
   * @return false if timeout, true if img obtained
   */

  bool getCurrentImage(CameraRGBImage& copyOfImage);

  bool startCameraStream(CameraImageCallback cb = NULL, void * cbParam = NULL);

  void stopCameraStream();

  bool startCameraH264(H264Callback cb = NULL, void * cbParam = NULL);

  void stopCameraH264();

private:
  DJICameraStreamLink     *rawDataStream;
  DJICameraStreamDecoder  *decoder;

  CameraType cameraType;
  std::string cameraNameStr;
  CameraRGBImage latestImage;
};

#endif // DJICAMERASTREAM_H
