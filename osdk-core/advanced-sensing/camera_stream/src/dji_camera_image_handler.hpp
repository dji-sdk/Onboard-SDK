/** @file dji_camera_image_handler.hpp
 *  @version 3.5
 *  @date Dec 2017
 *
 *  @brief Handle the writing and reading of decoded image
 *  among different threads
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJICAMERAIMAGEHANDLER_HH
#define DJICAMERAIMAGEHANDLER_HH

#include "pthread.h"
#include "dji_camera_image.hpp"

class DJICameraImageHandler
{
public:
  DJICameraImageHandler();
  ~DJICameraImageHandler();

  bool newImageIsReady();

  void writeNewImageWithLock(uint8_t* buf, int bufSize, int width, int height);
  bool getNewImageWithLock(CameraRGBImage & copyOfImage, int timeoutMilliSec);

private:
  pthread_mutex_t m_mutex;
  pthread_cond_t  m_condv;
  CameraRGBImage  m_img;
  bool            m_newImageFlag;
};

#endif
