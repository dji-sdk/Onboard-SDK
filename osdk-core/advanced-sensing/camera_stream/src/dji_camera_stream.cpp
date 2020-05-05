/*
 * DJI Onboard SDK Advanced Sensing APIs
 *
 * Copyright (c) 2017-2018 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 * @file dji_camera_stream.cpp
 *  @version 3.5
 *  @date Dec 2017
 *
 */

#include "dji_camera_stream.hpp"
#include "dji_camera_stream_link.hpp"
#include "dji_camera_stream_decoder.hpp"

#include "dji_log.hpp"

void decodeStream(void* cbParam, uint8_t* buf, int len)
{
  DJICameraStreamDecoder *d = reinterpret_cast<DJICameraStreamDecoder*>(cbParam);
  d->decodeBuffer(buf, len);
}

DJICameraStream::DJICameraStream(CameraType camType) :
        cameraType(camType)
{
  cameraNameStr = (camType == FPV_CAMERA) ? std::string("FPV_CAMERA") : std::string("MAIN_CAMERA");
  rawDataStream = new DJICameraStreamLink(camType);
  decoder       = new DJICameraStreamDecoder;
}

DJICameraStream::~DJICameraStream()
{
  if(rawDataStream)
  {
    delete rawDataStream;
  }

  if(decoder)
  {
    delete decoder;
  }
}

bool DJICameraStream::startCameraStream(CameraImageCallback cb, void* cbParam)
{
  if(!rawDataStream->init())
  {
    DERROR_PRIVATE("Initialize %s failed\nDouble check USB connection or re-plug in USB cable.\n",  cameraNameStr.c_str());
    return false;
  }

  if(!decoder->init())
  {
    DERROR_PRIVATE("There is issue with decoder for %s, please restart the program.\n", cameraNameStr.c_str());
    return false;
  }

  /*!
   * 1. Register callback (decoding) when raw data is received.
   * 2. Start udt thread: it'll start read raw data and call decoding function
   * 3. Register decoder callback by user. It'll start callback thread.
   */
  rawDataStream->registerCallback(&decodeStream, decoder);

  if(!rawDataStream->start())
  {
    return false;
  }

  /*! 
   * Callback registered by user.
   * Run when a new image is available.
   */
  if(!decoder->registerCallback(cb, cbParam))
  {
    return false;
  }

  return true;
}

void DJICameraStream::stopCameraStream()
{
  decoder->registerCallback(NULL, NULL);
  rawDataStream->registerCallback(NULL, NULL);
  rawDataStream->cleanup();
  decoder->cleanup();
}

bool DJICameraStream::getCurrentImage(CameraRGBImage& copyOfImage)
{
  return decoder->decodedImageHandler.getNewImageWithLock(copyOfImage, 20);
}

bool DJICameraStream::newImageIsReady()
{
  return decoder->decodedImageHandler.newImageIsReady();
}

typedef struct H264CbHandler{
  H264Callback cb;
  void* userData;
} H264CbHandler;

void exportH264Cb(void* cbParam, uint8_t* buf, int len)
{
  H264CbHandler *handler = (H264CbHandler *)cbParam;
  handler->cb(buf, len, handler->userData);
}

bool DJICameraStream::startCameraH264(H264Callback cb, void* cbParam)
{
  static H264CbHandler handler;

  if(!rawDataStream->init())
  {
    DERROR_PRIVATE("Initialize %s failed\nDouble check USB connection or re-plug in USB cable.\n",  cameraNameStr.c_str());
    return false;
  }

  if(!decoder->init())
  {
    DERROR_PRIVATE("There is issue with decoder for %s, please restart the program.\n", cameraNameStr.c_str());
    return false;
  }

  handler = {cb, cbParam};

  rawDataStream->registerCallback(&exportH264Cb, (void *)&handler);

  if(!rawDataStream->start())
  {
    return false;
  }

  return true;
}

void DJICameraStream::stopCameraH264()
{
  decoder->registerCallback(NULL, NULL);
  rawDataStream->registerCallback(NULL, NULL);
  rawDataStream->cleanup();
  decoder->cleanup();
}

