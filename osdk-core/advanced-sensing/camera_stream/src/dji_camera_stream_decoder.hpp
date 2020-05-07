/** @file dji_camera_stream_decoder.hpp
 *  @version 3.5
 *  @date Dec 2017
 *
 *  @brief The class to decode the raw data from main camera into pictures

 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJICAMERASTREAMDECODER_HH
#define DJICAMERASTREAMDECODER_HH
extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include "pthread.h"
#include "dji_camera_image.hpp"
#include "dji_camera_image_handler.hpp"

class DJICameraStreamDecoder
{
public:
  DJICameraStreamDecoder();
  ~DJICameraStreamDecoder();
  bool init();
  void cleanup();

  bool getNewImage(CameraRGBImage & copyOfImage, int timeoutMilliSec);

  void callbackThreadFunc();

  void decodeBuffer(uint8_t* pBuf, int len);

  static void* callbackThreadEntry(void *p); 

  bool registerCallback(CameraImageCallback f, void* param);

  DJICameraImageHandler decodedImageHandler;

private:
  bool initSuccess;

  pthread_t callbackThread;
  bool      cbThreadIsRunning;
  int       cbThreadStatus;

  CameraImageCallback cb;
  void*               cbUserParam;

  AVCodecContext*       pCodecCtx;
  AVCodec*              pCodec;
  AVCodecParserContext* pCodecParserCtx;
  SwsContext*           pSwsCtx;

  AVFrame* pFrameYUV;
  AVFrame* pFrameRGB;
  uint8_t* rgbBuf;
  size_t   bufSize;
};

#endif // DJICAMERASTREAMDECODER_HH
