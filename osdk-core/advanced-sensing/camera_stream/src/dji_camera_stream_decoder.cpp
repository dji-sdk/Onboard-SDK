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
 * @file dji_camera_stream_decoder.cpp
 *  @version 3.5
 *  @date Dec 2017
 *
 */

#include "dji_camera_stream_decoder.hpp"
#include "dji_log.hpp"
#include "unistd.h"
#include "pthread.h"

DJICameraStreamDecoder::DJICameraStreamDecoder()
  : initSuccess(false),
    cbThreadIsRunning(false),
    cbThreadStatus(-1),
    cb(NULL),
    cbUserParam(NULL),
    pCodecCtx(NULL),
    pCodec(NULL),
    pCodecParserCtx(NULL),
    pSwsCtx(NULL),
    pFrameYUV(NULL),
    pFrameRGB(NULL),
    rgbBuf(NULL),
    bufSize(0)
{
}

DJICameraStreamDecoder::~DJICameraStreamDecoder()
{
  if(cb)
  {
    registerCallback(NULL, NULL);
  }

  cleanup();
}

bool DJICameraStreamDecoder::init()
{
  if(true == initSuccess)
  {
    DSTATUS_PRIVATE("Decoder already initialized.\n");
    return true;
  }

  avcodec_register_all();
  pCodecCtx = avcodec_alloc_context3(NULL);
  if (!pCodecCtx)
  {
    return false;
  }

  pCodecCtx->thread_count = 4;
  pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!pCodec || avcodec_open2(pCodecCtx, pCodec, NULL) < 0)
  {
    return false;
  }

  pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
  if (!pCodecParserCtx)
  {
    return false;
  }

  pFrameYUV = av_frame_alloc();
  if (!pFrameYUV)
  {
    return false;
  }

  pFrameRGB = av_frame_alloc();
  if (!pFrameRGB)
  {
    return false;
  }

  pSwsCtx = NULL;

  DSTATUS_PRIVATE("All components for decoding initialized ...\n");
  DDEBUG_PRIVATE("Decoder Version = %d\n", avcodec_version());

  pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;

  initSuccess = true;
  return true;
}

bool DJICameraStreamDecoder::getNewImage(CameraRGBImage & copyOfImage, int timeoutMilliSec)
{
  return decodedImageHandler.getNewImageWithLock(copyOfImage, timeoutMilliSec);
}

void DJICameraStreamDecoder::cleanup()
{
  initSuccess = false;
  if (NULL != pSwsCtx)
  {
    sws_freeContext(pSwsCtx);
    pSwsCtx = NULL;
  }

  if (NULL != pFrameYUV)
  {
    av_free(pFrameYUV);
    pFrameYUV = NULL;
  }

  if (NULL != pCodecParserCtx)
  {
    av_parser_close(pCodecParserCtx);
    pCodecParserCtx = NULL;
  }

  if (NULL != pCodec)
  {
    avcodec_close(pCodecCtx);
    pCodec = NULL;
  }

  if (NULL != pCodecCtx)
  {
    av_free(pCodecCtx);
    pCodecCtx = NULL;
  }

  if (NULL != rgbBuf )
  {
    av_free(rgbBuf);
    rgbBuf = NULL;
  }

  if (NULL != pFrameRGB)
  {
    av_free(pFrameRGB);
    pFrameRGB = NULL;
  }
}

void* DJICameraStreamDecoder::callbackThreadEntry(void* p)
{
  DSTATUS_PRIVATE("****** Decoder Callback Thread Start ******\n");
  static_cast<DJICameraStreamDecoder*>(p)->callbackThreadFunc();
  return NULL;
}

void DJICameraStreamDecoder::callbackThreadFunc()
{
  while(cbThreadIsRunning)
  {
    CameraRGBImage copyOfImage;
    if(!decodedImageHandler.getNewImageWithLock(copyOfImage, 1000))
    {
      DDEBUG_PRIVATE("Decoder Callback Thread: Get image time out\n");
      continue;
    }

    if(cb)
    {
      (*cb)(copyOfImage, cbUserParam);
    }
  }
  DSTATUS_PRIVATE("Decoder Callback Thread Stopped...\n");
}

void DJICameraStreamDecoder::decodeBuffer(uint8_t* buf, int bufLen)
{
  uint8_t* pData   = buf;
  int remainingLen = bufLen;
  int processedLen = 0;

  AVPacket pkt;
  av_init_packet(&pkt);
  while (remainingLen > 0)
  {
    processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                    &pkt.data, &pkt.size,
                                    pData, remainingLen,
                                    AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
    remainingLen -= processedLen;
    pData        += processedLen;

    if (pkt.size > 0)
    {
      int gotPicture = 0;
      avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

      if (!gotPicture)
      {
        //DSTATUS_PRIVATE("Got Frame, but no picture\n");
        continue;
      }
      else
      {
        int w = pFrameYUV->width;
        int h = pFrameYUV->height;
        //DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

        if(NULL == pSwsCtx)
        {
          pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                   w, h, AV_PIX_FMT_RGB24,
                                   4, NULL, NULL, NULL);
        }

        if(NULL == rgbBuf)
        {
          bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
          rgbBuf = (uint8_t*) av_malloc(bufSize);
          avpicture_fill((AVPicture*)pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
        }

        if(NULL != pSwsCtx && NULL != rgbBuf)
        {
          sws_scale(pSwsCtx,
                    (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                             pFrameRGB->data, pFrameRGB->linesize);

          pFrameRGB->height = h;
          pFrameRGB->width = w;

          decodedImageHandler.writeNewImageWithLock(pFrameRGB->data[0], bufSize, w, h);
        }
      }
    }
  }
  av_free_packet(&pkt);
}

bool DJICameraStreamDecoder::registerCallback(CameraImageCallback f, void *param)
{
  cb = f;
  cbUserParam = param;

  /* When users register a non-NULL callback, we will start the callback thread. */
  if(NULL != cb)
  {
    if(!cbThreadIsRunning)
    {
      cbThreadStatus = pthread_create(&callbackThread, NULL, callbackThreadEntry, this);
      if(0 == cbThreadStatus)
      {
        DSTATUS_PRIVATE("User callback thread created successfully!\n");
        cbThreadIsRunning = true;
        return true;
      }
      else
      {
        DERROR_PRIVATE("User called thread creation failed!\n");
        cbThreadIsRunning = false;
        return false;
      }
    }
    else
    {
      DERROR_PRIVATE("Callback thread already running!\n");
      return true;
    }
  }
  else
  {
    if(cbThreadStatus == 0)
    {
      cbThreadIsRunning = false;
      pthread_join(callbackThread, NULL);
      cbThreadStatus = -1;
    }
    return true;
  }
}

