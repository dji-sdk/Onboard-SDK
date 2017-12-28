/*! @file image_process_container.cpp
 *  @version 3.4
 *  @date Oct 10 2017
 *
 *  @brief
 *  A simple class to allow user to copy images
 *  from reading thread and process them in
 *  a separate image_processing thread
 *
 *  @Copyright (c) 2017 DJI
 *
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

#include "image_process_container.hpp"

using namespace DJI::OSDK;

ImageProcessContainer::ImageProcessContainer(Vehicle *vehicle)
  : vehicle_ptr(vehicle)
  , gotNew240pImg(false)
  , gotNewVGAImg(false)
{
  if(!this->initImgProcessThread())
  {
    DERROR("Failed to initialize image process thread.\n");
  }
}

ImageProcessContainer::~ImageProcessContainer()
{
  this->imgProcessThread->stopThread();
}

bool
ImageProcessContainer::initImgProcessThread()
{
  this->imgProcessThread = new UtilityThread(this, IMG_PROCESS_THREAD);

  if(this->imgProcessThread == 0)
  {
    DERROR("Error instantiating image process thread object\n");
    return false;
  }

  bool imgProcessThreadStatus = false;
  imgProcessThreadStatus = this->imgProcessThread->createThread();
  if (!imgProcessThreadStatus)
  {
    DERROR("Error creating image process thread!\n");
  }

  return imgProcessThreadStatus;
}

void
ImageProcessContainer::copyVGAImg(const ACK::StereoVGAImgData *img)
{
  memcpy(&stereoVGAImg, img, sizeof(ACK::StereoVGAImgData));
  this->gotNewVGAImg = true;
}

void
ImageProcessContainer::copy240pImg(const ACK::StereoImgData *img)
{
  memcpy(&stereo240pImg, img, sizeof(ACK::StereoImgData));
  this->gotNew240pImg = true;
}


void
ImageProcessContainer::displayStereoVGACallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::UserData userData)
{
  ImageProcessContainer* img_container_ptr = (ImageProcessContainer*)userData;

  DSTATUS("you got a set of VGA image frame in %s thread!",
          img_container_ptr->imgProcessThread->getThreadName().c_str());

#ifdef OPEN_CV_INSTALLED
  // VGA always comes in pair
  cv::Mat cv_img_stereo[2];
  for (int i = 0; i < 2; ++i) {
    cv_img_stereo[i] = cv::Mat(480, 640, CV_8U);
  }

  memcpy(cv_img_stereo[0].data, img_container_ptr->stereoVGAImg.img_vec[0], sizeof(char)*480*640);
  memcpy(cv_img_stereo[1].data, img_container_ptr->stereoVGAImg.img_vec[1], sizeof(char)*480*640);

  cv::Mat img_to_show;
  cv::hconcat(cv_img_stereo[0], cv_img_stereo[1], img_to_show);
  cv::resize(img_to_show, img_to_show,
             cv::Size(640*2, 480), (0, 0), (0, 0), cv::INTER_LINEAR);

  cv::imshow("cv_img_stereo", img_to_show);
  cv::waitKey(1);
#endif
}

void
ImageProcessContainer::displayStereo240pCallback(DJI::OSDK::Vehicle *vehiclePtr, DJI::OSDK::UserData userData)
{
  ImageProcessContainer* img_container_ptr = (ImageProcessContainer*)userData;

  DSTATUS("you got 240p images in %s thread!",
          img_container_ptr->imgProcessThread->getThreadName().c_str());

#ifdef OPEN_CV_INSTALLED
  cv::Mat cv_img_stereo[img_container_ptr->stereo240pImg.num_imgs];

  for (int i = 0; i < img_container_ptr->stereo240pImg.num_imgs; ++i)
  {
    cv_img_stereo[i] = cv::Mat(240, 320, CV_8U);

    memcpy(cv_img_stereo[i].data, img_container_ptr->stereo240pImg.img_vec[i].image,
           sizeof(char)*ACK::IMG_240P_SIZE);

    cv::imshow(img_container_ptr->stereo240pImg.img_vec[i].name,
               cv_img_stereo[i]);
    cv::waitKey(1);
  }
#endif
}

UtilityThread*
ImageProcessContainer::getImgProcessThread()
{
  return this->imgProcessThread;
}

void
ImageProcessContainer::setCallbackHandler(ImgCallBack callback, DJI::OSDK::UserData userData)
{
  this->imgCallBackHandler.callback = callback;
  this->imgCallBackHandler.userData = userData;
}

bool
ImageProcessContainer::newDataPoll()
{
  if(this->gotNew240pImg)
  {
    this->imgCallBackHandler.callback(vehicle_ptr, this->imgCallBackHandler.userData);
    this->gotNew240pImg = false;
    return true;
  }
  if(this->gotNewVGAImg)
  {
    this->imgCallBackHandler.callback(vehicle_ptr, this->imgCallBackHandler.userData);
    this->gotNewVGAImg = false;
    return true;
  }
  return false;
}