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
 */
#define ADVANCED_SENSING
#include <dji_vehicle.hpp>
#include "dji_advanced_sensing.hpp"
#include "dji_version.hpp"

using namespace DJI;
using namespace DJI::OSDK;

/**** Internal variables and functions to restrict AdvancedSensing for M210 ****/

// Constants
uint32_t firmware_check_min = Version::FW(3, 2, 39, 0); //0x03022700
uint32_t firmware_check_max = Version::FW(3, 5, 99, 99); //Should cover all the 3.3.xx branches

// Functions
bool versionPass();
void internalGetDroneVersion(Vehicle* vehiclePtr);
void parseVersion(uint8_t* ackPtr);

// Variables
Version::VersionData internal_drone_version;

static pthread_t adv_pthread_handle;

void *adv_pthread(void *p){
  DSTATUS("adv pthread created !!!!!!!!!!!!!!!!!!!!!!!");
  if (p) {
    DSTATUS("adv pthread running !!!!!!!!!!!!!!!!!!!!!!!");
    RecvContainer container = {0};
    RecvContainer* recvContainer = &container;
    Vehicle*       vehiclePtr    = (Vehicle *)p;
    while (true)
    {
      recvContainer = vehiclePtr->advancedSensing->getAdvancedSensingProtocol()->receive();
      if(recvContainer->recvInfo.cmd_id != 0xFF)
      {
        vehiclePtr->processAdvancedSensingImgs(recvContainer);
      }
      usleep(10);
    }
  } else {
    DERROR("passing parameter error !");
  }

  return NULL;
}

void AdvancedSensing::init()
{
  if (!vehicle_ptr->isM300())
  pthread_create(&adv_pthread_handle, NULL, adv_pthread, vehicle_ptr);
}

void AdvancedSensing::deinit()
{
}

AdvancedSensing::AdvancedSensing(Vehicle* vehiclePtr) :
  vehicle_ptr(vehiclePtr),
  advancedSensingProtocol(NULL),
  liveview(NULL),
  perception(NULL),
  fpvCam_ptr(NULL),
  mainCam_ptr(NULL)
{
  stereoHandler.callback  = 0;
  stereoHandler.userData  = 0;
  vgaHandler.callback     = 0;
  vgaHandler.userData     = 0;

  // call a closed-source version of getDroneVersion() to prevent hacking
  internalGetDroneVersion(vehiclePtr);

  // wait for half sec
  sleep(0.5);

  if (!versionPass())
  {
    DERROR("Please make sure the connected drone is a M210 and "
             "firmware version is supported.\n");
    return;
  }

  if (vehiclePtr->isM300()) {
    DSTATUS("Advanced Sensing init for the M300 drone");
    liveview = new LiveView(vehiclePtr);
    perception = new Perception(vehiclePtr);
  } else {
    DSTATUS("Advanced Sensing init for the M210 drone");
    this->advancedSensingProtocol = new AdvancedSensingProtocol();
    liveview = new LiveView(vehiclePtr);
    fpvCam_ptr = new DJICameraStream(FPV_CAMERA);
    mainCam_ptr = new DJICameraStream(MAIN_CAMERA);
  }
}

AdvancedSensing::~AdvancedSensing()
{
  if (this->advancedSensingProtocol)
    delete this->advancedSensingProtocol;

  if(fpvCam_ptr)
  {
    delete fpvCam_ptr;
  }

  if(mainCam_ptr)
  {
    delete mainCam_ptr;
  }

  if(liveview)
  {
    delete liveview;
  }

  if(perception)
  {
    delete perception;
  }
}

void
AdvancedSensing::subscribeStereoImages(const ImageSelection *select, VehicleCallBack callback, UserData userData)
{
/*
  if (!vehicle_ptr->isUSBThreadReady())
  {
    DERROR("USB thread is not ready, please make sure AdvancedSensing is set up correctly.\n");
    if (!versionPass())
    {
      DERROR("Please make sure the connected drone is a M210 and firmware version is supported. "
               "subscription failed.\n");
    }
    return;
  }
*/

  AdvancedSensingConfig config;
  memset(&config, 0, sizeof(config));
  config.is_stereo_img_subscribed = true;

  if(select->front_left)
    config.image_selected[AdvancedSensingProtocol::FRONT][AdvancedSensingProtocol::LEFT] = 1;
  if(select->front_right)
    config.image_selected[AdvancedSensingProtocol::FRONT][AdvancedSensingProtocol::RIGHT] = 1;
  if(select->down_front)
    config.image_selected[AdvancedSensingProtocol::DOWN][AdvancedSensingProtocol::RIGHT] = 1;
  if(select->down_back)
    config.image_selected[AdvancedSensingProtocol::DOWN][AdvancedSensingProtocol::LEFT] = 1;

  uint8_t* data = (uint8_t*)&(config.image_selected);

  if (callback)
  {
    stereoHandler.callback = callback;
    stereoHandler.userData = userData;
  }
  else
  {
    stereoHandler.callback = &AdvancedSensing::stereoCallback;
    stereoHandler.userData = NULL;
  }

  sendCommonCmd(data, sizeof(config.image_selected), AdvancedSensingProtocol::SELECT_IMG_CMD_ID);

  sendCommonCmd(NULL, 0, AdvancedSensingProtocol::START_CMD_ID);
}

typedef struct M300VGAHandlerData {
  VehicleCallBackHandler handler;
  Vehicle* vehicle;
} M300VGAHandlerData;

void M300VGAHandleCB(Perception::ImageInfoType info, uint8_t *imageRawBuffer,
                       int bufferLen, void *userData) {
  static DJI::OSDK::ACK::StereoVGAImgData stereoVGAImg = {0};
  DSTATUS("image info : dataId(%d) seq(%d) timestamp(%d) datatype(%d)",
          info.dataId, info.sequence,
          info.timeStamp, info.dataType);
  DSTATUS("image info : index(%d) h(%d) w(%d) dir(%d) bpp(%d) bufferlen(%d)",
          info.rawInfo.index, info.rawInfo.height, info.rawInfo.width,
          info.rawInfo.direction, info.rawInfo.bpp, bufferLen);
  if (bufferLen != 480 * 640) {
    DERROR("Error image raw data len : %d, should be 480 * 640.", bufferLen);
    return;
  }
  auto m300Handler = (M300VGAHandlerData *) userData;
  if ((!m300Handler) || (!m300Handler->vehicle)) {
    DERROR("Error userdata");
    return;
  }
  if (!m300Handler->handler.callback) {
    DERROR("Error callback");
    return;
  }

  switch (stereoVGAImg.num_imgs) {
    case 0 :
      /*! get the first VGA image */
      DSTATUS("#### ( 1 ) get the first VGA image");
      stereoVGAImg.direction = info.rawInfo.direction;
      stereoVGAImg.frame_index = info.rawInfo.index;
      stereoVGAImg.time_stamp = info.timeStamp;
      memcpy(stereoVGAImg.img_vec[0], imageRawBuffer, 480 * 640);
      stereoVGAImg.num_imgs = 1;
      break;
    case 1 :
      /*! get the second VGA image and do handling */
      if ((stereoVGAImg.direction == info.rawInfo.direction) &&
          (stereoVGAImg.frame_index == info.rawInfo.index) &&
          (stereoVGAImg.time_stamp == info.timeStamp)) {
        DSTATUS("#### ( 2 ) get the second VGA image");
        memcpy(stereoVGAImg.img_vec[1], imageRawBuffer, 480 * 640);
        RecvContainer recvFrame = {0};
        recvFrame.recvData.stereoVGAImgData = &stereoVGAImg;
        m300Handler->handler.callback(m300Handler->vehicle, recvFrame,
                                      m300Handler->handler.userData);
        stereoVGAImg.num_imgs = 0;
      } else {
        /*! replace the first VGA image */
        DSTATUS("#### (1.5) replace the first VGA image");
        stereoVGAImg.direction = info.rawInfo.direction;
        stereoVGAImg.frame_index = info.rawInfo.index;
        stereoVGAImg.time_stamp = info.timeStamp;
        memcpy(stereoVGAImg.img_vec[0], imageRawBuffer, 480 * 640);
        stereoVGAImg.num_imgs = 1;
      }
      break;
    default:
      return;
  }

}



void
AdvancedSensing::subscribeFrontStereoVGA(const uint8_t freq,
                                         VehicleCallBack callback,
                                         UserData userData) {
/*
  if (!vehicle_ptr->isUSBThreadReady())
  {
    DERROR("USB thread is not ready, please make sure AdvancedSensing is set up correctly.\n");
    if (!versionPass())
    {
      DERROR("Please make sure the connected drone is a M210 and firmware version is supported. "
               "subscription failed.\n");
    }
    return;
  }
*/
  if (vehicle_ptr->isM210V2()) {
    AdvancedSensingConfig config;
    memset(&config, 0, sizeof(config));
    config.is_vga_img_subscribed = true;
    config.vga_subscription.image_selection[0][AdvancedSensingProtocol::FRONT].img_flag = 1;
    config.vga_subscription.image_selection[0][AdvancedSensingProtocol::FRONT].img_freq = freq;
    config.vga_subscription.image_selection[0][AdvancedSensingProtocol::FRONT].img_resolution = 1;

    uint8_t *data = (uint8_t *) &(config.vga_subscription);

    if (callback) {
      vgaHandler.callback = callback;
      vgaHandler.userData = userData;
    } else {
      vgaHandler.callback = &AdvancedSensing::VGACallback;
      vgaHandler.userData = NULL;
    }

    sendCommonCmd(data, sizeof(config.vga_subscription),
                  AdvancedSensingProtocol::SELECT_VGA_IMG_CMD_ID);

    sendCommonCmd(NULL, 0, AdvancedSensingProtocol::START_CMD_ID);
  } else if (vehicle_ptr->isM300()) {
    DSTATUS("M300 VGA freq is running at a default value at 20Hz. So the "
            "parameter freq is useless here.");
    static M300VGAHandlerData m300handler;
    m300handler.vehicle = vehicle_ptr;
    m300handler.handler = {callback, userData};
    perception->subscribePerceptionImage(Perception::RECTIFY_FRONT, M300VGAHandleCB, &m300handler);
  }
}

void
AdvancedSensing::subscribeFrontStereoDisparity(VehicleCallBack callback, UserData userData)
{
/*
  if (!vehicle_ptr->isUSBThreadReady())
  {
    DERROR("USB thread is not ready, please make sure AdvancedSensing is set up correctly.\n");
    if (!versionPass())
    {
      DERROR("Please make sure the connected drone is a M210 and firmware version is supported. "
               "subscription failed.\n");
    }
    return;
  }
*/
  AdvancedSensingConfig config;
  memset(&config, 0, sizeof(config));
  config.image_selected[AdvancedSensingProtocol::FRONT][AdvancedSensingProtocol::DISPARITY] = 1;

  uint8_t* data = (uint8_t*)&(config.image_selected);

  if (callback)
  {
    stereoHandler.callback = callback;
    stereoHandler.userData = userData;
  }
  else
  {
    stereoHandler.callback = &AdvancedSensing::stereoCallback;
    stereoHandler.userData = NULL;
  }

  sendCommonCmd(data, sizeof(config.image_selected), AdvancedSensingProtocol::SELECT_IMG_CMD_ID);

  sendCommonCmd(NULL, 0, AdvancedSensingProtocol::START_CMD_ID);
}

void
AdvancedSensing::unsubscribeStereoImages()
{
  AdvancedSensingConfig config;
  memset(&config, 0, sizeof(config));

  uint8_t* data = (uint8_t*)&(config.image_selected);

  sendCommonCmd(data, sizeof(config.image_selected), AdvancedSensingProtocol::SELECT_IMG_CMD_ID);
}

void
AdvancedSensing::unsubscribeVGAImages()
{
  if (vehicle_ptr->isM210V2()) {
    AdvancedSensingConfig config;
    memset(&config, 0, sizeof(config));

    config.is_vga_img_subscribed = true;

    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < AdvancedSensingProtocol::DISPARITY; ++j) {
        config.vga_subscription.image_selection[i][j].img_flag = 1;
        config.vga_subscription.image_selection[i][j].img_freq =
            AdvancedSensingProtocol::FREQ_20HZ;
        config.vga_subscription.image_selection[i][j].img_resolution = 0;
      }
    }

    uint8_t *data = (uint8_t *) &(config.vga_subscription);

    sendCommonCmd(data, sizeof(config.vga_subscription),
                  AdvancedSensingProtocol::SELECT_VGA_IMG_CMD_ID);

    sendCommonCmd(NULL, 0, AdvancedSensingProtocol::START_CMD_ID);
  } else if (vehicle_ptr->isM300()) {
    perception->unsubscribePerceptionImage(Perception::RECTIFY_FRONT);
  }
}

void
AdvancedSensing::sendCommonCmd(uint8_t *data, uint8_t data_len, uint8_t cmd_id)
{
  uint8_t buf[256] = {0};
  advancedSensingProtocol->formatProtocol(buf, cmd_id, data, data_len);

  advancedSensingProtocol->send((void*) buf, (void*) data, data_len);
}

void
AdvancedSensing::stereoCallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData)
{
  DSTATUS("default stereoCallback receive an image at frame: %d and time stamp: %d",
          recvFrame.recvData.stereoImgData->frame_index,
          recvFrame.recvData.stereoImgData->time_stamp);
}

void
AdvancedSensing::VGACallback(Vehicle* vehiclePtr, RecvContainer recvFrame, UserData userData)
{
  DSTATUS("default VGACallback receive an image at frame: %d and time stamp: %d",
          recvFrame.recvData.stereoVGAImgData->frame_index,
          recvFrame.recvData.stereoVGAImgData->time_stamp);
}

AdvancedSensingProtocol*
AdvancedSensing::getAdvancedSensingProtocol()
{
  return this->advancedSensingProtocol;
}

bool AdvancedSensing::startFPVCameraStream(CameraImageCallback cb, void * cbParam)
{
  return fpvCam_ptr->startCameraStream(cb, cbParam);
}

bool AdvancedSensing::startFPVCameraH264(H264Callback cb, void * cbParam)
{
  return fpvCam_ptr->startCameraH264(cb, cbParam);
}

bool AdvancedSensing::startMainCameraStream(CameraImageCallback cb, void * cbParam)
{
  // Use the keep_camera_x5s_state to prevent x5s become a storage device, otherwise could not get the stream
  return mainCam_ptr->startCameraStream(cb, cbParam);
}

bool AdvancedSensing::startMainCameraH264(H264Callback cb, void * cbParam)
{
  return mainCam_ptr->startCameraH264(cb, cbParam);
}

void AdvancedSensing::stopFPVCameraStream()
{
  fpvCam_ptr->stopCameraStream();
}

void AdvancedSensing::stopFPVCameraH264()
{
  fpvCam_ptr->stopCameraH264();
}

void AdvancedSensing::stopMainCameraStream()
{
  mainCam_ptr->stopCameraStream();
}

void AdvancedSensing::stopMainCameraH264()
{
  mainCam_ptr->stopCameraH264();
}

bool AdvancedSensing::newFPVCameraImageIsReady()
{
  return fpvCam_ptr->newImageIsReady();
}

bool AdvancedSensing::newMainCameraImageReady()
{
  return mainCam_ptr->newImageIsReady();
}

bool AdvancedSensing::getMainCameraImage(CameraRGBImage& copyOfImage)
{
  return mainCam_ptr->getCurrentImage(copyOfImage);
}

bool AdvancedSensing::getFPVCameraImage(CameraRGBImage& copyOfImage)
{
  return fpvCam_ptr->getCurrentImage(copyOfImage);
}
void AdvancedSensing::setAcmDevicePath(const char *acm_path)
{
    this->acm_dev=acm_path;
}

LiveView::LiveViewErrCode AdvancedSensing::startH264Stream(
    LiveView::LiveViewCameraPosition pos, H264Callback cb, void *userData) {
  if (vehicle_ptr->isM300())
    return liveview->startH264Stream(pos, cb, userData);
  else if(vehicle_ptr->isM210V2()) {
    switch (pos) {
      case LiveView::OSDK_CAMERA_POSITION_FPV:
        return (startFPVCameraH264(cb, userData)) ? LiveView::OSDK_LIVEVIEW_PASS : LiveView::OSDK_LIVEVIEW_UNKNOWN;
      case LiveView::OSDK_CAMERA_POSITION_NO_1:
        return(startMainCameraH264(cb, userData)) ? LiveView::OSDK_LIVEVIEW_PASS : LiveView::OSDK_LIVEVIEW_UNKNOWN;
      default:
        DERROR("M210 V2 series only support FPV and MainCam H264 steam in OSDK.");
        return LiveView::OSDK_LIVEVIEW_INDEX_ILLEGAL;
    }
  } else {
    return LiveView::OSDK_LIVEVIEW_UNSUPPORT_AIRCRAFT;
  }
}

LiveView::LiveViewErrCode AdvancedSensing::stopH264Stream(
    LiveView::LiveViewCameraPosition pos) {
  if (vehicle_ptr->isM300())
    return liveview->stopH264Stream(pos);
  else if (vehicle_ptr->isM210V2()) {
    switch (pos) {
      case LiveView::OSDK_CAMERA_POSITION_FPV:vehicle_ptr->advancedSensing->stopFPVCameraH264();
        return LiveView::OSDK_LIVEVIEW_PASS;
      case LiveView::OSDK_CAMERA_POSITION_NO_1:vehicle_ptr->advancedSensing->stopMainCameraH264();
        return LiveView::OSDK_LIVEVIEW_PASS;
      default:
        DERROR(
            "M210 V2 series only support FPV and MainCam H264 steam in OSDK.");
        return LiveView::OSDK_LIVEVIEW_INDEX_ILLEGAL;
    }
  } else {
    DERROR("Only support M210 V2 and M300.");
    return LiveView::OSDK_LIVEVIEW_UNSUPPORT_AIRCRAFT;
  }
}

Perception::PerceptionErrCode AdvancedSensing::subscribePerceptionImage(
    Perception::DirectionType direction, Perception::PerceptionImageCB cb,
    void *userData) {
  return perception->subscribePerceptionImage(direction, cb, userData);
}

Perception::PerceptionErrCode AdvancedSensing::unsubscribePerceptionImage(
    Perception::DirectionType direction) {
  return perception->unsubscribePerceptionImage(direction);
}

Perception::PerceptionErrCode AdvancedSensing::triggerStereoCamParamsPushing() {
  return perception->triggerStereoCamParamsPushing();
}

void AdvancedSensing::setStereoCamParamsObserver(Perception::PerceptionCamParamCB cb,
                                        void *userData) {
  return perception->setStereoCamParamsObserver(cb, userData);
}

void parseVersion(uint8_t* ackPtr)
{
  //! 2b ACK.
  ackPtr += 2;

  //! Next, we might have CRC or ID; Put them into a variable that we will parse
  //! later. Find next \0
  while (*ackPtr != '\0')
  {
    ackPtr++;
  }
  //! Fill in the termination character
  ackPtr++;

  //! Now, we start parsing the name. Let's find the second space character.
  while (*ackPtr != ' ')
  {
    ackPtr++;
  } //! Found first space ("SDK-v1.x")
  ackPtr++;

  while (*ackPtr != ' ')
  {
    ackPtr++;
  } //! Found second space ("BETA")
  ackPtr++;

  //! Next is the HW version
  int j = 0;
  while (*ackPtr != '-')
  {
    internal_drone_version.hwVersion[j] = *ackPtr;
    ackPtr++;
    j++;
  }
  //! Fill in the termination character
  internal_drone_version.hwVersion[j] = '\0';
  ackPtr++;

  //! Finally, we come to the FW version. We don't know if each clause is 2 or 3
  //! digits long.
  int ver1 = 0, ver2 = 0, ver3 = 0, ver4 = 0;

  while (*ackPtr != '.')
  {
    ver1 = (*ackPtr - 48) + 10 * ver1;
    ackPtr++;
  }
  ackPtr++;
  while (*ackPtr != '.')
  {
    ver2 = (*ackPtr - 48) + 10 * ver2;
    ackPtr++;
  }
  ackPtr++;
  while (*ackPtr != '.')
  {
    ver3 = (*ackPtr - 48) + 10 * ver3;
    ackPtr++;
  }
  ackPtr++;
  while (*ackPtr != '\0')
  {
    ver4 = (*ackPtr - 48) + 10 * ver4;
    ackPtr++;
  }

  internal_drone_version.fwVersion = Version::FW(ver1, ver2, ver3, ver4);
}

bool versionPass()
{
  char hwString[5] = {0};
  memcpy(hwString, internal_drone_version.hwVersion, sizeof(char)*5);
  bool isHWsupported = (strncmp(hwString, Version::M300, 5) == 0) || (strncmp(hwString, Version::M210V2, 5) == 0);

  return (internal_drone_version.fwVersion >= firmware_check_min)
         && (internal_drone_version.fwVersion <= firmware_check_max)
         && (isHWsupported);
}

void internalGetDroneVersion(Vehicle* vehiclePtr)
{
  ACK::DroneVersion version = vehiclePtr->getDroneVersion(1000);

  internal_drone_version = version.data;
}
