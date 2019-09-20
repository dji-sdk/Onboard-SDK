/** @file dji_camera_module.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of camera module for payload node
 *
 *  @Copyright (c) 2019 DJI
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

#ifndef ONBOARDSDK_DJI_CAMERA_MODULE_HPP
#define ONBOARDSDK_DJI_CAMERA_MODULE_HPP

#include "dji_command.hpp"
#include "dji_payload_base.hpp"
#include "dji_payload_link.hpp"
#include "dji_type.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI {
namespace OSDK {
/*! @brief CameraModule of PayloadNode
 */
class CameraModule : public PayloadBase {
 public:
  /*! @brief CameraModule work modes.
   */
  enum WorkMode {
    /*!
     - Capture mode. In this mode, the user can capture pictures.
     */
    SHOOT_PHOTO = 0,
    /*!
     - Record mode. In this mode, the user can record videos.
     */
    RECORD_VIDEO = 1,
    /*!
     - Playback mode. In this mode, the user can preview photos and videos, and
     can delete files. It is supported by Phantom 3 Professional camera, X3, X5
     and X5R cameras on aircraft and Phantom 4 camera. Playback mode is not
     supported by Z30, X5S, X4S, Phantom 4 Pro, Mavic Pro, Phantom 3 Standard,
     Phantom 3 Advanced, Phantom 3 4K and Osmo series.
     */
    PLAYBACK = 2,
    /*!
     - In this mode, the user can download media to the Mobile Device. Not
     supported by X5 camera nor X5R camera while mounted on aircraft.
     */
    MEDIA_DOWNLOAD = 3,
    /*!
     - In this mode, live stream resolution and frame rate will be 1080i50 (PAL)
     or 720p60 (NTSC). In this mode videos can be recorded. Still photos can
     also be taken only when video is recording. The only way to exit broadcast
     mode is to change modes to RECORD_VIDEO. Only supported by Inspire 2.
     */
    BROADCAST = 4,
    /*!
     * The camera work mode is unknown.
     */
    WORK_MODE_UNKNOWN = 0xFF,
  };

  /*! @brief The ShootPhoto mode itself can have several modes. The default
   * value is SINGLE.
   */
  enum ShootPhotoMode {
    /*!
     - Sets the camera to take a single photo.
     */
    SINGLE = DJI_CAMERA_TAKE_PHOTO_TYPE_NORMAL,
    /*!
     - Sets the camera to take an HDR photo. X5 camera, X5R camera, XT camera,
     Z30 camera, Phantom 4 Pro camera, X4S camera and X5S camera do not support
     HDR mode.
     */
    HDR = DJI_CAMERA_TAKE_PHOTO_TYPE_HDR,
    /*!
     - Set the camera to take multiple photos at once. XT camera does not
     support Burst mode.
     */
    BURST = DJI_CAMERA_TAKE_PHOTO_TYPE_BURST,
    /*!
     - Automatic Exposure Bracketing (AEB) capture. In this mode you can quickly
     take multiple shots (the default is 3) at different exposures without
     having to manually change any settings between frames. XT camera and Z30
     camera does not support AEB mode.
     */
    AEB = DJI_CAMERA_TAKE_PHOTO_TYPE_AEB,
    /*!
     - Sets the camera to take a picture (or multiple pictures) continuously at
     a set time interval. The minimum interval for JPEG format of any quality is
     2s. For all cameras except X4S, X5S and Phantom 4 Pro camera: The minimum
     interval for RAW or RAW+JPEG format is 10s. For the X4S, X5S and Phantom 4
     Pro cameras the minimum interval for RAW or RAW+JPEG dformat is 5s.
     */
    INTERVAL = DJI_CAMERA_TAKE_PHOTO_TYPE_TIME_LAPSE,
    /*!
     - Sets the camera to take a burst of RAW photos. Use getRAWPhotoBurstCount
     to check how many photos have been shot. Only supported by X5S.
     */
    RAW_BURST = DJI_CAMERA_TAKE_PHOTO_TYPE_RAW_BURST,
    /*!
     - 	Sets the camera to take an enhanced-HDR photo. It blends a sequence of
     photos for ghost-free high dynamic range photos. It is supported by Mavic 2
     Zoom and Mavic 2 Pro.
     */
    EHDR = DJI_CAMERA_TAKE_PHOTO_TYPE_EHDR,
    /*!
     - The shoot photo mode is unknown.
     */
    SHOOT_PHOTO_MODE_UNKNOWN = 0xFF,
  };

  /*! @brief the photo action of INTERVAL shooting photo mode
   */
  enum ContiCapMode {
    CONTI_CAP_SINGLE = 0, /*!< take a single photo in each Interval shooting */
    CONTI_CAP_MULTIPLE =
        1, /*!< take multiple photos in each Interval shooting */
    CONTI_CAP_TIMELAPSE = 2, /*!< used as timelapse shooting */
  };

  /*! @brief the photo action of INTERVAL shooting photo mode
   */
  enum ExposureMode {
    PROGRAM_AUTO = 1,       /*!< Program mode */
    SHUTTER_PRIORITY = 2,   /*!< Shutter priority mode */
    APERTURE_PRIORITY = 3,  /*!< Aperture priority mode */
    EXPOSURE_MANUAL = 4,    /*!< Manual mode */
    EXPOSURE_UNKNOWN = 0xFF /*!< The camera exposure mode is unknown. */
  };

  /*! @brief the photo action of INTERVAL shooting photo mode
   */
  enum PhotoAEBCount {
    /*! The number of pictures to continuously take at one time in AEB mode is 3
     */
    AEB_COUNT_3 = 3,
    /*! The number of pictures to continuously take at one time in AEB mode is 5
     */
    AEB_COUNT_5 = 5,
    /*! The number of pictures to continuously take at one time in AEB mode is 7
     */
    AEB_COUNT_7 = 7,
    /*! The number of pictures to continuously take at one time in AEB mode is
     * unknown.
     */
    AEB_COUNT_KNOWN = 0xFF,
  };

  enum PhotoBurstCount {
    /*! The number of pictures to continuously take each time in BURST mode is 2
     */
    BURST_COUNT_2 = 2,
    /*! The number of pictures to continuously take each time in BURST mode is 3
     */
    BURST_COUNT_3 = 3,
    /*! The number of pictures to continuously take each time in BURST mode is 5
     */
    BURST_COUNT_5 = 5,
    /*! The number of pictures to continuously take each time in BURST mode is 7
     */
    BURST_COUNT_7 = 7,
    /*! The number of pictures to continuously take at one time in BURST mode is
     * 10, Only supported by X4S camera, X5S camera and Phantom 4 Pro camera.
     */
    BURST_COUNT_10 = 10,
    /*! The number of pictures to continuously take at one time in BURST mode is
     * 14, Only supported by X4S camera, X5S camera and Phantom 4 Pro camera.
     */
    BURST_COUNT_14 = 14,
    /*!	The camera burst shoot count value is unknown.
     */
    BURST_COUNT_KNOWN = 0xFF,
  };
  /*! @breif CameraModule focus mode. If the physical AF switch on the camera is
   * set to auto.
   */
  enum FocusMode {
    /*!
     - The camera's focus mode is set to manual. In this mode, user sets the
     focus ring value to adjust the focal distance.
     */
    MANUAL = 0,
    /*!
     - The camera's focus mode is set to auto. For the Z30 camera, the focus is
     calculated completely automatically. For all other cameras, a focus target
     can be set by the user, which is used to calculate focus automatically.
     */
    AUTO = 1,
    /*!
     - The camera's focus mode is set to Continuous AF. It is only supported by
     Mavic Pro with firmware version V01.03.0000 or above, X4S camera, Mavic 2
     Zoom camera and Mavic 2 Pro camera.
     */
    AFC = 2,
    /*!
     - The camera's focus mode is unknown.
     */
    FOCUS_MODE_UNKNOWN = 0xFFFF,
  };

  /*! @breif CameraModule shutter mode.
   */
  enum ShutterMode {
    /*! The shutter mode of camera is automatical */
    SHUTTER_AUTO_MODE = 0x00,
    /*! The shutter mode of camera is manual, the shutter speed setting is
       valid. */
    SHUTTER_MANUAL_MODE = 0x01,
  };

  /*! @brief CameraModule shutter speed values.
   */
  enum ShutterSpeed {
    SHUTTER_SPEED_1_8000 = 0,     /*!< 1/8000 s */
    SHUTTER_SPEED_1_6400 = 1,     /*!< 1/6400 s */
    SHUTTER_SPEED_1_6000 = 2,     /*!< 1/6000 s */
    SHUTTER_SPEED_1_5000 = 3,     /*!< 1/5000 s */
    SHUTTER_SPEED_1_4000 = 4,     /*!< 1/4000 s */
    SHUTTER_SPEED_1_3200 = 5,     /*!< 1/3200 s */
    SHUTTER_SPEED_1_3000 = 6,     /*!< 1/3000 s */
    SHUTTER_SPEED_1_2500 = 7,     /*!< 1/2500 s */
    SHUTTER_SPEED_1_2000 = 8,     /*!< 1/2000 s */
    SHUTTER_SPEED_1_1600 = 9,     /*!< 1/1600 s */
    SHUTTER_SPEED_1_1500 = 10,    /*!< 1/1500 s */
    SHUTTER_SPEED_1_1250 = 11,    /*!< 1/1250 s */
    SHUTTER_SPEED_1_1000 = 12,    /*!< 1/1000 s */
    SHUTTER_SPEED_1_800 = 13,     /*!< 1/800 s */
    SHUTTER_SPEED_1_725 = 14,     /*!< 1/725 s */
    SHUTTER_SPEED_1_640 = 15,     /*!< 1/640 s */
    SHUTTER_SPEED_1_500 = 16,     /*!< 1/500 s */
    SHUTTER_SPEED_1_400 = 17,     /*!< 1/400 s */
    SHUTTER_SPEED_1_350 = 18,     /*!< 1/350 s */
    SHUTTER_SPEED_1_320 = 19,     /*!< 1/320 s */
    SHUTTER_SPEED_1_250 = 20,     /*!< 1/250 s */
    SHUTTER_SPEED_1_240 = 21,     /*!< 1/240 s */
    SHUTTER_SPEED_1_200 = 22,     /*!< 1/200 s */
    SHUTTER_SPEED_1_180 = 23,     /*!< 1/180 s */
    SHUTTER_SPEED_1_160 = 24,     /*!< 1/160 s */
    SHUTTER_SPEED_1_125 = 25,     /*!< 1/125 s */
    SHUTTER_SPEED_1_120 = 26,     /*!< 1/120 s */
    SHUTTER_SPEED_1_100 = 27,     /*!< 1/100 s */
    SHUTTER_SPEED_1_90 = 28,      /*!< 1/90 s */
    SHUTTER_SPEED_1_80 = 29,      /*!< 1/80 s */
    SHUTTER_SPEED_1_60 = 30,      /*!< 1/60 s */
    SHUTTER_SPEED_1_50 = 31,      /*!< 1/50 s */
    SHUTTER_SPEED_1_40 = 32,      /*!< 1/40 s */
    SHUTTER_SPEED_1_30 = 33,      /*!< 1/30 s */
    SHUTTER_SPEED_1_25 = 34,      /*!< 1/25 s */
    SHUTTER_SPEED_1_20 = 35,      /*!< 1/20 s */
    SHUTTER_SPEED_1_15 = 36,      /*!< 1/15 s */
    SHUTTER_SPEED_1_12DOT5 = 37,  /*!< 1/12.5 s */
    SHUTTER_SPEED_1_10 = 38,      /*!< 1/10 s */
    SHUTTER_SPEED_1_8 = 39,       /*!< 1/8 s */
    SHUTTER_SPEED_1_6DOT25 = 40,  /*!< 1/6.25 s */
    SHUTTER_SPEED_1_5 = 41,       /*!< 1/5 s */
    SHUTTER_SPEED_1_4 = 42,       /*!< 1/4 s */
    SHUTTER_SPEED_1_3 = 43,       /*!< 1/3 s */
    SHUTTER_SPEED_1_2DOT5 = 44,   /*!< 1/2.5 s */
    SHUTTER_SPEED_1_2 = 45,       /*!< 1/2 s */
    SHUTTER_SPEED_1_1DOT67 = 46,  /*!< 1/1.67 s */
    SHUTTER_SPEED_1_1DOT25 = 47,  /*!< 1/1.25 s */
    SHUTTER_SPEED_1 = 48,         /*!< 1.0 s */
    SHUTTER_SPEED_1DOT3 = 49,     /*!< 1.3 s */
    SHUTTER_SPEED_1DOT6 = 50,     /*!< 1.6 s */
    SHUTTER_SPEED_2 = 51,         /*!< 2.0 s */
    SHUTTER_SPEED_2DOT5 = 52,     /*!< 2.5 s */
    SHUTTER_SPEED_3 = 53,         /*!< 3.0 s */
    SHUTTER_SPEED_3DOT2 = 54,     /*!< 3.2 s */
    SHUTTER_SPEED_4 = 55,         /*!< 4.0 s */
    SHUTTER_SPEED_5 = 56,         /*!< 5.0 s */
    SHUTTER_SPEED_6 = 57,         /*!< 6.0 s */
    SHUTTER_SPEED_7 = 58,         /*!< 7.0 s */
    SHUTTER_SPEED_8 = 59,         /*!< 8.0 s */
    SHUTTER_SPEED_9 = 60,         /*!< 9.0 s */
    SHUTTER_SPEED_10 = 61,        /*!< 10.0 s */
    SHUTTER_SPEED_13 = 62,        /*!< 13.0 s */
    SHUTTER_SPEED_15 = 63,        /*!< 15.0 s */
    SHUTTER_SPEED_20 = 64,        /*!< 20.0 s */
    SHUTTER_SPEED_25 = 65,        /*!< 25.0 s */
    SHUTTER_SPEED_30 = 66,        /*!< 30.0 s */
    SHUTTER_SPEED_UNKNOWN = 0xFF, /*!< Unknown */
  };

  /*! @brief CameraModule ISO values.
   */
  enum ISO {
    /*! The ISO value is automatically set. This cannot be used for all cameras
       when in Manual mode. */
    ISO_AUTO = DJI_CAMERA_ISO_PARAMETER_AUTO,
    /*!  The ISO value is set to 100. */
    ISO_100 = DJI_CAMERA_ISO_PARAMETER_100,
    /*! The ISO value is set to 100. */
    ISO_200 = DJI_CAMERA_ISO_PARAMETER_200,
    /*! The ISO value is set to 100.*/
    ISO_400 = DJI_CAMERA_ISO_PARAMETER_400,
    /*! The ISO value is set to 100.*/
    ISO_800 = DJI_CAMERA_ISO_PARAMETER_800,
    /*! The ISO value is set to 100.*/
    ISO_1600 = DJI_CAMERA_ISO_PARAMETER_1600,
    /*! The ISO value is set to 100.*/
    ISO_3200 = DJI_CAMERA_ISO_PARAMETER_3200,
    /*! The ISO value is set to 100.*/
    ISO_6400 = DJI_CAMERA_ISO_PARAMETER_6400,
    /*! The ISO value is set to 100.*/
    ISO_12800 = DJI_CAMERA_ISO_PARAMETER_12800,
    /*! The ISO value is set to 100.*/
    ISO_25600 = DJI_CAMERA_ISO_PARAMETER_25600,
    /*! ISO value is fixed by the camera firmware. When the camera color is set
     to D_LOG, camera will fix the ISO to a specific value in order to optimize
     the performance.
     */
    ISO_FIXED = DJI_CAMERA_ISO_PARAMETER_FIXED,
    /*! The ISO value is set to Unknown value. */
    ISO_UNKNOWN = 0xFFFF,
  };

  /*! @brief CameraModule exposure compensation.
   */
  enum ExposureCompensation {
    /*! The camera's exposure compensation is -5.0ev.*/
    N_5_0 = 1,
    /*! The camera's exposure compensation is -4.7ev.*/
    N_4_7 = 2,
    /*! The camera's exposure compensation is -4.3ev.*/
    N_4_3 = 3,
    /*! The camera's exposure compensation is -4.0ev.*/
    N_4_0 = 4,
    /*! The camera's exposure compensation is -3.7ev.*/
    N_3_7 = 5,
    /*! The camera's exposure compensation is -3.3ev.*/
    N_3_3 = 6,
    /*! The camera's exposure compensation is -3.0ev.*/
    N_3_0 = 7,
    /*! The camera's exposure compensation is -2.7ev.*/
    N_2_7 = 8,
    /*! The camera's exposure compensation is -2.3ev.*/
    N_2_3 = 9,
    /*! The camera's exposure compensation is -2.0ev.*/
    N_2_0 = 10,
    /*! The camera's exposure compensation is -1.7ev.*/
    N_1_7 = 11,
    /*! The camera's exposure compensation is -1.3ev.*/
    N_1_3 = 12,
    /*! The camera's exposure compensation is -1.0ev.*/
    N_1_0 = 13,
    /*! The camera's exposure compensation is -0.7ev.*/
    N_0_7 = 14,
    /*! The camera's exposure compensation is -0.3ev.*/
    N_0_3 = 15,
    /*! The camera's exposure compensation is 0.0ev.*/
    N_0_0 = 16,
    /*! The camera's exposure compensation is +0.3ev.*/
    P_0_3 = 17,
    /*! The camera's exposure compensation is +0.7ev.*/
    P_0_7 = 18,
    /*! The camera's exposure compensation is +1.0ev.*/
    P_1_0 = 19,
    /*! The camera's exposure compensation is +1.3ev.*/
    P_1_3 = 20,
    /*! The camera's exposure compensation is +1.7ev.*/
    P_1_7 = 21,
    /*! The camera's exposure compensation is +2.0ev.*/
    P_2_0 = 22,
    /*! The camera's exposure compensation is +2.3ev.*/
    P_2_3 = 23,
    /*! The camera's exposure compensation is +2.7ev.*/
    P_2_7 = 24,
    /*! The camera's exposure compensation is +3.0ev.*/
    P_3_0 = 25,
    /*! The camera's exposure compensation is +3.3ev.*/
    P_3_3 = 26,
    /*! The camera's exposure compensation is +3.7ev.*/
    P_3_7 = 27,
    /*! The camera's exposure compensation is +4.0ev.*/
    P_4_0 = 28,
    /*! The camera's exposure compensation is +4.3ev.*/
    P_4_3 = 29,
    /*! The camera's exposure compensation is +4.7ev.*/
    P_4_7 = 30,
    /*! The camera's exposure compensation is +5.0ev.*/
    P_5_0 = 31,
    /*! The camera's exposure compensation is fixed by the camera.*/
    FIXED = 0xFF,
    /*! The camera's exposure compensation is unknown. */
    UNKNOWN = 0xFFFF,
  };

  /*! @brief the function ID of camera used in the CMD
   * OpenProtocolCMD::CMDSet::Control::cameraExtraFunction
   */
  enum FunctionID {
    FUNCTION_SIMPLE_SHOT = 0,
    FUNCTION_RECORD_VIDEO = 1,
    FUNCTION_SET_WORKING_MODE = 3,
    FUNCTION_GET_WORKING_MODE = 4,
    FUNCTION_SET_FOCUS_PARAMETER = 7,
    FUNCTION_SET_EXPOSURE_MODE = 8,
    FUNCTION_GET_EXPOSURE_MODE = 9,
    FUNCTION_SET_FOCUS_MODE = 10,
    FUNCTION_GET_FOCUS_MODE = 11,
    FUNCTION_SET_EV_PARAMETER = 12,
    FUNCTION_GET_EV_PARAMETER = 13,
    FUNCTION_GET_FOCUS_PARAMETER = 14,
    FUNCTION_SET_POINT_ZOOM_MODE = 15,
    FUNCTION_GET_POINT_ZOOM_MODE = 16,
    FUNCTION_POINT_ZOOM_CTRL = 17,
    FUNCTION_SET_ZOOM_PARAMETER = 19,
    FUNCTION_GET_ZOOM_PARAMETER = 20,
    FUNCTION_SET_ISO_PARAMETER = 22,
    FUNCTION_GET_ISO_PARAMETER = 23,
    FUNCTION_SET_SHUTTER_SPEED = 24,
    FUNCTION_GET_SHUTTER_SPEED = 25,
    FUNCTION_SET_APERTURE_SIZE = 26,
    FUNCTION_GET_APERTURE_SIZE = 27,
    FUNCTION_SET_METERING_MODE = 28,
    FUNCTION_GET_METERING_MODE = 29,
    FUNCTION_SET_SPOT_FOCUS_AERA = 30,
    FUNCTION_GET_SPOT_FOCUS_AERA = 31,
    FUNCTION_SET_SHOT_MODE = 32,
    FUNCTION_GET_SHOT_MODE = 33,
    FUNCTION_CONTROL_OPTIZOOM = 34,
  };

  /*! @brief CameraModule aperture values.
   *  @note X5, X5R, Z30, Phantom 4 Pro camera, X4S and X5S support this
   * setting.
   */
  enum Aperture {
    /*! 	The Aperture value is f/1.6. It is only supported by Z30
       camera.*/
    F_1_DOT_6 = 160,
    /*! The Aperture value is f/1.7.*/
    F_1_DOT_7 = 170,
    /*! The Aperture value is f/1.8.*/
    F_1_DOT_8 = 180,
    /*! The Aperture value is f/2.*/
    F_2 = 200,
    /*! The Aperture value is f/2.2.*/
    F_2_DOT_2 = 220,
    /*! The Aperture value is f/2.4. It is only supported by Z30 camera.*/
    F_2_DOT_4 = 240,
    /*! The Aperture value is f/2.5.*/
    F_2_DOT_5 = 250,
    /*! The Aperture value is f/2.6.*/
    F_2_DOT_6 = 260,
    /*! The Aperture value is f/2.8.*/
    F_2_DOT_8 = 280,
    /*! The Aperture value is f/3.2.*/
    F_3_DOT_2 = 320,
    /*! The Aperture value is f/3.4.*/
    F_3_DOT_4 = 340,
    /*! The Aperture value is f/3.5.*/
    F_3_DOT_5 = 350,
    /*! The Aperture value is f/4.*/
    F_4 = 400,
    /*! The Aperture value is f/4.5.*/
    F_4_DOT_5 = 450,
    /*! The Aperture value is f/4.8.*/
    F_4_DOT_8 = 480,
    /*! The Aperture value is f/5.*/
    F_5 = 500,
    /*! The Aperture value is f/5.6.*/
    F_5_DOT_6 = 560,
    /*! The Aperture value is f/6.3.*/
    F_6_DOT_3 = 630,
    /*! The Aperture value is f/6.8.*/
    F_6_DOT_8 = 680,
    /*! The Aperture value is f/7.1.*/
    F_7_DOT_1 = 710,
    /*! The Aperture value is f/8.*/
    F_8 = 800,
    /*! The Aperture value is f/9.*/
    F_9 = 900,
    /*! The Aperture value is f/9.6.*/
    F_9_DOT_6 = 960,
    /*! The Aperture value is f/10.*/
    F_10 = 1000,
    /*! The Aperture value is f/11.*/
    F_11 = 1100,
    /*! The Aperture value is f/13.*/
    F_13 = 1300,
    /*! The Aperture value is f/14.*/
    F_14 = 1400,
    /*! The Aperture value is f/16.*/
    F_16 = 1600,
    /*! The Aperture value is f/18.*/
    F_18 = 1800,
    /*! The Aperture value is f/19.*/
    F_19 = 1900,
    /*! The Aperture value is f/20.*/
    F_20 = 2000,
    /*! The Aperture value is f/22.*/
    F_22 = 2200,
    /*! The Aperture value is Unknown. */
    F_UNKNOWN = 0xFFFF,
  };

  enum ZoomDirection {
    /*! Lens will zoom out.The focal length decreases,
        field of view becomes wider and magnification is lower. */
    ZOOM_OUT = 0,
    /*! Lens will zoom in.The focal length increases,
      field of view becomes narrower and magnification is
      higher */
    ZOOM_IN = 1,
  };

  enum ZoomSpeed {
    SLOWEST = 72,         /*!< slowest speed */
    SLOW = 73,            /*!< slow speed */
    MODERATELY_SLOW = 74, /*!< slightly slower than normal speed */
    NORMAL = 75,          /*!< normal speed */
    MODERATELY_FAST = 76, /*!< speed slightly faster than normal speed */
    FAST = 77,            /*!< fast speed */
    FASTEST = 78,         /*!< fastest speed */
  };

#pragma pack(1)

  /*! @brief Temporary struct used in the special Open Protocol CMD
   * DJI::OSDK::CameraModule::FunctionID. User should choose the Function ID to
   * choose the function and then use the payloadNodeIndex to specify which
   * device to receive the data.
   */
  typedef struct FuncParam {
    uint8_t funcIndex;        /*!< ref to DJI::OSDK::CameraModule::FunctionID */
    uint8_t payloadNodeIndex; /*!< PayloadNode index. It should be paid
                                 attention that real payload node index +1 */
  } FuncParam;                // pack(1)

  /*! @brief It is the common ack of Open Protocol CMD
   * DJI::OSDK::CameraModule::FunctionID. All the ack data from this CMD will
   * include the ack struct.
   */
  typedef struct CommonAck {
    FuncParam funcParam; /*!< ref to DJI::OSDK::CameraModule::FuncParam */
    uint8_t ret_code;    /*!< original return code from vehicle */
  } CommonAck;           // pack(1)

  /*! @brief Take photo mode data used by user.
   */
  typedef uint8_t
      ShootPhotoModeData; /*!< ref to DJI::OSDK::CameraModule::ShootPhotoMode */

  /*! @brief Take photo mode data used by user.ref to
   * DJI::OSDK::CameraModule::PhotoAEBCount
   */
  typedef uint8_t PhotoAEBCountData;

  /*! @brief Take photo mode data used by user.ref to
   * DJI::OSDK::CameraModule::PhotoBurstCount
   */
  typedef uint8_t PhotoNumBurstData;

  /*! @brief Interval take photo mode setting
   */
  typedef struct PhotoIntervalData {
    uint8_t photoNumConticap; /*!< 0:reserve 1~254:number 255:keep capturing
                                 till stop */
    int16_t timeInterval;     /*!< time interval (second) */
  } PhotoIntervalData;

  /*! @brief Take photo mode parameters
   */
  typedef struct CaptureParamData {
    ShootPhotoModeData
        captureMode; /*!< ref to DJI::OSDK::CameraModule::ShootPhotoMode */
    PhotoNumBurstData photoNumBurst; /*!< burst take photo number */
    uint8_t conticapType; /*!< ref to DJI::OSDK::CameraModule::ContiCapMode */
    PhotoIntervalData intervalSetting; /*!< time interval setting */
  } CaptureParamData;                  // pack(1)

  /*! @brief Request data to take photo mode
   */
  typedef struct CaptureParamReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    CaptureParamData
        captureParam; /*!< Take photo mode parameters, ref to
                         DJI::OSDK::CameraModule::CaptureParamData */
  } CaptureParamReq;  // pack(1)

  /*! @brief Response data of take photo mode
   */
  typedef struct CaptureParamAck {
    FuncParam funcParam;           /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;              /*!< original return code from vehicle */
    CaptureParamData captureParam; /*!< Take photo mode parameters, ref to
                         DJI::OSDK::CameraModule::CaptureParamData */
  } CaptureParamAck;               // pack(1)

  /*! @brief Request data to record video mode
   */
  typedef struct RecordVideoReq {
    FuncParam funcParam;           /*!< function ID and payloadnode indiex ID */
    uint8_t recording_control : 2; /*!< ref to
                                      DJI::OSDK::DJI_CAMERA_RECORDING_CONTROL */
    uint8_t
        recording_type : 4; /*!< ref to DJI::OSDK::DJI_CAMERA_RECORDING_TYPE */
    uint8_t reserved : 2;
  } RecordVideoReq;  // pack(1)

  /*! @brief CameraModule working mode data used by user
   */
  typedef uint8_t WorkModeData; /*!< ref to DJI::OSDK::CameraModule::WorkMode */

  /*! @brief Requesting CameraModule working mode data, used in internal link
   * layer.
   */
  typedef struct WorkModeReq {
    FuncParam funcParam;      /*!< function ID and payloadnode indiex ID */
    WorkModeData workingMode; /*!< ref to DJI::OSDK::CameraModule::WorkMode */
  } WorkModeReq;              // pack(1)

  /*! @brief Responsing CameraModule working mode data, used in internal link
   * layer.
   */
  typedef struct WorkModeAck {
    FuncParam funcParam;      /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;         /*!< original return code from vehicle */
    WorkModeData workingMode; /*!< ref to DJI::OSDK::CameraModule::WorkMode */
  } WorkModeAck;              // pack(1)

  /*! @brief Requesting take photo mode, used in internal link layer.
   */
  typedef struct ShootPhotoReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    ShootPhotoModeData
        takePhotoType; /*!< ref to DJI::OSDK::CameraModule::ShootPhotoMode */
  } ShootPhotoReq;     // pack(1)

  /*! @brief Focus mode data used by user
   */
  typedef uint8_t
      FocusModeData; /*!< ref to DJI::OSDK::CameraModule::FocusMode */

  /*! @brief Requesting focus mode, used in internal link layer.
   */
  typedef struct FocusModeReq {
    FuncParam funcParam;     /*!< function ID and payloadnode indiex ID */
    FocusModeData focusMode; /*!< ref to DJI::OSDK::CameraModule::FocusMode */
  } FocusModeReq;            // pack(1)

  /*! @brief Response data struct of focus mode, used in internal link layer.
   */
  typedef struct FocusModeAck {
    FuncParam funcParam;     /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;        /*!< original return code from vehicle */
    FocusModeData focusMode; /*!< ref to DJI::OSDK::CameraModule::FocusMode */
  } FocusModeAck;            // pack(1)

  /*! @brief Exposure mode data used by user
   */
  typedef uint8_t
      ExposureModeData; /*!< ref to DJI::OSDK::CameraModule::ExposureMode */

  /*! @brief Requesting exposure mode, used in internal link layer.
   */
  typedef struct ExposureModeReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    ExposureModeData
        exposureModeMode; /*!< ref to DJI::OSDK::CameraModule::ExposureMode */
    uint8_t reserve;
  } ExposureModeReq;  // pack(1)

  /*! @brief Response data struct of exposure mode, used in internal link layer.
   */
  typedef struct ExposureModeAck {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;    /*!< original return code from vehicle */
    ExposureModeData
        exposureMode; /*!< ref to DJI::OSDK::CameraModule::ExposureMode */
    uint8_t reserve;
  } ExposureModeAck;  // pack(1)

  /*! @brief Zoom direction data used by user, ref to
   * DJI::OSDK::CameraModule::ZoomDirection
   */
  typedef uint8_t zoomDirectionData;

  /*! @brief Zoom speed data used by user, ref to
   * DJI::OSDK::CameraModule::ZoomSpeed
   */
  typedef uint8_t zoomSpeedData;

  /*! @brief Data struct of optical zoom parameters
   */
  typedef struct zoomOptiParamType {
    zoomDirectionData
        zoomType; /*!< 1:continue zoom 2:setting focus lengh  255:stop */
    zoomSpeedData zoomSpeed; /*!< optical zoom direction, ref to
                        DJI::OSDK::CameraModule::ZoomSpeed */
    union {
      struct {
        uint8_t zoomDirection; /*!< optical zoom direction, ref to
                                  DJI::OSDK::CameraModule::ZoomDirection */
        uint8_t padding;       /*!< reserve value*/
      } zoomContiParam;
      uint16_t
          zoomFocusLength; /*!< focal length of the thermal imaging camera */
    } zoomParam;
  } zoomOptiParamType;

  /*! @brief Requesting optical zoom, used in internal link layer.
   */
  typedef struct zoomOptiParamReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    zoomOptiParamType zoomOptiParam; /*!< optical zoom parameters */
  } zoomOptiParamReq;

  /*! @brief Data struct of tap zoom enabled used by user.
   */
  typedef uint8_t TapZoomEnableData;

  /*! @brief Data struct of tap zoom multiplier used by user. Limit: 1~5
   */
  typedef uint8_t TapZoomMultiplierData;

  /*! @brief Requesting to set tap zoom enable, used in internal link layer.
   */
  typedef struct TapZoomEnableReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    TapZoomEnableData tapZoomEnable;  /*!< tap zoom enabled parameter */
    TapZoomMultiplierData multiplier; /*!< tap zoom multiplier parameter */
  } TapZoomEnableReq;                 // pack(1)

  /*! @brief Response data struct of tap zoom enable parameter, used in internal
   * link layer.
   */
  typedef struct TapZoomEnableAck {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;    /*!< original return code from vehicle */
    TapZoomEnableData tapZoomEnable;  /*!< tap zoom enabled parameter */
    TapZoomMultiplierData multiplier; /*!< tap zoom multiplier parameter */
  } TapZoomEnableAck;                 // pack(1)

  /*! @brief 2D float point data struct
   */
  typedef struct FloatPoint {
    float x;     /*!< range from 0~1, top left corner is origin point */
    float y;     /*!< range from 0~1, top left corner is origin point */
  } FloatPoint;  // pack(1)

  /*! @brief Tap zoom target point data struct, used by user.
   */
  typedef FloatPoint TapZoomPosData;  // pack(1)

  /*! @brief Tap focus target point data struct, used by user.
   */
  typedef FloatPoint TapFocusPosData;  // pack(1)

  /*! @brief Requesting to set the target point of some functions , used in
   * internal link layer.
   */
  typedef struct FloatPointReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    FloatPoint p;        /*!< float x and float y of the point */
  } FloatPointReq;

  /*! @brief Requesting to set the target of tapzoom , used in internal link
   * layer.
   */
  typedef FloatPointReq TapZoomPosReq;  // pack(1)

  /*! @brief Requesting to set the target of tapfocus , used in internal link
   * layer.
   */
  typedef FloatPointReq TapFocusPosReq;  // pack(1)

  /*! @brief Response data struct of tap focus target parameter, used in
   * internal link layer.
   */
  typedef struct TapFocusPosAck {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;    /*!< original return code from vehicle */
    TapZoomPosData p;    /*!< tap zoom point data */
    int8_t hor_region_num;
    int8_t ver_region_num;
  } TapFocusPosAck;  // pack(1)

  /*! @brief ISO data used by user.
   */
  typedef uint8_t ISOParamData; /*!< ref to DJI::OSDK::CameraModule::ISO */

  /*! @brief Requesting to set ISO parameter, used in internal link layer.
   */
  typedef struct ISOParamReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    ISOParamData iso;    /*!< ref to DJI::OSDK::CameraModule::ISO */
  } ISOParamReq;         // pack(1)

  /*! @brief Response data struct of ISO parameter, used in internal link layer.
   */
  typedef struct ISOParamAck {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;    /*!< original return code from vehicle */
    ISOParamData iso;    /*!< ref to DJI::OSDK::CameraModule::ISO */
  } ISOParamAck;         // pack(1)

  /*! @brief Aperture size data used by user. ref to
   * DJI::OSDK::CameraModule::Aperture
   */
  typedef uint16_t ApertureData;

  /*! @brief Requesting to set aperture parameter, used in internal link layer.
   */
  typedef struct ApertureReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    ApertureData size;   /*!< aperture value of camera */
  } ApertureReq;         // pack(1)

  /*! @brief Response data struct of aperture, used in internal link layer.
   */
  typedef struct ApertureAck {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;    /*!< original return code from vehicle */
    ApertureData size;   /*!< aperture value of camera */
  } ApertureAck;         // pack(1)

  /*! @brief Shutter data struct
   */
  typedef struct ShutterParamType {
    uint16_t integer_part : 15; /*!< the integer part of shutter speed */
    uint16_t reciprocal : 1;    /*!< take the reciprocal or not */
    uint8_t decimal_part;       /*!< the decimal part of shutter speed */
  } ShutterSpeedType;           // pack(1)

  /*! @brief Shutter data type used by user. ref to
   * DJI::OSDK::CameraModule::ShutterSpeed
   */
  typedef uint8_t ShutterSpeedData;  // pack(1)

  /*! @brief Requesting to set shutter parameter, used in internal link layer.
   */
  typedef struct ShutterReq {
    FuncParam funcParam;  /*!< function ID and payloadnode indiex ID */
    uint8_t shutter_mode; /*!< ref to DJI::OSDK::CameraModule::ShutterMode */
    ShutterSpeedType shutterSpeed; /*!< shutter speed of camera */
  } ShutterReq;                    // pack(1)

  /*! @brief Response data struct of shutter, used in internal link layer.
   */
  typedef struct ShutterAck {
    FuncParam funcParam;  /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code;     /*!< original return code from vehicle */
    uint8_t shutter_mode; /*!< ref to DJI::OSDK::CameraModule::ShutterMode */
    ShutterSpeedType shutter;
  } ShutterAck;  // pack(1)

  /*! @brief Exposure compensation data type used by user.
   */
  typedef uint8_t
      ExposureCompensationData; /*!< ref to
                                   DJI::OSDK::CameraModule::ExposureCompensation
                                 */

  /*! @brief Requesting to set Exposure compensation, used in internal link
   * layer.
   */
  typedef struct ExposureCompensationReq {
    FuncParam funcParam; /*!< function ID and payloadnode indiex ID */
    ExposureCompensationData
        ev; /*!< ref to DJI::OSDK::CameraModule::ExposureCompensation */
  } ExposureCompensationReq;  // pack(1)

  /*! @brief Response data struct of exposure compensation, used in internal
   * link layer.
   */
  typedef struct ExposureCompensationAck {
    FuncParam param;  /*!< function ID and payloadnode indiex ID */
    uint8_t ret_code; /*!< original return code from vehicle */
    ExposureCompensationData
        ev_param; /*!< ref to DJI::OSDK::CameraModule::ExposureCompensation */
  } ExposureCompensationAck;  // pack(1)

#pragma pack()

  /*! @brief type of callback only deal the retCode for user
   */
  typedef struct UCBRetCodeHandler {
    void (*UserCallBack)(ErrorCode::ErrorCodeType errCode, UserData userData);

    UserData userData;
  } UCBRetCodeHandler;

  /*! @brief struct of callback deal the param and retCode for user
   */
  template <typename T>
  struct UCBRetParamStruct {
    void (*UserCallBack)(ErrorCode::ErrorCodeType errCode, T param,
                         UserData userData);

    UserData userData;
  };

  /*! @brief type of callback deal the param and retCode for user
   */
  template <typename T>
  using UCBRetParamHandler = UCBRetParamStruct<T>;

 public:
  CameraModule(PayloadLink* payloadLink, PayloadIndexType payloadIndex,
               std::string name, bool enable);

  ~CameraModule();

 public:
  /*! @brief start to shoot photo, non-blocking calls
   *
   *  @param mode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::TakePhotoMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startShootPhotoAsync(
      ShootPhotoMode mode,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief start to shoot photo, blocking calls
   *
   *  @param mode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::TakePhotoMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startShootPhotoSync(ShootPhotoMode mode,
                                               int timeout);

  /*! @brief stop to shoot photo, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopShootPhotoAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief stop to shoot photo, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopShootPhotoSync(int timeout);

  /*! @brief set the shoot photo mode, non-blocking calls
   *
   *  @param takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setShootPhotoModeAsync(
      ShootPhotoMode takePhotoMode,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the shoot photo mode, blocking calls
   *
   *  @param takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setShootPhotoModeSync(ShootPhotoMode takePhotoMode,
                                                 int timeout);

  /*! @brief get the shoot photo mode, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getShootPhotoModeAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           ShootPhotoMode takePhotoMode, UserData userData),
      UserData userData);

  /*! @brief get the shoot photo mode, blocking calls
   *
   *  @param takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getShootPhotoModeSync(ShootPhotoMode& takePhotoMode,
                                                 int timeout);

  /*! @brief set the burst count in the Burst take-photo mode, non-blocking
   * calls
   *
   *  @param count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setPhotoBurstCountAsync(
      PhotoBurstCount count,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the burst count in the Burst take-photo mode, blocking calls
   *
   *  @param count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setPhotoBurstCountSync(PhotoBurstCount count,
                                                  int timeout);

  /*! get the burst count in the Burst take-photo mode, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getPhotoBurstCountAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           PhotoBurstCount count, UserData userData),
      UserData userData);

  /*! @brief get the burst count in the Burst take-photo mode, blocking calls
   *
   *  @param count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPhotoBurstCountSync(PhotoBurstCount& count,
                                                  int timeout);

  /*! @brief set the burst count in the AEB take-photo mode, non-blocking calls
   *
   *  @param count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setPhotoAEBCountAsync(
      PhotoAEBCount count,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the burst count in the AEB take-photo mode, blocking calls
   *
   *  @param count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setPhotoAEBCountSync(PhotoAEBCount count,
                                                int timeout);

  /*! get the burst count in the AEB take-photo mode, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getPhotoAEBCountAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           PhotoAEBCount count, UserData userData),
      UserData userData);

  /*! @brief get the burst count in the AEB take-photo mode, blocking calls
   *
   *  @param count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPhotoAEBCountSync(PhotoAEBCount& count,
                                                int timeout);

  /*! @brief set the parameters in the INTERVAL take-photo mode, non-blocking
   * calls
   *
   *  @param intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setPhotoTimeIntervalSettingsAsync(
      PhotoIntervalData intervalSetting,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the parameters in the INTERVAL take-photo mode, blocking calls
   *
   *  @param intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setPhotoTimeIntervalSettingsSync(
      PhotoIntervalData intervalSetting, int timeout);

  /*! @brief get the parameters in the INTERVAL take-photo mode, non-blocking
   * calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getPhotoIntervalDatasAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           PhotoIntervalData intervalSetting,
                           UserData userData),
      UserData userData);

  /*! @brief get the parameters in the INTERVAL take-photo mode, blocking calls
   *
   *  @param intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPhotoIntervalDatasSync(
      PhotoIntervalData& intervalSetting, int timeout);

  /*! @brief start to take video, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startRecordVideoAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief start to take video, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startRecordVideoSync(int timeout);

  /*! @brief stop to take video, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopRecordVideoAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief stop to take video, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopRecordVideoSync(int timeout);

  /*! @brief set camera working mode, non-blocking calls
   *
   *  @param mode camera working mode, input limit see enum
   * DJI::OSDK::CameraModule::WorkMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setModeAsync(WorkMode mode,
                    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                         UserData userData),
                    UserData userData);

  /*! @brief set camera working mode, blocking calls
   *
   *  @param mode camera working mode, input limit see enum
   * DJI::OSDK::CameraModule::WorkMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setModeSync(WorkMode mode, int timeout);

  /*! @brief get camera working mode, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b mode used as an input param, please see enum
   * DJI::OSDK::CameraModule::WorkMode
   *  @arg @b userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getModeAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                         WorkMode workingMode,
                                         UserData userData),
                    UserData userData);

  /*! @brief get camera working mode, blocking calls
   *
   *  @param workingMode used as an output param, camera working mode, input
   * limit see enum DJI::OSDK::CameraModule::WorkMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getModeSync(WorkMode& workingMode, int timeout);

  /*! @brief set camera focus mode, non-blocking calls
   *
   *  @param mode camera focus mode, input limit see enum
   * DJI::OSDK::CameraModule::FocusMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setFocusModeAsync(FocusMode mode,
                         void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief get camera focus mode, blocking calls
   *
   *  @param mode camera focus mode, input limit see enum
   * DJI::OSDK::CameraModule::FocusMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setFocusModeSync(FocusMode mode, int timeout);

  /*! @brief get camera focus mode, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b mode used as an input param, please see enum
   * DJI::OSDK::CameraModule::FocusMode userData the interface to pass userData
   * in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getFocusModeAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              FocusMode mode,
                                              UserData userData),
                         UserData userData);

  /*! @brief get camera focus mode, blocking calls
   *
   *  @param focusMode used as an output param, camera focus mode, input limit
   * see enum DJI::OSDK::CameraModule::FocusMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getFocusModeSync(FocusMode& focusMode, int timeout);

  /*! @brief set camera tap focus target point, non-blocking calls
   *
   *  @note Sets the lens focus mode. When the focus mode is auto, the target
   *  point is the focal point. When the focus mode is manual, the target point
   *  is the zoom out area if the focus assistant is enabled for the manual
   *  mode. Supported only by the X5, X5R, Z3 cameras, Mavic Pro camera,
   *  Phantom 4 Pro camera, Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise
   *  Camera, X5S. It's should be attention that X4S will keep focus point as
   *  (0.5,0.5) all the time, the setting of focus point to X4S will quickly
   *  replaced by (0.5, 0.5).
   *  @param tapFocusPos the param of tap focus, including x,y value
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setFocusTargetAsync(
      TapFocusPosData tapFocusPos,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera focus point, blocking calls
   *
   *  @note Sets the lens focus mode. When the focus mode is auto, the target
   *  point is the focal point. When the focus mode is manual, the target point
   *  is the zoom out area if the focus assistant is enabled for the manual
   *  mode. Supported only by the X5, X5R, Z3 cameras, Mavic Pro camera,
   *  Phantom 4 Pro camera, Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise
   *  Camera, X5S. It's should be attention that X4S will keep focus point as
   *  (0.5,0.5) all the time, the setting of focus point to X4S will quickly
   *  replaced by (0.5, 0.5).
   *  @param tapFocusPos the param of tap focus, including x,y value
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setFocusTargetSync(TapFocusPosData tapFocusPos,
                                              int timeout);

  /*! @brief get camera tap focus target point, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b tapFocusPos used as an input param, the param of tap focus,
   * including x,y value userData the interface to pass userData in when the
   * callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getFocusTargetAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType,
                                                TapFocusPosData tapFocusPos,
                                                UserData userData),
                           UserData userData);

  /*! @brief get camera focus point, blocking calls
   *
   *  @param tapFocusPos used as an output param, the param of tap focus,
   * including x,y value
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getFocusTargetSync(TapFocusPosData& tapFocusPos,
                                              int timeout);

  /*! @brief start camera optical zooming, non-blocking calls
   *
   *  @param zoomDirection optical zoom direction, ref to
   * DJI::OSDK::CameraModule::ZoomDirection
   *  @param zoomSpeed optical zoom direction, ref to
   * DJI::OSDK::CameraModule::ZoomSpeed
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startContinuousOpticalZoomAsync(
      zoomDirectionData zoomDirection, zoomSpeedData zoomSpeed,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief start camera optical zooming, blocking calls
   *
   *  @param zoomDirection optical zoom direction, ref to
   * DJI::OSDK::CameraModule::ZoomDirection
   *  @param zoomSpeed optical zoom direction, ref to
   * DJI::OSDK::CameraModule::ZoomSpeed
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startContinuousOpticalZoomSync(
      zoomDirectionData zoomDirection, zoomSpeedData zoomSpeed, int timeout);

  /*! @brief stop camera optical zooming, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopContinuousOpticalZoomAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief stop camera optical zooming, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopContinuousOpticalZoomSync(int timeout);

  /*! @brief set camera tap zoom function parameters, non-blocking calls
   *
   *  @param param tap zoom enable data
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback
   * is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   *  @details It should be paid attention that, tap zoom have not getter
   * API
   */
  void setTapZoomEnabledAsync(
      bool param,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera tap zoom parameters, blocking calls
   *
   *  @param param tap zoom enable data
   *  @param timeout blocking timeout in seconds
   *  @details It should be paid attention that, tap zoom have not getter API
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setTapZoomEnabledSync(bool param, int timeout);

  /*! @brief get camera tap zoom function parameters, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b param used as an input param, tap zoom enable data
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getTapZoomEnabledAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, bool param,
                           UserData userData),
      UserData userData);

  /*! @brief get camera tap zoom parameters, blocking calls
   *
   *  @param param used as an output param, tap zoom enable data
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getTapZoomEnabledSync(bool& param, int timeout);

  /*! @brief set camera tap zoom function parameters, non-blocking calls
   *
   *  @param param tap zoom multiplier data
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   *  @details It should be paid attention that, tap zoom have not getter API
   */
  void setTapZoomMultiplierAsync(
      TapZoomMultiplierData param,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera tap zoom parameters, blocking calls
   *
   *  @param param tap multiplier enable data
   *  @param timeout blocking timeout in seconds
   *  @details It should be paid attention that, tap zoom have not getter API
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setTapZoomMultiplierSync(TapZoomMultiplierData param,
                                                    int timeout);

  /*! @brief get camera tap zoom function parameters, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b param used as an input param, tap zoom multiplier data
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getTapZoomMultiplierAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           TapZoomMultiplierData param, UserData userData),
      UserData userData);

  /*! @brief get camera tap zoom parameters, blocking calls
   *
   *  @param param used as an output param, tap zoom multiplier data
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getTapZoomMultiplierSync(
      TapZoomMultiplierData& param, int timeout);

  /*! @brief set camera tap zoom point, non-blocking calls
   *
   *  @param tapZoomPos the param of tap zoom, including x,y value
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void tapZoomAtTargetAsync(
      TapZoomPosData tapZoomPos,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera tap zoom point, blocking calls
   *
   *  @param tapZoomPos the param of tap zoom, including x,y value
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType tapZoomAtTargetSync(TapZoomPosData tapZoomPos,
                                               int timeout);

  /*! @brief set camera exposure mode, non-blocking calls
   *
   *  @note The different exposure modes define whether Aperture, Shutter Speed,
   * ISO can be set automatically or manually. Exposure compensation can be
   * changed in all modes except Manual mode where it is not settable. X5, X5R,
   * Phantom 4 Pro camera, X4S and X5S: Program Mode: Shutter: Auto Aperture:
   * Auto ISO: Manual or Auto Shutter Priority: Shutter: Manual Aperture: Auto
   * ISO: Manual or Auto Aperture Priority: Shutter: Auto Aperture: Manual ISO:
   * Manual or Auto Manual Mode: Shutter: Manual Aperture: Manual ISO: Manual
   * All other cameras:
   * Program Mode: Shutter: Auto Aperture: Fixed ISO: Auto
   * Shutter Priority: Shutter: Manual Aperture: Fixed ISO: Auto
   * Aperture Priority: NA
   * Manual Mode: Shutter: Manual Aperture: Manual ISO: Manual
   *  @param mode exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setExposureModeAsync(
      ExposureMode mode,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera exposure mode, blocking calls
   *
   *  @param mode exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setExposureModeSync(ExposureMode mode, int timeout);

  /*! @brief get camera exposure mode, non-blocking calls
   *
   *  @note The different exposure modes define whether Aperture, Shutter Speed,
   * ISO can be set automatically or manually. Exposure compensation can be
   * changed in all modes except Manual mode where it is not settable. X5, X5R,
   * Phantom 4 Pro camera, X4S and X5S: Program Mode: Shutter: Auto Aperture:
   * Auto ISO: Manual or Auto Shutter Priority: Shutter: Manual Aperture: Auto
   * ISO: Manual or Auto Aperture Priority: Shutter: Auto Aperture: Manual ISO:
   * Manual or Auto Manual Mode: Shutter: Manual Aperture: Manual ISO: Manual
   * All other cameras:
   * Program Mode: Shutter: Auto Aperture: Fixed ISO: Auto
   * Shutter Priority: Shutter: Manual Aperture: Fixed ISO: Auto
   * Aperture Priority: NA
   * Manual Mode: Shutter: Manual Aperture: Manual ISO: Manual
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b mode exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @arg @b userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getExposureModeAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, ExposureMode mode,
                           UserData userData),
      UserData userData);

  /*! @brief get camera exposure mode, blocking calls
   *
   *  @param mode used as an output param,exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getExposureModeSync(ExposureMode& mode, int timeout);

  /*! @brief set camera iso value, non-blocking calls
   *
   *  @param iso camera iso, input limit see enum
   * DJI::OSDK::CameraModule::ISOParameter
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setISOAsync(ISO iso,
                   void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                        UserData userData),
                   UserData userData);

  /*! @brief set camera iso value, blocking calls
   *
   *  @param iso camera iso, input limit see enum
   * DJI::OSDK::CameraModule::ISOParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setISOSync(ISO iso, int timeout);

  /*! @brief get camera iso value, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b iso used as an input param, camera iso, input limit see enum
   *  DJI::OSDK::CameraModule::ISOParameter
   *  @arg @b userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getISOAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType, ISO iso,
                                        UserData userData),
                   UserData userData);

  /*! @brief get camera iso value, blocking calls
   *
   *  @param iso used as an output param, camera iso, input limit see enum
   * DJI::OSDK::CameraModule::ISOParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getISOSync(ISO& iso, int timeout);

  /*! @brief set camera aperture size value, non-blocking calls
   *
   *  @param size camera aperture size, input limit see enum
   * DJI::OSDK::CameraModule::ApertureSize
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setApertureAsync(Aperture size,
                        void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);

  /*! @brief set camera aperture size value, blocking calls
   *
   *  @param size camera aperture size, input limit see enum
   * DJI::OSDK::CameraModule::ApertureSize
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setApertureSync(Aperture size, int timeout);

  /*! @brief get camera aperture size value, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b size used as an input param, camera aperture size, input limit see
   * enum DJI::OSDK::CameraModule::ApertureSize
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getApertureAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType,
                                             Aperture size, UserData userData),
                        UserData userData);

  /*! @brief get camera aperture size value, blocking calls
   *
   *  @param size used as an output param, camera aperture size, input limit see
   * enum DJI::OSDK::CameraModule::ApertureSize
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getApertureSync(Aperture& size, int timeout);

  /*! @brief set camera shutter value, non-blocking calls
   *
   *  @param shutter the shutter mode and param of camera, input limit see enum
   *  DJI::OSDK::CameraModule::ShutterMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setShutterSpeedAsync(
      ShutterSpeed shutterSpeed,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera shutter value, blocking calls
   *
   *  @param shutter the shutter mode and param of camera, input limit see enum
   *  DJI::OSDK::CameraModule::ShutterMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setShutterSpeedSync(ShutterSpeed shutterSpeed,
                                               int timeout);

  /*! @brief get camera shutter value, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b shutter used as an input param, the shutter mode and param of
   * camera, input limit see enum DJI::OSDK::CameraModule::ShutterMode userData
   * the interface to pass userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getShutterSpeedAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           ShutterSpeed shutterSpeed, UserData userData),
      UserData userData);

  /*! @brief get camera shutter value, blocking calls
   *
   *  @param shutter used as an output param, the shutter mode and param of
   * camera, input limit see enum DJI::OSDK::CameraModule::ShutterMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getShutterSpeedSync(ShutterSpeed& shutterSpeed,
                                               int timeout);

  /*! @brief set camera EV value, non-blocking calls
   *
   *  @param ev ev parameter value of camera, input limit see enum
   * DJI::OSDK::CameraModule::EVParameter
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setExposureCompensationAsync(
      ExposureCompensation ev,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera EV value, blocking calls
   *
   *  @param ev ev parameter value of camera, input limit see enum
   * DJI::OSDK::CameraModule::EVParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setExposureCompensationSync(ExposureCompensation ev,
                                                       int timeout);

  /*! @brief get camera EV value, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b ev used as an input param, ev parameter value of camera, input
   * limit see enum DJI::OSDK::CameraModule::EVParameter userData the interface
   * to pass userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getExposureCompensationAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           ExposureCompensation ev, UserData userData),
      UserData userData);

  /*! @brief get camera EV value, blocking calls
   *
   *  @param ev used as an output param, ev parameter value of camera, input
   * limit see enum DJI::OSDK::CameraModule::EVParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getExposureCompensationSync(ExposureCompensation& ev,
                                                       int timeout);

 private:
  /*! @brief Decoder callback to decode common ack of camera module, then call
   * the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void commonAckDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                               UCBRetCodeHandler* ucb);

  /*! @brief Decoder callback to decode the ack of getting ExposureMode, then
   * call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getExposureModeDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                     UCBRetParamHandler<ExposureMode>* ucb);

  /*! @brief Decoder callback to decode the ack of getting ISO, then call the
   * ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getISODecoder(Vehicle* vehicle, RecvContainer recvFrame,
                            UCBRetParamHandler<ISO>* ucb);

  /*! @brief Decoder callback to decode the ack of getting camera work mode ,
   * then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getModeDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                             UCBRetParamHandler<WorkMode>* ucb);
  /*! @brief Decoder callback to decode the ack of getting focus mode, then call
   * the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getFocusModeDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                  UCBRetParamHandler<FocusMode>* ucb);
  /*! @brief Decoder callback to decode the ack of getting focus target, then
   * call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getFocusTargetDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                    UCBRetParamHandler<TapFocusPosData>* ucb);
  /*! @brief Decoder callback to decode the ack of getting aperture, then call
   * the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getApertureDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                 UCBRetParamHandler<Aperture>* ucb);
  /*! @brief Decoder callback to decode the ack of getting shutter speed, then
   * call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getShutterSpeedDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                     UCBRetParamHandler<ShutterSpeed>* ucb);
  /*! @brief Decoder callback to decode the ack of getting tap zoom enable
   * parameter, then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getTapZoomEnabledDecoder(Vehicle* vehicle,
                                       RecvContainer recvFrame,
                                       UCBRetParamHandler<bool>* ucb);
  /*! @brief Decoder callback to decode the ack of getting exposure compensation
   * parameter, then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getEVDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                           UCBRetParamHandler<ExposureCompensation>* ucb);
  /*! @brief Decoder callback to decode the ack of getting tap zoom multiplier,
   * then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getTapZoomMultiplierDecoder(
      Vehicle* vehicle, RecvContainer recvFrame,
      UCBRetParamHandler<TapZoomMultiplierData>* ucb);
  /*! @brief Decoder callback to decode the ack of getting capture parameters
   * data, then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getCaptureParamDataDecoder(
      Vehicle* vehicle, RecvContainer recvFrame,
      UCBRetParamHandler<CaptureParamData>* ucb);
  /*! @brief Decoder callback to decode the ack of getting shoot photo mode,
   * then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getShootPhotoModeDataDecoder(
      Vehicle* vehicle, RecvContainer recvFrame,
      UCBRetParamHandler<ShootPhotoMode>* ucb);
  /*! @brief Decoder callback to decode the ack of getting burst count parameter
   * of BURST mode, then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getPhotoBurstCountDecoder(
      Vehicle* vehicle, RecvContainer recvFrame,
      UCBRetParamHandler<PhotoBurstCount>* ucb);

  static void getPhotoAEBCountDecoder(Vehicle* vehicle, RecvContainer recvFrame,
                                      UCBRetParamHandler<PhotoAEBCount>* ucb);
  /*! @brief Decoder callback to decode the ack of getting parameters of
   * INTERVAL mode, then call the ucb
   *
   *  @param vehicle vehicle instance
   *  @param recvFrame data frame decoded from link layer
   *  @param ucb include the user callback and userdata defined by user, it will
   * be called once the data frame is valid.
   */
  static void getPhotoIntervalDatasDecoder(
      Vehicle* vehicle, RecvContainer recvFrame,
      UCBRetParamHandler<PhotoIntervalData>* ucb);

  static void callbackToSetTapZoomMultiplier(ErrorCode::ErrorCodeType retCode,
                                             bool enable, UserData userData);
  static void callbackToSetTapZoomEnabled(ErrorCode::ErrorCodeType retCode,
                                          TapZoomMultiplierData multiplier,
                                          UserData userData);
  static void callbackToSetShootPhotoMode(ErrorCode::ErrorCodeType retCode,
                                          CaptureParamData captureParam,
                                          UserData userData);
  static void callbackToSetPhotoBurstCount(ErrorCode::ErrorCodeType retCode,
                                           CaptureParamData captureParam,
                                           UserData userData);
  static void callbackToSetPhotoTimeIntervalSettings(
      ErrorCode::ErrorCodeType retCode, CaptureParamData captureParam,
      UserData userData);

  static const int maxSize = 32;

  UCBRetCodeHandler ucbHandler[maxSize];

  /*! @brief alloc memory for ucb handler
   *
   *  @details Mainly used in handling userData memory issues for the callback
   * in payloads. The userData recorded in vehicle->nbUserData will be called
   * when the callback is called. So this function is to avoid the userData to
   * be a invalid value in the callback.
   * @TODO Here is only the temporary way to alloc memory for the asynchronous
   * interface to stash the user data. This method will be optimized in the
   * future.
   * @Note There are 32 memory units for this method. So if the API calling
   * this method too fast, the memory units of this API may overflow. So the
   * calling frequency of releated APIs should less than 10Hz
   */
  UCBRetCodeHandler* allocUCBHandler(void* callback, UserData userData);

  template <typename AckT>
  static ErrorCode::ErrorCodeType commonDataUnpacker(RecvContainer recvFrame,
                                                     AckT& ack);

  template <typename DataT>
  void getInterfaceAsync(
      FuncParam req,
      void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                           UCBRetParamHandler<DataT>* ucb),
      void (*userCB)(ErrorCode::ErrorCodeType, DataT data, UserData userData),
      UserData userData, int timeout = 500, int retry_time = 2);

  template <typename AckT>
  ErrorCode::ErrorCodeType getInterfaceSync(FuncParam req, AckT& ack,
                                            int timeout);

  template <typename ReqT>
  void setInterfaceAsync(
      ReqT req,
      void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                           UCBRetCodeHandler* ucb),
      void (*userCB)(ErrorCode::ErrorCodeType, UserData userData),
      UserData userData, int timeout = 500, int retry_time = 2);

  template <typename ReqT>
  ErrorCode::ErrorCodeType setInterfaceSync(ReqT req, int timeout);

  template <typename ReqT>
  void actionInterfaceAsync(
      ReqT req,
      void (*ackDecoderCB)(Vehicle* vehicle, RecvContainer recvFrame,
                           UCBRetCodeHandler* ucb),
      void (*userCB)(ErrorCode::ErrorCodeType, UserData userData),
      UserData userData, int timeout = 500, int retry_time = 2);
  template <typename ReqT>
  ErrorCode::ErrorCodeType actionInterfaceSync(ReqT req, int timeout);

 private:
  PayloadLink* payloadLink;

  static ShutterSpeedType createShutterSpeedStruct(int reciprocal,
                                                   int integer_part,
                                                   int decimal_part);

  static ShutterSpeedType ShutterSpeedEnumToShutterSpeedType(
      ShutterSpeed shutterSpeed);

  static ShutterSpeed ShutterSpeedTypeToShutterSpeedEnum(int reciprocal,
                                                         int integer_part,
                                                         int decimal_part);

  void getCaptureParamDataAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CaptureParamData captureParam, UserData userData),
      UserData userData);

  ErrorCode::ErrorCodeType getCaptureParamDataSync(
      CaptureParamData& captureParam, int timeout);

}; /* CameraModule camera */
}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_CAMERA_MODULE_HPP
