/*! @file camera_manager_async_sample.hpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief
 *  Demonstrate how to use the asynchronous apis of camera manager.
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

#ifndef ONBOARDSDK_CAMERA_MANAGER_ASYNC_SAMPLE_HPP
#define ONBOARDSDK_CAMERA_MANAGER_ASYNC_SAMPLE_HPP

#include <dji_vehicle.hpp>
#include "dji_camera_manager.hpp"

/*! @brief camera manager async sample
 */
class CameraManagerAsyncSample {
 public:
  CameraManagerAsyncSample(Vehicle* vehiclePtr);

  ~CameraManagerAsyncSample();

 public:
  /*! @brief Sample to set exposure mode for camera, using async api
   *
   *  @note In this interface, exposure will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target exposure mode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setExposureModeAsyncSample(
      PayloadIndexType index, CameraModule::ExposureMode dataTarget,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to set ISO value for camera, using async api
   *
   *  @note In this interface, ISO will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target ISO value
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setISOAsyncSample(PayloadIndexType index, CameraModule::ISO dataTarget,
                         void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Sample to set shutter speed for camera, using async api
   *
   *  @note In this interface, shutter speed will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target shutter speed
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setShutterSpeedAsyncSample(
      PayloadIndexType index, CameraModule::ShutterSpeed dataTarget,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to set shutter aperture value for camera, using async api
   *
   *  @note In this interface, aperture value will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target aperture value
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setApertureAsyncSample(
      PayloadIndexType index, CameraModule::Aperture dataTarget,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to set exposure compensation value for camera, using async
   * api
   *
   *  @note In this interface, exposure compensation value will be got then be
   * set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target exposure compensation value
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setEVAsyncSample(PayloadIndexType index,
                        CameraModule::ExposureCompensation dataTarget,
                        void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);

  /*! @brief Sample to set focus point for camera, using async api
   *
   *  @note In this interface, focus mode will be set to be AUTO. Then the
   * focus point will be set to be (x, y)
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param x the x value of target focus point, 0~1
   *  @param y the y value of target focus point, 0~1
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setFocusPointAsyncSample(
      PayloadIndexType index, float x, float y,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to set tap-zoom point for camera, using async api
   *
   *  @note In this interface, tap-zoom function will be enable and the
   * multiplier will be set. Then the tap-zoom function will start with the
   * target tap-zoom point (x, y)
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param multiplier the zoom multiplier of each tap zoom
   *  @param x the x value of target tap-zoom point, 0~1
   *  @param y the y value of target tap-zoom point, 0~1
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setTapZoomPointAsyncSample(
      PayloadIndexType index, uint8_t multiplier, float x, float y,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to execute continuous zoom on camera, using async api
   *
   *  @note It is only supported by X5, X5R and X5S camera on Osmo with lens
   * Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.
   *  @note In this interface, the zoom will start with the designated direction
   * and speed, and will stop after zoomTimeInSecond second(s).
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param direction the choice of zoom out or zoom in
   *  @param speed zooming speed
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startZoomAsyncSample(
      PayloadIndexType index, CameraModule::zoomDirectionData direction,
      CameraModule::zoomSpeedData speed,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to stop continuous zoom on camera, using async api
   *
   *  @note It is only supported by X5, X5R and X5S camera on Osmo with lens
   * Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.
   *  @note In this interface, the zoom will start with the designated direction
   * and speed, and will stop after zoomTimeInSecond second(s).
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopZoomAsyncSample(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to start record video, using async api
   *
   *  @note In this interface, camera will be set to be the RECORD_VIDEO mode
   * then start to record video.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startRecordVideoAsyncSample(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to stop record video, using async api
   *
   *  @note In this interface, camera will be set to be the RECORD_VIDEO mode
   * then stop recording video.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopRecordVideoAsyncSample(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to shoot single photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a single photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startShootSinglePhotoAsyncSample(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to shoot burst photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a burst photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count The number of pictures in each burst shooting
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startShootBurstPhotoAsyncSample(
      PayloadIndexType index, CameraModule::PhotoBurstCount count,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to shoot AEB photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a AEB photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param photoNum The number of pictures in each AEB shooting
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startShootAEBPhotoAsyncSample(
      PayloadIndexType index, CameraModule::PhotoAEBCount count,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to start shooting interval photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a interval photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param intervalData the parameter of interval shooting
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startShootIntervalPhotoAsyncSample(
      PayloadIndexType index, CameraModule::PhotoIntervalData intervalData,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to stop shooting, using async api
   *
   *  @note In this interface, camera will stop all the shooting action
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopShootPhotoAsyncSample(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

 private:
  Vehicle* vehicle;

  static const uint8_t sampleDataBufferLen = 100;
  typedef struct AsyncSampleData {
    PayloadIndexType index;
    CameraManager* pm;
    uint8_t dataTarget[sampleDataBufferLen];
    void* userCallBack;
    UserData userData;
  } AsyncSampleData;

  /*! @brief Callback of getExposureModeAsync, used in
   * setExposureModeAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param exposureModeGet exposure mode got from camera
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void getExposureModeCb(ErrorCode::ErrorCodeType retCode,
                                CameraModule::ExposureMode exposureModeGet,
                                UserData userData);

  /*! @brief Callback of getISOAsync, used in setISOAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param isoGet ISO value got from camera
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void getISOCb(ErrorCode::ErrorCodeType retCode,
                       CameraModule::ISO isoGet, UserData userData);

  /*! @brief Callback of getShutterSpeedAsync, used in
   * setShutterSpeedAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param shutterSpeedGet shutter speed parameter value got from camera
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void getShutterSpeedCb(ErrorCode::ErrorCodeType retCode,
                                CameraModule::ShutterSpeed shutterSpeedGet,
                                UserData userData);

  /*! @brief Callback of getApertureAsync, used in setApertureAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param apertureGet aperture parameter value got from camera
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void getApertureCb(ErrorCode::ErrorCodeType retCode,
                            CameraModule::Aperture apertureGet,
                            UserData userData);

  /*! @brief Callback of getExposureCompensationAsync, used in setEVAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param evGet exposure compensation parameter value got from camera
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void getEVCb(ErrorCode::ErrorCodeType retCode,
                      CameraModule::ExposureCompensation evGet,
                      UserData userData);

  /*! @brief Callback of setFocusModeAsync, used in setFocusPointAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setFocusModeCb(ErrorCode::ErrorCodeType retCode,
                             UserData userData);

  /*! @brief Callback of setTapZoomEnabledAsync, used in
   * setTapZoomPointAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setTapZoomEnableCb(ErrorCode::ErrorCodeType retCode,
                                 UserData userData);

  /*! @brief Callback of setTapZoomMultiplierAsync, used in
   * setTapZoomPointAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setTapZoomMultiplierCb(ErrorCode::ErrorCodeType retCode,
                                     UserData userData);

  /*! @brief Callback of setModeAsync, used in
   * startShootSinglePhotoAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setCameraModeForSingleShootCb(ErrorCode::ErrorCodeType retCode,
                                            UserData userData);

  /*! @brief Callback of setShootPhotoModeAsync, used in
   * setCameraModeForSingleShootCb
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setShootPhotoModeForSingleShootCb(
      ErrorCode::ErrorCodeType retCode, UserData userData);

  /*! @brief Callback of setModeAsync, used in
   * startShootBurstPhotoAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setCameraModeForBurstShootCb(ErrorCode::ErrorCodeType retCode,
                                           UserData userData);

  /*! @brief Callback of setShootPhotoModeAsync, used in
   * setCameraModeForBurstShootCb
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setShootPhotoModeForBurstShootCb(ErrorCode::ErrorCodeType retCode,
                                               UserData userData);

  /*! @brief Callback of setPhotoBurstCountAsync, used in
   * setShootPhotoModeForBurstShootCb
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setPhotoBurstCountCb(ErrorCode::ErrorCodeType retCode,
                                   UserData userData);

  /*! @brief Callback of setModeAsync, used in
   * startShootAEBPhotoAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setCameraModeForAEBShootCb(ErrorCode::ErrorCodeType retCode,
                                         UserData userData);

  /*! @brief Callback of setShootPhotoModeAsync, used in
   * setCameraModeForAEBShootCb
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setShootPhotoModeForAEBShootCb(ErrorCode::ErrorCodeType retCode,
                                             UserData userData);

  /*! @brief Callback of setPhotoAEBCountAsync, used in
   * setShootPhotoModeForAEBShootCb
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setPhotoAEBCountCb(ErrorCode::ErrorCodeType retCode,
                                 UserData userData);

  /*! @brief Callback of setModeAsync, used in
   * startShootIntervalPhotoAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setCameraModeForIntervalShootCb(ErrorCode::ErrorCodeType retCode,
                                              UserData userData);

  /*! @brief Callback of setShootPhotoModeAsync, used in
   * setCameraModeForIntervalShootCb
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setShootPhotoModeForIntervalShootCb(
      ErrorCode::ErrorCodeType retCode, UserData userData);

  /*! @brief Callback of setPhotoTimeIntervalSettingsAsync, used in
   * setShootPhotoModeForIntervalShootCb
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setPhotoIntervalCb(ErrorCode::ErrorCodeType retCode,
                                 UserData userData);

  /*! @brief Callback of setModeAsync, used in
   * startRecordVideoAsyncSample
   *
   *  @param retCode return code of the api, ErrorCode::ErrorCodeType error code
   *  @param userData the interface to pass userData in when the callback is
   * called
   */
  static void setCameraModeForRecordVideoCb(ErrorCode::ErrorCodeType retCode,
                                            UserData userData);
};

#endif  // ONBOARDSDK_CAMERA_MANAGER_ASYNC_SAMPLE_HPP
