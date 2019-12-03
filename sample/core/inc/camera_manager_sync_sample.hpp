/*! @file camera_manager_sync_sample.hpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief
 *  Demonstrate how to use the synchronous apis of camera manager.
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

#ifndef ONBOARDSDK_CAMERA_MANAGER_SYNC_SAMPLE_HPP
#define ONBOARDSDK_CAMERA_MANAGER_SYNC_SAMPLE_HPP

#include <dji_vehicle.hpp>
#include "dji_camera_manager.hpp"

/*! @brief camera manager sync sample
 */
class CameraManagerSyncSample {
 public:
  CameraManagerSyncSample(Vehicle* vehiclePtr);

  ~CameraManagerSyncSample();

 public:
  /*! @brief Sample to set exposure compensation value for camera, using async
   * api
   *
   *  @note In this interface, exposure compensation value will be got then be
   * set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target exposure compensation value
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setEVSyncSample(
      PayloadIndexType index, CameraModule::ExposureCompensation dataTarget);

  /*! @brief Sample to set exposure mode for camera, using async api
   *
   *  @note In this interface, exposure will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target exposure mode
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setExposureModeSyncSample(
      PayloadIndexType index, CameraModule::ExposureMode dataTarget);

  /*! @brief Sample to set ISO value for camera, using async api
   *
   *  @note In this interface, ISO will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target ISO value
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setISOSyncSample(PayloadIndexType index,
                                            CameraModule::ISO dataTarget);

  /*! @brief Sample to set shutter speed for camera, using async api
   *
   *  @note In this interface, shutter speed will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target shutter speed
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setShutterSpeedSyncSample(
      PayloadIndexType index, CameraModule::ShutterSpeed dataTarget);

  /*! @brief Sample to set shutter aperture value for camera, using async api
   *
   *  @note In this interface, aperture value will be got then be set.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param dataTarget the target aperture value
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setApertureSyncSample(
      PayloadIndexType index, CameraModule::Aperture dataTarget);

  /*! @brief Sample to set focus point for camera, using async api
   *
   *  @note In this interface, focus mode will be set to be AUTO. Then the
   * focus point will be set to be (x, y)
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param x the x value of target focus point, 0~1
   *  @param y the y value of target focus point, 0~1
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setFocusPointSyncSample(PayloadIndexType index,
                                                   float x, float y);

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
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setTapZoomPointSyncSample(PayloadIndexType index,
                                                     uint8_t multiplier,
                                                     float x, float y);

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
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startZoomSyncSample(
      PayloadIndexType index, CameraModule::zoomDirectionData direction,
      CameraModule::zoomSpeedData speed);

  /*! @brief Sample to stop continuous zoom on camera, using async api
   *
   *  @note It is only supported by X5, X5R and X5S camera on Osmo with lens
   * Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopZoomSyncSample(PayloadIndexType index);

  /*! @brief Sample to shoot single photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a single photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startShootSinglePhotoSyncSample(
      PayloadIndexType index);

  /*! @brief Sample to shoot burst photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a burst photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count The number of pictures in each burst shooting
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startShootBurstPhotoSyncSample(
      PayloadIndexType index, CameraModule::PhotoBurstCount count);

  /*! @brief Sample to shoot AEB photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a AEB photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param photoNum The number of pictures in each AEB shooting
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startShootAEBPhotoSyncSample(
      PayloadIndexType index, CameraModule::PhotoAEBCount photoNum);

  /*! @brief Sample to start shooting interval photo, using async api
   *
   *  @note In this interface, camera will be set to be the SHOOT_PHOTO mode
   * then start to shoot a interval photo.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param intervalData the parameter of interval shooting
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startShootIntervalPhotoSyncSample(
      PayloadIndexType index, CameraModule::PhotoIntervalData intervalData);

  /*! @brief Sample to stop shooting, using async api
   *
   *  @note In this interface, camera will stop all the shooting action
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType shootPhotoStopSyncSample(PayloadIndexType index);

  /*! @brief Sample to start record video, using async api
   *
   *  @note In this interface, camera will be set to be the RECORD_VIDEO mode
   * then start to record video.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startRecordVideoSyncSample(PayloadIndexType index);

  /*! @brief Sample to stop record video, using async api
   *
   *  @note In this interface, camera will be set to be the RECORD_VIDEO mode
   * then stop recording video.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopRecordVideoSyncSample(PayloadIndexType index);

 private:
  Vehicle* vehicle;
};

#endif  // ONBOARDSDK_CAMERA_MANAGER_SYNC_SAMPLE_HPP
