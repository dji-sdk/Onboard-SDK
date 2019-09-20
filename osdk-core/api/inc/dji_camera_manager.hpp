/** @file dji_camera_manager.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of the manager for camera module
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

#ifndef ONBOARDSDK_DJI_CAMERA_MANAGER_HPP
#define ONBOARDSDK_DJI_CAMERA_MANAGER_HPP

#include <vector>
#include "dji_camera_module.hpp"

namespace DJI {
namespace OSDK {
/*! @brief The manager of camera module
 */
class CameraManager {
 public:
  CameraManager(Vehicle *vehiclePtr);

  ~CameraManager();

 public:
  /*! @brief init the camera module
   *
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param name camera module name used in initialization
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType initCameraModule(PayloadIndexType index,
                                            const char *name);

  /*! @brief Deinit the camera module. It is a opposite operation
   *  to initCameraModule. It means this camera will disable.
   *  In the deinit, the camera module will set name to be
   *  defaultCameraName and set enable to be false.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType deinitCameraModule(PayloadIndexType index);

  /*! @brief deinit all the camera modules
   */
  void deinitAllCameraModule(void);

  /*! @brief get the name of camera module, searched by index
   *
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param name name of the camera module, it's a output parameter. If get
   * fail, this parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getCameraModuleName(PayloadIndexType index,
                                               std::string &name);

  /*! @brief get the index of camera module, searched by name
   *
   *  @param name name of camera module
   *  @param index camera module index, see enum DJI::OSDK::PayloadIndexType. If
   * get fail, this parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getCameraModuleIndex(const char *name,
                                                uint8_t &index);

  /*! @brief get the enable status of camera module, searched by index
   *
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param enable get the enable status of CameraModule. If get fail, this
   * parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getCameraModuleEnable(PayloadIndexType index,
                                                 bool &enable);

 public:
  /*! @brief start to shoot photo, non-blocking calls
   *
   *  @note Camera must be in ShootPhoto mode. For thermal imaging camera,
   * Single photo can be taken while recording video. The SD card state should
   * be checked before this method is used to ensure sufficient space exists.
   *  @note Starting to shoot photo need about 2 seconds.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note It should be paid attention that if request is timeout, the callback
   * will not be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::TakePhotoMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startShootPhotoAsync(
      PayloadIndexType index, CameraModule::ShootPhotoMode mode,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief start to shoot photo, blocking calls
   *
   *  @note Camera must be in ShootPhoto mode. For thermal imaging camera,
   * Single photo can be taken while recording video. The SD card state should
   * be checked before this method is used to ensure sufficient space exists.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::TakePhotoMode
   *  @param timeout blocking timeout in seconds. Starting to shoot photo need
   * about 2 seconds.
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startShootPhotoSync(
      PayloadIndexType index, CameraModule::ShootPhotoMode mode, int timeout);

  /*! @brief stop to shoot photo, non-blocking calls
   *
   *  @note  startShootPhoto has been invoked and the shoot mode is either
   * Interval or Time-lapse. If the capture mode is set to Single, the camera
   * will automatically stop taking the photo once the individual photo is
   * taken.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopShootPhotoAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief stop to shoot photo, blocking calls
   *
   *  @note  startShootPhoto has been invoked and the shoot mode is either
   * Interval or Time-lapse. If the capture mode is set to Single, the camera
   * will automatically stop taking the photo once the individual photo is
   * taken.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopShootPhotoSync(PayloadIndexType index,
                                              int timeout);

  /*! @brief set the shoot photo mode, non-blocking calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setShootPhotoModeAsync(
      PayloadIndexType index, CameraModule::ShootPhotoMode takePhotoMode,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the shoot photo mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setShootPhotoModeSync(
      PayloadIndexType index, CameraModule::ShootPhotoMode takePhotoMode,
      int timeout);

  /*! @brief get the shoot photo mode, non-blocking calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getShootPhotoModeAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::ShootPhotoMode takePhotoMode,
                           UserData userData),
      UserData userData);

  /*! @brief get the shoot photo mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param takePhotoMode take photo mode, input limit see enum
   * DJI::OSDK::CameraModule::ShootPhotoMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getShootPhotoModeSync(
      PayloadIndexType index, CameraModule::ShootPhotoMode &takePhotoMode,
      int timeout);

  /*! @brief set the burst count in the Burst take-photo mode, non-blocking
   * calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setPhotoBurstCountAsync(
      PayloadIndexType index, CameraModule::PhotoBurstCount count,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the burst count in the Burst take-photo mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setPhotoBurstCountSync(
      PayloadIndexType index, CameraModule::PhotoBurstCount count, int timeout);

  /*! get the burst count in the Burst take-photo mode, non-blocking calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getPhotoBurstCountAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::PhotoBurstCount count,
                           UserData userData),
      UserData userData);

  /*! @brief get the burst count in the Burst take-photo mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count burst photos count in the each burst photo taking
   * DJI::OSDK::CameraModule::PhotoBurstCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPhotoBurstCountSync(
      PayloadIndexType index, CameraModule::PhotoBurstCount &count,
      int timeout);

  /*! @brief set the burst count in the AEB take-photo mode, non-blocking calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setPhotoAEBCountAsync(
      PayloadIndexType index, CameraModule::PhotoAEBCount count,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the burst count in the AEB take-photo mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setPhotoAEBCountSync(
      PayloadIndexType index, CameraModule::PhotoAEBCount count, int timeout);

  /*! get the burst count in the AEB take-photo mode, non-blocking calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getPhotoAEBCountAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::PhotoAEBCount count,
                           UserData userData),
      UserData userData);

  /*! @brief get the burst count in the AEB take-photo mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param count burst photos count in the each AEB photo taking
   * DJI::OSDK::CameraModule::PhotoAEBCount
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPhotoAEBCountSync(
      PayloadIndexType index, CameraModule::PhotoAEBCount &count, int timeout);

  /*! @brief set the parameters in the INTERVAL take-photo mode, non-blocking
   * calls
   *
   *  @note When in this shoot-photo mode, The camera will capture a photo, wait
   * a specified interval of time, take another photo, and continue in this
   * manner until it has taken the required number of photos. Also supported by
   * thermal imaging camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setPhotoTimeIntervalSettingsAsync(
      PayloadIndexType index, CameraModule::PhotoIntervalData intervalSetting,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set the parameters in the INTERVAL take-photo mode, blocking calls
   *
   *  @note When in this shoot-photo mode, The camera will capture a photo, wait
   * a specified interval of time, take another photo, and continue in this
   * manner until it has taken the required number of photos. Also supported by
   * thermal imaging camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setPhotoTimeIntervalSettingsSync(
      PayloadIndexType index, CameraModule::PhotoIntervalData intervalSetting,
      int timeout);

  /*! @brief get the parameters in the INTERVAL take-photo mode, non-blocking
   * calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getPhotoIntervalDatasAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::PhotoIntervalData intervalSetting,
                           UserData userData),
      UserData userData);

  /*! @brief get the parameters in the INTERVAL take-photo mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param intervalSetting parameters in the INTERVAL take-photo mode,
   * including photo number and time interval
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPhotoIntervalDatasSync(
      PayloadIndexType index, CameraModule::PhotoIntervalData &intervalSetting,
      int timeout);

  /*! @brief start to take video, non-blocking calls
   *
   *  @note Camera must be in RECORD_VIDEO mode. For thermal imaging camera,
   * user can take Single photo when recording video.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void startRecordVideoAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief start to take video, blocking calls
   *
   *  @note Camera must be in RECORD_VIDEO mode. For thermal imaging camera,
   * user can take Single photo when recording video.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startRecordVideoSync(PayloadIndexType index,
                                                int timeout);

  /*! @brief stop to take video, non-blocking calls
   *
   *  @note Precondition: The camera is recording currently.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopRecordVideoAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief stop to take video, blocking calls
   *
   *  @note Precondition: The camera is recording currently.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopRecordVideoSync(PayloadIndexType index,
                                               int timeout);

  /*! @brief set camera working mode, non-blocking calls
   *
   *  @note Sets the camera's work mode to taking pictures, video, playback or
   * download. Please note that you cannot change the mode when a certain task
   * is executing, such as taking photo(s), recording video, or downloading and
   * saving files. Also supported by thermal imaging camera.
   *  @note In Onboard SDK, only taking pictures and video are supported. Change
   * work mode need about 2 seconds.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode camera working mode, input limit see enum
   * DJI::OSDK::CameraModule::WorkMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setModeAsync(PayloadIndexType index, CameraModule::WorkMode mode,
                    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                         UserData userData),
                    UserData userData);

  /*! @brief set camera working mode, blocking calls
   *
   *  @note Sets the camera's work mode to taking pictures, video, playback or
   * download. Please note that you cannot change the mode when a certain task
   * is executing, such as taking photo(s), recording video, or downloading and
   * saving files. Also supported by thermal imaging camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note In Onboard SDK, only taking pictures and video are supported. Change
   * work mode need about 2 seconds.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode camera working mode, input limit see enum
   * DJI::OSDK::CameraModule::WorkMode
   *  @param timeout blocking timeout in seconds, here the timeout value should
   * be >= 2
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setModeSync(PayloadIndexType index,
                                       CameraModule::WorkMode mode,
                                       int timeout);

  /*! @brief get camera working mode, non-blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b workingMode used as an input param, please see enum
   * DJI::OSDK::CameraModule::WorkMode
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getModeAsync(PayloadIndexType index,
                    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                         CameraModule::WorkMode workingMode,
                                         UserData userData),
                    UserData userData);

  /*! @brief get camera working mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param workingMode used as an output param, camera working mode, input
   * limit see enum DJI::OSDK::CameraModule::WorkMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getModeSync(PayloadIndexType index,
                                       CameraModule::WorkMode &workingMode,
                                       int timeout);

  /*! @brief set camera focus mode, non-blocking calls
   *
   *  @note Sets the lens focus mode. When the focus mode is auto, the target
   *  point is the focal point. When the focus mode is manual, the target point
   *  is the zoom out area if the focus assistant is enabled for the manual
   *  mode. Supported only by the X5, X5R, Z3 cameras, Mavic Pro camera,
   *  Phantom 4 Pro camera, Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise
   *  Camera, X5S. It's should be attention that X4S will keep focus point as
   *  (0.5,0.5) all the time, the setting of focus point to X4S will quickly
   *  replaced by (0.5, 0.5).
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode camera focus mode, input limit see enum
   * DJI::OSDK::CameraModule::FocusMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setFocusModeAsync(PayloadIndexType index, CameraModule::FocusMode mode,
                         void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief get camera focus mode, blocking calls
   *
   *  @note Sets the lens focus mode. When the focus mode is auto, the target
   *  point is the focal point. When the focus mode is manual, the target point
   *  is the zoom out area if the focus assistant is enabled for the manual
   *  mode. Supported only by the X5, X5R, Z3 cameras, Mavic Pro camera,
   *  Phantom 4 Pro camera, Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise
   *  Camera, X5S. It's should be attention that X4S will keep focus point as
   *  (0.5,0.5) all the time, the setting of focus point to X4S will quickly
   *  replaced by (0.5, 0.5).
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode camera focus mode, input limit see enum
   * DJI::OSDK::CameraModule::FocusMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setFocusModeSync(PayloadIndexType index,
                                            CameraModule::FocusMode mode,
                                            int timeout);

  /*! @brief get camera focus mode, non-blocking calls
   *
   *  @note Gets the lens focus mode. Please check FocusMode. Supported only by
   * the X5, X5R, Z3 cameras, Mavic Pro camera, Z30, Phantom 4 Pro camera, X4S,
   * X5S, Mavic 2 Pro, Mavic 2 Zoom Camera and Mavic 2 Enterprise Camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b focusMode used as an input param, please see enum
   * DJI::OSDK::CameraModule::FocusMode
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getFocusModeAsync(PayloadIndexType index,
                         void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              CameraModule::FocusMode focusMode,
                                              UserData userData),
                         UserData userData);

  /*! @brief get camera focus mode, blocking calls
   *
   *  @note Gets the lens focus mode. Please check FocusMode. Supported only by
   * the X5, X5R, Z3 cameras, Mavic Pro camera, Z30, Phantom 4 Pro camera, X4S,
   * X5S, Mavic 2 Pro, Mavic 2 Zoom Camera and Mavic 2 Enterprise Camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param focusMode used as an output param, camera focus mode, input limit
   * see enum DJI::OSDK::CameraModule::FocusMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getFocusModeSync(PayloadIndexType index,
                                            CameraModule::FocusMode &focusMode,
                                            int timeout);

  /*! @brief set camera tap focus target point, non-blocking calls
   *
   *  @note Sets the lens focus target point. When the focus mode is auto, the
   * target point is the focal point. When the focus mode is manual, the target
   * point is the zoom out area if the focus assistant is enabled for the manual
   * mode. Supported only by the X5, X5R, Z3 cameras, Mavic Pro camera, Phantom
   * 4 Pro camera, Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise Camera,
   * X4S and X5S. camera, X4S and X5S.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param tapFocusPos the param of tap focus, including x,y value
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setFocusTargetAsync(
      PayloadIndexType index, CameraModule::TapFocusPosData tapFocusPos,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera focus point, blocking calls
   *
   *  @note Sets the lens focus target point. When the focus mode is auto, the
   * target point is the focal point. When the focus mode is manual, the target
   * point is the zoom out area if the focus assistant is enabled for the manual
   * mode. Supported only by the X5, X5R, Z3 cameras, Mavic Pro camera, Phantom
   * 4 Pro camera, Mavic 2 Pro, Mavic 2 Zoom Camera, Mavic 2 Enterprise Camera,
   * X4S and X5S. camera, X4S and X5S.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param tapFocusPos the param of tap focus, including x,y value
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setFocusTargetSync(
      PayloadIndexType index, CameraModule::TapFocusPosData tapFocusPos,
      int timeout);

  /*! @brief get camera tap focus target point, non-blocking calls
   *
   *  @note Gets the lens focus target point. Supported only by the X5,
   * X5R, Z3 cameras, Mavic Pro camera and Phantom 4 Pro camera, X4S, X5S, Mavic
   * 2 Pro, Mavic 2 Zoom Camera and Mavic 2 Enterprise Camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b tapFocusPos used as an input param, the param of tap focus,
   * including x,y value
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getFocusTargetAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType,
                           CameraModule::TapFocusPosData tapFocusPos,
                           UserData userData),
      UserData userData);

  /*! @brief get camera focus point, blocking calls
   *
   *  @note Gets the lens focus target point. Supported only by the X5,
   * X5R, Z3 cameras, Mavic Pro camera and Phantom 4 Pro camera, X4S, X5S, Mavic
   * 2 Pro, Mavic 2 Zoom Camera and Mavic 2 Enterprise Camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param tapFocusPos used as an output param, the param of tap focus,
   * including x,y value
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getFocusTargetSync(
      PayloadIndexType index, CameraModule::TapFocusPosData &tapFocusPos,
      int timeout);

  /*! @brief start camera optical zooming, non-blocking calls
   *
   *  @note Start changing the focal length of the lens in specified direction
   * with specified speed. Focal length change (zooming) will halt when maximum
   * or minimum focal lengths are reached, or stopContinuousOpticalZoom* is
   * called. It is only supported by X5, X5R and X5S camera on Osmo with lens
   * Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera, Mavic 2
   * Zoom Camera and Mavic 2 Enterprise Camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
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
      PayloadIndexType index, CameraModule::zoomDirectionData zoomDirection,
      CameraModule::zoomSpeedData zoomSpeed,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief start camera optical zooming, blocking calls
   *
   *  @note Start changing the focal length of the lens in specified direction
   * with specified speed. Focal length change (zooming) will halt when maximum
   * or minimum focal lengths are reached, or stopContinuousOpticalZoom* is
   * called. It is only supported by X5, X5R and X5S camera on Osmo with lens
   * Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3 camera, Z30 camera, Mavic 2
   * Zoom Camera and Mavic 2 Enterprise Camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param zoomDirection optical zoom direction, ref to
   * DJI::OSDK::CameraModule::ZoomDirection
   *  @param zoomSpeed optical zoom direction, ref to
   * DJI::OSDK::CameraModule::ZoomSpeed
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType startContinuousOpticalZoomSync(
      PayloadIndexType index, CameraModule::zoomDirectionData zoomDirection,
      CameraModule::zoomSpeedData zoomSpeed, int timeout);

  /*! @brief stop camera optical zooming, non-blocking calls
   *
   *  @note Called to stop focal length changing, when it currently is from
   * calling startContinuousOpticalZoom*. It is only supported by X5, X5R and
   * X5S camera on Osmo with lens Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3
   * camera, Z30 camera, Mavic 2 Zoom Camera and Mavic 2 Enterprise camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void stopContinuousOpticalZoomAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief stop camera optical zooming, blocking calls
   *
   *  @note Called to stop focal length changing, when it currently is from
   * calling startContinuousOpticalZoom*. It is only supported by X5, X5R and
   * X5S camera on Osmo with lens Olympus M.Zuiko ED 14-42mm f/3.5-5.6 EZ, Z3
   * camera, Z30 camera, Mavic 2 Zoom Camera and Mavic 2 Enterprise camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType stopContinuousOpticalZoomSync(PayloadIndexType index,
                                                         int timeout);

  /*! @brief set camera tap zoom function parameters, non-blocking calls
   *
   *  @note Enable/disable TapZoom. tapZoomAtTarget can only be called when
   * TapZoom is enabled. It is only supported Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param param tap zoom enable data.
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   *  @details It should be paid attention that, tap zoom have not getter API
   */
  void setTapZoomEnabledAsync(
      PayloadIndexType index, bool param,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera tap zoom parameters, blocking calls
   *
   *  @note Enable/disable TapZoom. tapZoomAtTarget can only be called when
   * TapZoom is enabled. It is only supported Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param param tap zoom enable data.
   *  @param timeout blocking timeout in seconds
   *  @details It should be paid attention that, tap zoom have not getter API
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setTapZoomEnabledSync(PayloadIndexType index,
                                                 bool param, int timeout);

  /*! @brief get camera tap zoom function parameters, non-blocking calls
   *
   *  @note Determines whether TapZoom is enabled. It is only supported by Z30
   * camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b param used as an input param, tap zoom enable data.
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getTapZoomEnabledAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, bool param,
                           UserData userData),
      UserData userData);

  /*! @brief get camera tap zoom parameters, blocking calls
   *
   *  @note Determines whether TapZoom is enabled. It is only supported by Z30
   * camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param param used as an output param, tap zoom enable data.
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getTapZoomEnabledSync(PayloadIndexType index,
                                                 bool &param, int timeout);

  /*! @brief set camera tap zoom function parameters, non-blocking calls
   *
   *  @note TapZoom uses a multiplier to change the zoom scale when called. The
   * final zoom scale for a TapZoom will be: Current Zoom Scale x Multiplier.
   * The multiplier range is [1,5]. A multiplier of 1 will not change the zoom.
   * When the multiplier is 1, the zoom scale will not change during TapZoom. It
   * is only supported by Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param param tap zoom multiplier data
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   *  @details It should be paid attention that, tap zoom have not getter API
   */
  void setTapZoomMultiplierAsync(
      PayloadIndexType index, CameraModule::TapZoomMultiplierData param,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera tap zoom parameters, blocking calls
   *
   *  @note TapZoom uses a multiplier to change the zoom scale when called. The
   * final zoom scale for a TapZoom will be: Current Zoom Scale x Multiplier.
   * The multiplier range is [1,5]. A multiplier of 1 will not change the zoom.
   * When the multiplier is 1, the zoom scale will not change during TapZoom. It
   * is only supported by Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param param tap multiplier multiplier data
   *  @param timeout blocking timeout in seconds
   *  @details It should be paid attention that, tap zoom have not getter API
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setTapZoomMultiplierSync(
      PayloadIndexType index, CameraModule::TapZoomMultiplierData param,
      int timeout);

  /*! @brief get camera tap zoom function parameters, non-blocking calls
   *
   *  @note Gets the multiplier for TapZoom. It is only supported by Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b param used as an input param, tap zoom multiplier data
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getTapZoomMultiplierAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::TapZoomMultiplierData param,
                           UserData userData),
      UserData userData);

  /*! @brief get camera tap zoom parameters, blocking calls
   *
   *  @note Gets the multiplier for TapZoom. It is only supported by Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param param used as an output param, tap zoom multiplier data
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getTapZoomMultiplierSync(
      PayloadIndexType index, CameraModule::TapZoomMultiplierData &param,
      int timeout);

  /*! @brief set camera tap zoom point, non-blocking calls
   *
   *  @note TapZoom at the target. It can be called only when TapZoom is
   * enabled. When a new target is set, the gimbal will rotate and locate the
   * target in the center of the screen. At the same time, the camera will zoom
   * by multiplying the TapZoom multiplier. It is only supported Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param tapZoomPos the param of tap zoom, including x,y value
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void tapZoomAtTargetAsync(
      PayloadIndexType index, CameraModule::TapZoomPosData tapZoomPos,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera tap zoom point, blocking calls
   *
   *  @note TapZoom at the target. It can be called only when TapZoom is
   * enabled. When a new target is set, the gimbal will rotate and locate the
   * target in the center of the screen. At the same time, the camera will zoom
   * by multiplying the TapZoom multiplier. It is only supported Z30 camera.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param tapZoomPos the param of tap zoom, including x,y value
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType tapZoomAtTargetSync(
      PayloadIndexType index, CameraModule::TapZoomPosData tapZoomPos,
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
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setExposureModeAsync(
      PayloadIndexType index, CameraModule::ExposureMode mode,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera exposure mode, blocking calls
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
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setExposureModeSync(PayloadIndexType index,
                                               CameraModule::ExposureMode mode,
                                               int timeout);

  /*! @brief get camera exposure mode, non-blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b mode exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @arg @b userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getExposureModeAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::ExposureMode mode, UserData userData),
      UserData userData);

  /*! @brief get camera exposure mode, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param mode used as an output param,exposure mode, input limit see enum
   * DJI::OSDK::CameraModule::ExposureMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getExposureModeSync(PayloadIndexType index,
                                               CameraModule::ExposureMode &mode,
                                               int timeout);

  /*! @brief set camera iso value, non-blocking calls
   *
   *  @note Sets the camera's ISO value. For the X5, X5R, Phantom 4 Pro camera,
   * X4S and X5S, the ISO value can be set for all modes. For the other cameras,
   * the ISO value can only be set when the camera exposure mode is in Manual
   * mode. For Mavic 2 Enterprise Dual, the ISO value is always AUTO.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param iso camera iso, input limit see enum
   * DJI::OSDK::CameraModule::ISOParameter
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setISOAsync(PayloadIndexType index, CameraModule::ISO iso,
                   void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                        UserData userData),
                   UserData userData);

  /*! @brief set camera iso value, blocking calls
   *
   *  @note Sets the camera's ISO value. For the X5, X5R, Phantom 4 Pro camera,
   * X4S and X5S, the ISO value can be set for all modes. For the other cameras,
   * the ISO value can only be set when the camera exposure mode is in Manual
   * mode. For Mavic 2 Enterprise Dual, the ISO value is always AUTO.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param iso camera iso, input limit see enum
   * DJI::OSDK::CameraModule::ISOParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setISOSync(PayloadIndexType index,
                                      CameraModule::ISO iso, int timeout);

  /*! @brief get camera iso value, non-blocking calls
   *
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b iso used as an input param, camera iso, input limit see enum
   * DJI::OSDK::CameraModule::ISOParameter
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getISOAsync(PayloadIndexType index,
                   void (*UserCallBack)(ErrorCode::ErrorCodeType,
                                        CameraModule::ISO iso,
                                        UserData userData),
                   UserData userData);

  /*! @brief get camera iso value, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param iso used as an output param, camera iso, input limit see enum
   * DJI::OSDK::CameraModule::ISOParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getISOSync(PayloadIndexType index,
                                      CameraModule::ISO &iso, int timeout);

  /*! @brief set camera aperture size value, non-blocking calls
   *
   *  @note The exposure mode ExposureMode must be in MANUAL or
   * APERTURE_PRIORITY. Supported only by the X5, X5R, X4S, X5S camera and Mavic
   * 2 Pro.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param aperture camera aperture size, input limit see enum
   * DJI::OSDK::CameraModule::ApertureSize
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setApertureAsync(PayloadIndexType index, CameraModule::Aperture aperture,
                        void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                             UserData userData),
                        UserData userData);

  /*! @brief set camera aperture size value, blocking calls
   *
   *  @note The exposure mode ExposureMode must be in MANUAL or
   * APERTURE_PRIORITY. Supported only by the X5, X5R, X4S, X5S camera and Mavic
   * 2 Pro.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param aperture camera aperture size, input limit see enum
   * DJI::OSDK::CameraModule::ApertureSize
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setApertureSync(PayloadIndexType index,
                                           CameraModule::Aperture aperture,
                                           int timeout);

  /*! @brief get camera aperture size value, non-blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b size used as an input param, camera aperture size, input limit see
   * enum DJI::OSDK::CameraModule::ApertureSize
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getApertureAsync(PayloadIndexType index,
                        void (*UserCallBack)(ErrorCode::ErrorCodeType,
                                             CameraModule::Aperture aperture,
                                             UserData userData),
                        UserData userData);

  /*! @brief get camera aperture size value, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param aperture used as an output param, camera aperture size, input limit
   * see enum DJI::OSDK::CameraModule::ApertureSize
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getApertureSync(PayloadIndexType index,
                                           CameraModule::Aperture &aperture,
                                           int timeout);

  /*! @brief set camera shutter value, non-blocking calls
   *
   *  @note Sets the camera shutter speed. The shutter speed should not be set
   * slower than the video frame rate when the camera's mode is RECORD_VIDEO.
   * For example, if the video frame rate is 30fps, the shutterSpeed must be <=
   * 1/30. Precondition: The shutter speed can be set only when the camera
   * exposure mode is Shutter mode or Manual mode.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param shutter the shutter mode and param of camera, input limit see enum
   *  DJI::OSDK::CameraModule::ShutterMode
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setShutterSpeedAsync(
      PayloadIndexType index, CameraModule::ShutterSpeed shutterSpeed,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera shutter value, blocking calls
   *
   *  @note Sets the camera shutter speed. The shutter speed should not be set
   * slower than the video frame rate when the camera's mode is RECORD_VIDEO.
   * For example, if the video frame rate is 30fps, the shutterSpeed must be <=
   * 1/30. Precondition: The shutter speed can be set only when the camera
   * exposure mode is Shutter mode or Manual mode.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param shutter the shutter mode and param of camera, input limit see enum
   *  DJI::OSDK::CameraModule::ShutterMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setShutterSpeedSync(
      PayloadIndexType index, CameraModule::ShutterSpeed shutterSpeed,
      int timeout);

  /*! @brief get camera shutter value, non-blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b shutterSpeed used as an input param, the shutter mode and param of
   * camera, input limit see enum DJI::OSDK::CameraModule::ShutterMode
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getShutterSpeedAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::ShutterSpeed shutterSpeed,
                           UserData userData),
      UserData userData);

  /*! @brief get camera shutter value, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param shutter used as an output param, the shutter mode and param of
   * camera, input limit see enum DJI::OSDK::CameraModule::ShutterMode
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getShutterSpeedSync(
      PayloadIndexType index, CameraModule::ShutterSpeed &shutterSpeed,
      int timeout);

  /*! @brief set camera EV value, non-blocking calls
   *
   *  @note Sets the camera's exposure compensation. In order to use this
   * function, set the camera exposure mode to shutter, program or aperture.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param ev ev parameter value of camera, input limit see enum
   * DJI::OSDK::CameraModule::EVParameter
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setExposureCompensationAsync(
      PayloadIndexType index, CameraModule::ExposureCompensation ev,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief set camera EV value, blocking calls
   *
   *  @note Sets the camera's exposure compensation. In order to use this
   * function, set the camera exposure mode to shutter, program or aperture.
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param ev ev parameter value of camera, input limit see enum
   * DJI::OSDK::CameraModule::EVParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setExposureCompensationSync(
      PayloadIndexType index, CameraModule::ExposureCompensation ev,
      int timeout);

  /*! @brief get camera EV value, non-blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @note It should be paid attention that if timeout, the callback will not
   * be called. This issue will be fixed in the future.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b ev used as an input param, ev parameter value of camera, input
   * limit see enum DJI::OSDK::CameraModule::EVParameter
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getExposureCompensationAsync(
      PayloadIndexType index,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           CameraModule::ExposureCompensation ev,
                           UserData userData),
      UserData userData);

  /*! @brief get camera EV value, blocking calls
   *
   *  @note All the APIs whose name ending with sync or async in this class
   * have a restriction on calling. All these API should not be called until
   * the previous request receives ack or timeout.
   *  @param index camera module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param ev used as an output param, ev parameter value of camera, input
   * limit see enum DJI::OSDK::CameraModule::EVParameter
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getExposureCompensationSync(
      PayloadIndexType index, CameraModule::ExposureCompensation &ev,
      int timeout);

 private:
  PayloadLink *payloadLink;
  std::vector<CameraModule *> cameraModuleVector;

  CameraModule *getCameraModule(PayloadIndexType index);
  CameraModule *getCameraModule(std::string name);

  /*! @note default name of camera module */
  const char *defaultCameraName = "uninitialized_camera";
};

}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_PAYLOAD_MANAGER_HPP
