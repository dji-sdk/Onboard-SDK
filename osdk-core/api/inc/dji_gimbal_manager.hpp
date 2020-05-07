/** @file dji_gimbal_manager.hpp
 *  @version 4.0
 *  @date November 2019
 *
 *  @brief Implementation of the manager for gimbal module
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

#ifndef ONBOARDSDK_DJI_GIMBAL_MANAGER_HPP
#define ONBOARDSDK_DJI_GIMBAL_MANAGER_HPP

#include <vector>
#include "dji_gimbal_module.hpp"

namespace DJI {
namespace OSDK {
/*! @brief The manager of gimbal module
 */
class GimbalManager {
 public:
  GimbalManager(Vehicle *vehiclePtr);

  ~GimbalManager();

 public:
  /*! @brief init the gimbal module
   *
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param name gimbal module name used in initialization
   *  @return ErrorCode::ErrorCodeType error code
   */

  ErrorCode::ErrorCodeType initGimbalModule(PayloadIndexType index,
                                          const char *name);
  /*! @brief Deinit the gimbal module. It is a opposite operation
   *  to initGimbalModule. It means this gimbal will disable.
   *  In the deinit, the gimbal module will set name to be
   *  defaultGimbalName and set enable to be false.
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType deinitGimbalModule(PayloadIndexType index);

  /*! @brief deinit all the gimbal modules
   */
  void deinitAllGimbalModule(void);

  /*! @brief get the name of gimbal module, searched by index
   *
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param name name of the gimbal module, it's a output parameter. If get
   * fail, this parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getGimbalModuleName(PayloadIndexType index,
                                             std::string &name);

  /*! @brief get the index of gimbal module, searched by name
   *
   *  @param name name of gimbal module
   *  @param index gimbal module index, see enum DJI::OSDK::PayloadIndexType. If
   * get fail, this parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getGimbalModuleIndex(const char *name, uint8_t &index);

  /*! @brief get the enable status of gimbal module, searched by index
   *
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param enable get the enable status of GimbalModule. If get fail, this
   * parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getGimbalModuleEnable(PayloadIndexType index,
                                               bool &enable);

  /*! @brief reset the pitch and yaw of the gimbal, non-blocking calls
   *
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void resetAsync(PayloadIndexType index,
                  void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                       UserData userData),
                  UserData userData);

  /*! @brief reset the pitch and yaw of the gimbal, blocking calls
   *
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param timeout blocking timeout in seconds.
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType resetSync(
      PayloadIndexType index, int timeout);

  /*! @brief rotate the angle of the gimbal, non-blocking calls
   *
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param rotation the rotation parameters to be executed on the target
   * gimbal, including the rotation mode, target angle value and executed
   * time, ref to DJI::OSDK::GimbalModule::Rotation
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode is the ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void rotateAsync(PayloadIndexType index, GimbalModule::Rotation rotation,
                   void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                        UserData userData),
                   UserData userData);

  /*! @brief rotate the angle of the gimbal, blocking calls
   *
   *  @param index gimbal module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param rotation the rotation parameters to be executed on the target
   * gimbal, including the rotation mode, target angle value and executed
   * time, ref to DJI::OSDK::GimbalModule::Rotation
   *  @param timeout blocking timeout in seconds.
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType rotateSync(
      PayloadIndexType index, GimbalModule::Rotation rotation, int timeout);
 private:
  Linker *linker;

  std::vector<GimbalModule *> gimbalModuleVector;

  GimbalModule *getGimbalModule(PayloadIndexType index);

  GimbalModule *getGimbalModule(std::string name);

  const char *defaultGimbalName = "uninitialized_gimbal";
};

}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_GIMBAL_MANAGER_HPP
