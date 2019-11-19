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

  ErrorCode::ErrorCodeType initGimbalModule(PayloadIndexType index,
                                          const char *name);

  ErrorCode::ErrorCodeType deinitGimbalModule(PayloadIndexType index);

  void deinitAllGimbalModule(void);

  ErrorCode::ErrorCodeType getGimbalModuleName(PayloadIndexType index,
                                             std::string &name);

  ErrorCode::ErrorCodeType getGimbalModuleIndex(const char *name, uint8_t &index);

  ErrorCode::ErrorCodeType getGimbalModuleEnable(PayloadIndexType index,
                                               bool &enable);

  void resetAsync(PayloadIndexType index,
                  void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                       UserData userData),
                  UserData userData);

  ErrorCode::ErrorCodeType resetSync(
      PayloadIndexType index, int timeout);

  void rotateAsync(PayloadIndexType index, GimbalModule::Rotation rotation,
                   void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                        UserData userData),
                   UserData userData);

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
