/** @file dji_psdk_manager.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of the manager for psdk module
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

#ifndef ONBOARDSDK_DJI_PSDK_MANAGER_HPP
#define ONBOARDSDK_DJI_PSDK_MANAGER_HPP

#include <vector>
#include "dji_psdk_module.hpp"

namespace DJI {
namespace OSDK {
/*! @brief The manager of psdk module
 * @details This class support the transparent communication and widget values
 * modifying between OSDK and PSDK.
 */
class PSDKManager {
 public:
  PSDKManager(Vehicle *vehiclePtr);

  ~PSDKManager();

 public:
  /*! @brief init the psdk module
   *
   *  @note It should be paid attention that only one PSDK device
   * is supported in OSDK 3.9 version. It will support more PSDK
   * devices in the same time in the future
   *  @param index psdk module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param name psdk module name used in initialization
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType initPSDKModule(PayloadIndexType index,
                                          const char *name);

  /*! @brief deinit the psdk module
   *  In the deinit, the psdk module will set name to be
   *  defaultPSDKName and set enable to be false.
   *  @param index psdk module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType deinitPSDKModule(PayloadIndexType index);

  /*! @brief deinit all the psdk modules
   */
  void deinitAllPSDKModule(void);

  /*! @brief get the name of psdk module, searched by index
   *
   *  @param index psdk module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param name name of the psdk module, it's a output parameter. If get
   * fail, this parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPSDKModuleName(PayloadIndexType index,
                                             std::string &name);

  /*! @brief get the index of psdk module, searched by name
   *
   *  @param name name of psdk module
   *  @param index psdk module index, see enum DJI::OSDK::PayloadIndexType. If
   * get fail, this parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPSDKModuleIndex(const char *name, uint8_t &index);

  /*! @brief get the enable status of psdk module, searched by index
   *
   *  @param index psdk module index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param enable get the enable status of PSDKModule. If get fail, this
   * parameter will not do any assignment
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getPSDKModuleEnable(PayloadIndexType index,
                                               bool &enable);

 public:
  /*! @brief Sample to configure the value, blocking
   *
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param widgetIndex the index of target widget
   *  @param widgetType the type of target widget, ref to
   * DJI::OSDK::PSDKModule::PayloadWidgetType
   *  @param widgetValue the value of target widget
   *  @param timeout timeout time in seconds to request
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType configureWidgetValueSync(
      PayloadIndexType index, uint8_t widgetIndex,
      PSDKModule::PayloadWidgetType widgetType, int widgetValue, int timeout);

  /*! @brief Sample to configure the value, non-blocking
   *
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param widgetIndex the index of target widget
   *  @param widgetType the type of target widget, ref to
   * DJI::OSDK::PSDKModule::PayloadWidgetType
   *  @param widgetValue the value of target widget
   *  @param UserCallBack UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void configureWidgetValueAsync(
      PayloadIndexType index, uint8_t widgetIndex,
      PSDKModule::PayloadWidgetType widgetType, int widgetValue,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to set the callback for widget values, non-blocking
   *
   *  @note When the widget values of target PSDK device is pushing to OSDK,
   *  the callback will be called and catch the widget values.
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param cb the callback to catch the widget values pushging.
   *  @param userData the userData to be called by cb
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType subscribePSDKWidgetValues(
      PayloadIndexType index, PSDKModule::PSDKWidgetValuesUserCallback cb,
      UserData userData);

  /*! @brief Sample to disable the callback for widget values, non-blocking
   *
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType unsubscribeWidgetValues(PayloadIndexType index);

  /*! @brief used in internal to do suhscribing task for PSDK widget
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return handler including callback and userdata to do psdk widget
   *  subscription related deocding.
   */
  VehicleCallBackHandler *getSubscribeWidgetValuesHandler(
      PayloadIndexType index);

  /*! @brief Sample to set the callback for PSDK commonication data,
   * non-blocking
   *
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param cb the callback to catch the communication data from PSDK.
   *  @param userData the userData to be called by cb
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType subscribePSDKCommonication(
      PayloadIndexType index, PSDKModule::PSDKCommunicationUserCallback cb,
      UserData userData);

  /*! @brief Sample to disable the callback for PSDK commonication data,
   * non-blocking
   *
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType unsubscribePSDKCommonication(PayloadIndexType index);

  /*! @brief used in internal to do suhscribing task for PSDK commonication data
   *
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return handler including callback and userdata to do PSDK commonication
   * data subscription related deocding.
   */
  VehicleCallBackHandler *getCommunicationHandler(PayloadIndexType index);

  /*! @brief sending data from OSDK to PSDK
   *
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @param data sent data
   *  @param len length of data
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType sendDataToPSDK(PayloadIndexType index, uint8_t *data,
                                          uint16_t len);

 private:
  PayloadLink *payloadLink;
  std::vector<PSDKModule *> psdkModuleVector;

  /*! @brief get psdk module by index
   *  @param index payload node index, input limit see enum
   * DJI::OSDK::PayloadIndexType
   *  @return address of psdk module
   */
  PSDKModule *getPSDKModule(PayloadIndexType index);

  /*! @brief get psdk module by module name
   *  @param name target name of module
   *  @return address of psdk module
   */
  PSDKModule *getPSDKModule(std::string name);

  /*! @note default name of psdk module */
  const char *defaultPSDKName = "uninitialized_psdk";
};

}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_PSDK_MANAGER_HPP
