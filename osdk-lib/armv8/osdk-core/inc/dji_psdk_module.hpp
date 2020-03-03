/** @file dji_psdk_module.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of psdk module for payload node
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

#ifndef ONBOARDSDK_DJI_PSDK_MODULE_HPP
#define ONBOARDSDK_DJI_PSDK_MODULE_HPP

#include <vector>
#include "dji_payload_base.hpp"
#include "dji_payload_link.hpp"

namespace DJI {
namespace OSDK {
/*! @brief PSDK module
 */
class PSDKModule : public PayloadBase {
 public:
  enum FunctionID {
    FUNCTION_SET_PSDK_1_0_WIDGET_VALUE = 70,
  };
  const static uint8_t psdkWidgetMaxCount = 30;
  const static uint16_t MAX_SIZE_OF_PACKAGE = 255;

  enum PayloadWidgetType {
    BUTTON = 1, /*!<  Button widget could return the widget value of 1 or 0,
                   normally it's widget value is 0, when the button is being
                   pressed the widget value will change to 1. */
    SWITCH = 2, /*!<  Switch widget could return the widget value of 1 or 0,
                   when the switch is changed to "ON" state, the widget value
                   will change to 1, if the switch is changed to "OFF" state,
                   the widget value will change to 0. */
    RANGE = 3,  /*!<  Range widget could return the widget value between 0 - 100
                   (inclusive). */
    LIST = 4,  /*!< List widget could return the int value between 0 and N, N is
                  determined by the firmware, the size of sub items returned by
                  getSubItems() method in PayloadWidget will be N+1 */
    INPUT = 5, /*!< Input widget could be set with any int value, the default
                  value is determined by the firmware. */
    UNKNOWN = 0xFF, /*!< Unknown widget type */
  };
#pragma pack(1)
  typedef struct PSDKWidgetInfo {
    uint8_t type;    /*!< psdk widget type */
    uint8_t index;   /*!< psdk widget index, ref to PayloadWidgetType */
    int32_t value;   /*!< psdk widget value */
  } PSDKWidgetInfo;  // pack(1)

  typedef struct PSDKWidgetReq {
    uint8_t funcIndex;        /*!< ref to DJI::OSDK::PSDKModule::FunctionID */
    uint8_t payloadNodeIndex; /*!< PayloadNode index. It should be paid
                                   attention that real payload node index +1 */
    PSDKWidgetInfo widgetInfo;
  } PSDKWidgetReq;  // pack(1)

  typedef struct PSDKWidgetRsp {
    uint8_t funcIndex;        /*!< ref to DJI::OSDK::PSDKModule::FunctionID */
    uint8_t payloadNodeIndex; /*!< PayloadNode index. It should be paid
                                   attention that real payload node index +1 */
    uint8_t ret_code;         /*!< original return code from vehicle */
  } PSDKWidgetRsp;            // pack(1)

  /*! @brief Capturing PushData of PSDK widget values, CMD: 0x00, 0x07
   */
  typedef struct PSDKWidgetValuesData {
    uint8_t widgetTotalCount; /*!< total widget count of current psdk device */
    uint8_t widgetLocalCount; /*!< local widget count of this package */
    PSDKWidgetInfo widgets[psdkWidgetMaxCount];
  } PSDKWidgetValuesData;  // pack(1)
#pragma pack()

  typedef void (*PSDKWidgetValuesUserCallback)(
      std::vector<PSDKWidgetInfo> widgets, UserData userData);

  typedef struct PSDKWidgetValuesUserHandler {
    PSDKWidgetValuesUserCallback callback;
    UserData userData;
  } PSDKWidgetValuesUserHandler;

  typedef void (*PSDKCommunicationUserCallback)(uint8_t *dataBuf,
                                                uint16_t dataLen,
                                                UserData userData);

  typedef struct PSDKCommonicationUserHandler {
    PSDKCommunicationUserCallback callback;
    UserData userData;
  } PSDKCommonicationUserHandler;

 public:
  PSDKModule(PayloadLink *payloadLink, PayloadIndexType payloadIndex,
             std::string name, bool enable);

  ~PSDKModule();

  /*! @brief Sample to configure the value, blocking
   *
   *  @param widgetIndex the index of target widget
   *  @param widgetType the type of target widget, ref to
   * DJI::OSDK::PSDKModule::PayloadWidgetType
   *  @param widgetValue the value of target widget
   *  @param timeout timeout time in seconds to request
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType configureWidgetValueSync(
      uint8_t widgetIndex, PayloadWidgetType widgetType, int widgetValue,
      int timeout);

  /*! @brief Sample to configure the value, non-blocking
   *
   *  @param widgetIndex the index of target widget
   *  @param widgetType the type of target widget, ref to
   * DJI::OSDK::PSDKModule::PayloadWidgetType
   *  @param widgetValue the value of target widget
   *  @param UserCallBack UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void configureWidgetValueAsync(
      uint8_t widgetIndex, PayloadWidgetType widgetType, int widgetValue,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Sample to set the callback for widget values, non-blocking
   *
   *  @note When the widget values of target PSDK device is pushing to OSDK,
   *  the callback will be called and catch the widget values.
   *  @param cb the callback to catch the widget values pushging.
   *  @param userData the userData to be called by cb
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType subscribePSDKWidgetValues(
      PSDKWidgetValuesUserCallback cb, UserData userData);

  /*! @brief Sample to disable the callback for widget values, non-blocking
   *
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType unsubscribeWidgetValues();

  /*! @brief used in internal to do suhscribing task for PSDK widget
   *
   *  @return handler including callback and userdata to do psdk widget
   *  subscription related deocding.
   */
  VehicleCallBackHandler *getSubscribeWidgetValuesHandler();

  /*! @brief Sample to set the callback for PSDK commonication data,
   * non-blocking
   *
   *  @param cb the callback to catch the communication data from PSDK.
   *  @param userData the userData to be called by cb
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType subscribePSDKCommonication(
      PSDKCommunicationUserCallback cb, UserData userData);

  /*! @brief Sample to disable the callback for PSDK commonication data,
   * non-blocking
   *
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType unsubscribePSDKCommonication();

  /*! @brief used in internal to do suhscribing task for PSDK commonication data
   *
   *  @return handler including callback and userdata to do PSDK commonication
   * data subscription related deocding.
   */
  VehicleCallBackHandler *getCommunicationHandler();

  /*! @brief sending data from OSDK to PSDK
   *
   *  @param data sent data
   *  @param len length of data
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType sendDataToPSDK(uint8_t *data, uint16_t len);

 private:
  static const int maxSize = 32;

  PayloadLink *payloadLink;

  /*! @brief handler to decoding the widget values raw data from raw data
   * package */
  VehicleCallBackHandler psdkWidgetValuesDecodeHandler;

  /*! @brief handler to decoding the communication data from raw data package */
  VehicleCallBackHandler psdkCommonicationDecodeHandler;

  /*! @brief handler to export widget values to user */
  PSDKWidgetValuesUserHandler psdkWidgetValuesUserHandler;

  /*! @brief handler to export commonication data to user */
  PSDKCommonicationUserHandler psdkCommonicationUserHandler;

  /*! @brief struct type used to temporarily stash the handler in the async
   * process */
  typedef struct UCBRetCodeHandler {
    void (*UserCallBack)(ErrorCode::ErrorCodeType errCode, UserData userData);
    UserData userData;
  } UCBRetCodeHandler;

  /*! @brief memory space used to temporarily stash the handler in the async
   * process */
  UCBRetCodeHandler ucbHandler[maxSize];

  /*! @brief alloc space used to temporarily stash the handler in the async
   * process */
  UCBRetCodeHandler *allocUCBHandler(void *callback, UserData userData);

  static void widgetValueDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                 UserData userData);

  static void PSDKSetWidgetDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                   UCBRetCodeHandler *ucb);

  static void commonicationDataDecoder(Vehicle *vehicle,
                                       RecvContainer recvFrame,
                                       UserData userData);
}; /* PSDKModule camera */
}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_PSDK_MODULE_HPP
