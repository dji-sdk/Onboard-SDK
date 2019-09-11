/** @file dji_psdk_module.cpp
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

#include "dji_psdk_module.hpp"

#include <vector>
using namespace DJI;
using namespace DJI::OSDK;

void PSDKModule::widgetValueDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                    UserData userData) {
  std::vector<PSDKModule::PSDKWidgetInfo> widgets;
  int rawDataLen = recvFrame.recvInfo.len - OpenProtocol::PackageMin;
  PSDKWidgetValuesData *widgetsData =
      (PSDKWidgetValuesData *)recvFrame.recvData.raw_ack_array;

  if ((rawDataLen >= 2) &&
      (widgetsData->widgetLocalCount <= psdkWidgetMaxCount) &&
      ((rawDataLen - 2) >=
       widgetsData->widgetLocalCount * sizeof(PSDKWidgetInfo))) {
    DDEBUG("widget count: (%d/%d)", widgetsData->widgetLocalCount,
           widgetsData->widgetTotalCount);
    for (int i = 0; i < widgetsData->widgetLocalCount; i++) {
      widgets.push_back(widgetsData->widgets[i]);
      DDEBUG("widget index:%d type:%d value:%d", widgetsData->widgets[i].index,
             widgetsData->widgets[i].type, widgetsData->widgets[i].value);
    }
    if (userData) {
      PSDKModule *module = (PSDKModule *)userData;
      if (module->psdkWidgetValuesUserHandler.callback)
        module->psdkWidgetValuesUserHandler.callback(
            widgets, module->psdkWidgetValuesUserHandler.userData);
    } else {
      DERROR("userData is not a valid value");
      return;
    }
  } else {
    DERROR("Received wrong data length.");
    return;
  }
}

void PSDKModule::commonicationDataDecoder(Vehicle *vehicle,
                                          RecvContainer recvFrame,
                                          UserData userData) {
  uint16_t rawDataLen = recvFrame.recvInfo.len - OpenProtocol::PackageMin;
  uint8_t *rawDataBuf = recvFrame.recvData.raw_ack_array;
  if (userData) {
    PSDKModule *module = (PSDKModule *)userData;
    if (module->psdkCommonicationUserHandler.callback)
      module->psdkCommonicationUserHandler.callback(
          rawDataBuf, rawDataLen,
          module->psdkCommonicationUserHandler.userData);
  } else {
    DERROR("userData is not a valid value");
    return;
  }
}

PSDKModule::PSDKModule(PayloadLink *payloadLink, PayloadIndexType payloadIndex,
                       std::string name, bool enable)
    : PayloadBase(payloadIndex, name, enable), payloadLink(payloadLink) {
  psdkWidgetValuesDecodeHandler.callback = widgetValueDecoder;
  psdkWidgetValuesDecodeHandler.userData = this;
  psdkCommonicationDecodeHandler.callback = commonicationDataDecoder;
  psdkCommonicationDecodeHandler.userData = this;
}

PSDKModule::~PSDKModule() {}

ErrorCode::ErrorCodeType PSDKModule::configureWidgetValueSync(
    uint8_t widgetIndex, PayloadWidgetType widgetType, int widgetValue,
    int timeout) {
  PSDKWidgetReq req;
  req.funcIndex = FUNCTION_SET_PSDK_1_0_WIDGET_VALUE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.widgetInfo.type = widgetType;
  req.widgetInfo.index = widgetIndex;
  req.widgetInfo.value = widgetValue;
  if (getEnable()) {
    ACK::ExtendedFunctionRsp *rsp = payloadLink->sendSync(
        OpenProtocolCMD::CMDSet::Control::extendedFunction, &req,
        sizeof(PSDKWidgetReq), timeout);
    if (rsp->updated && rsp->info.buf &&
        (rsp->info.len - OpenProtocol::PackageMin >= sizeof(PSDKWidgetRsp))) {
      return ErrorCode::getErrorCode(
          ErrorCode::PSDKModule, ErrorCode::PSDKCommon,
          ((PSDKWidgetRsp *)rsp->info.buf)->ret_code);
    } else {
      if (!rsp->updated)
        return ErrorCode::SysCommonErr::ReqTimeout;
      else
        return ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
  }
  return ErrorCode::SysCommonErr::ReqNotSupported;
}

/*! @TODO Here is only the temporary way to alloc memory for the asynchronous
 * interface to stash the user data. This method will be optimized in the
 * future.
 */
PSDKModule::UCBRetCodeHandler *PSDKModule::allocUCBHandler(void *callback,
                                                           UserData userData) {
  static int ucbHandlerIndex = 0;

  ucbHandlerIndex++;
  if (ucbHandlerIndex >= maxSize) {
    ucbHandlerIndex = 0;
  }
  ucbHandler[ucbHandlerIndex].UserCallBack =
      (void (*)(ErrorCode::ErrorCodeType errCode, UserData userData))callback;
  ucbHandler[ucbHandlerIndex].userData = userData;
  return &(ucbHandler[ucbHandlerIndex]);
}

void PSDKModule::PSDKSetWidgetDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                      UCBRetCodeHandler *ucb) {
  if (ucb && ucb->UserCallBack) {
    PSDKWidgetRsp ack = {0};
    ErrorCode::ErrorCodeType ret = 0;
    if (recvFrame.recvInfo.len - OpenProtocol::PackageMin >=
        sizeof(PSDKWidgetRsp)) {
      ack = *(PSDKWidgetRsp *)(recvFrame.recvData.raw_ack_array);
      ret = ErrorCode::getErrorCode(ErrorCode::PSDKModule,
                                    ErrorCode::PSDKCommon, ack.ret_code);
    } else {
      DERROR("ACK is exception, data len %d (expect >= %d)\n",
             recvFrame.recvInfo.len - OpenProtocol::PackageMin,
             sizeof(PSDKWidgetRsp));
      ret = ErrorCode::SysCommonErr::UnpackDataMismatch;
    }
    ucb->UserCallBack(ret, ucb->userData);
  }
}

void PSDKModule::configureWidgetValueAsync(
    uint8_t widgetIndex, PayloadWidgetType widgetType, int widgetValue,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  PSDKWidgetReq req;
  req.funcIndex = FUNCTION_SET_PSDK_1_0_WIDGET_VALUE;
  req.payloadNodeIndex = (uint8_t)(this->getIndex() + 1);
  req.widgetInfo.type = widgetType;
  req.widgetInfo.index = widgetIndex;
  req.widgetInfo.value = widgetValue;
  if (getEnable()) {
    payloadLink->sendAsync(OpenProtocolCMD::CMDSet::Control::extendedFunction,
                           &req, sizeof(req), (void *)PSDKSetWidgetDecoder,
                           allocUCBHandler((void *)UserCallBack, userData), 500,
                           2);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::ReqNotSupported, userData);
  }
}

ErrorCode::ErrorCodeType PSDKModule::subscribePSDKWidgetValues(
    PSDKWidgetValuesUserCallback cb, UserData userData) {
  if (!getEnable()) {
    DERROR(
        "PSDK module [%d] is not initialized, this function is not supported "
        "now.",
        this->getIndex());
    return ErrorCode::SysCommonErr::ReqNotSupported;
  }
  if (!cb) {
    DERROR("The user callback is null value.");
    return ErrorCode::SysCommonErr::UserCallbackInvalid;
  }
  psdkWidgetValuesUserHandler.callback = cb;
  psdkWidgetValuesUserHandler.userData = userData;
  return ErrorCode::SysCommonErr::Success;
}

ErrorCode::ErrorCodeType PSDKModule::unsubscribeWidgetValues() {
  if (!getEnable()) {
    DERROR(
        "PSDK module [%d] is not initialized, this function is not supported "
        "now.",
        this->getIndex());
    return ErrorCode::SysCommonErr::ReqNotSupported;
  }
  psdkWidgetValuesUserHandler.callback = 0;
  psdkWidgetValuesUserHandler.userData = 0;
  return ErrorCode::SysCommonErr::Success;
}

VehicleCallBackHandler *PSDKModule::getSubscribeWidgetValuesHandler() {
  return &psdkWidgetValuesDecodeHandler;
}

ErrorCode::ErrorCodeType PSDKModule::subscribePSDKCommonication(
    PSDKModule::PSDKCommunicationUserCallback cb, UserData userData) {
  if (!getEnable()) {
    DERROR(
        "PSDK module [%d] is not initialized, this function is not supported "
        "now.",
        this->getIndex());
    return ErrorCode::SysCommonErr::ReqNotSupported;
  }
  if (!cb) {
    DERROR("The user callback is null value.");
    return ErrorCode::SysCommonErr::UserCallbackInvalid;
  }
  psdkCommonicationUserHandler.callback = cb;
  psdkCommonicationUserHandler.userData = userData;
  return ErrorCode::SysCommonErr::Success;
}

ErrorCode::ErrorCodeType PSDKModule::unsubscribePSDKCommonication() {
  if (!getEnable()) {
    DERROR(
        "PSDK module [%d] is not initialized, this function is not supported "
        "now.",
        this->getIndex());
    return ErrorCode::SysCommonErr::ReqNotSupported;
  }
  psdkCommonicationUserHandler.callback = 0;
  psdkCommonicationUserHandler.userData = 0;
  return ErrorCode::SysCommonErr::Success;
}

VehicleCallBackHandler *PSDKModule::getCommunicationHandler() {
  return &psdkCommonicationDecodeHandler;
}

ErrorCode::ErrorCodeType PSDKModule::sendDataToPSDK(uint8_t *data,
                                                    uint16_t len) {
  if (!getEnable()) {
    DERROR(
        "PSDK module [%d] is not initialized, this function is not supported "
        "now.",
        this->getIndex());
    return ErrorCode::SysCommonErr::ReqNotSupported;
  }
  if (len > MAX_SIZE_OF_PACKAGE) len = MAX_SIZE_OF_PACKAGE;
  this->payloadLink->sendToPSDK(data, len);
  return ErrorCode::SysCommonErr::Success;
}