//
// Created by dji on 4/7/20.
//

#ifndef ONBOARDSDK_INTERNAL_RESOURCES_SRC_OSDK_CORE_MODULES_INC_FILEMGR_IMPL_CSDKERRORCODE_HPP_
#define ONBOARDSDK_INTERNAL_RESOURCES_SRC_OSDK_CORE_MODULES_INC_FILEMGR_IMPL_CSDKERRORCODE_HPP_

namespace DJI {
namespace OSDK {

static const int kNoError = 0x00000;
static const int kErrorRequestHandlerNotFound = -0x00001;
static const int kErrorRequestNotSupportedByHandler = -0x00002;
static const int kErrorRequestTimeout = -0x00003;
static const int kErrorSendPackFailure = -0x00004;
static const int kErrorDisconnected = -0x00005;
static const int kErrorInvalidParam = -0x00006;
static const int kErrorSystemError = -0x00007; //在key的使用中，该错误码大多是由于设备返回了未能识别的错误码
static const int kErrorCommandInterrupted = -0x00008;
static const int kErrorParametersGetError = -0x00009;
static const int kErrorParametersSetError = -0x0000A;
static const int kErrorInvalidRespond = -0x0000B;
static const int kErrorParamOutOfRange = -0x0000C;
static const int kErrorInvalidRequestInCurrentState = -0x0000D;
static const int kErrorExecutionFailed = -0x0000E;
static const int kErrorNeedRequestAgain = -0x0000F; //需要再次发送请求
}
}

#endif //ONBOARDSDK_INTERNAL_RESOURCES_SRC_OSDK_CORE_MODULES_INC_FILEMGR_IMPL_CSDKERRORCODE_HPP_
