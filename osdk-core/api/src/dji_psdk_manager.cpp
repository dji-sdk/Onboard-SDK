/** @file dji_psdk_manager.cpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of psdk module
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

#include "dji_psdk_manager.hpp"

using namespace DJI;
using namespace DJI::OSDK;

PSDKManager::PSDKManager(Vehicle *vehiclePtr) {
  payloadLink = new PayloadLink(vehiclePtr);
  for (int index = PAYLOAD_INDEX_0; index < PAYLOAD_INDEX_CNT; index++) {
    PSDKModule *module = new PSDKModule(payloadLink, (PayloadIndexType)index,
                                        defaultPSDKName, false);
    psdkModuleVector.push_back(module);
  }
}

PSDKManager::~PSDKManager() {
  for (int i = 0; i < psdkModuleVector.size(); i++) {
    if (psdkModuleVector[i]) {
      delete psdkModuleVector[i];
    }
  }
  delete payloadLink;
}

PSDKModule *PSDKManager::getPSDKModule(PayloadIndexType index) {
  for (int i = 0; i < psdkModuleVector.size(); ++i) {
    if (psdkModuleVector[i]->getIndex() == index) {
      return psdkModuleVector[i];
    }
  }
  return NULL;
}

PSDKModule *PSDKManager::getPSDKModule(std::string name) {
  for (int i = 0; i < psdkModuleVector.size(); ++i) {
    if (psdkModuleVector[i]->getName() == name) {
      return psdkModuleVector[i];
    }
  }
  return NULL;
}

ErrorCode::ErrorCodeType PSDKManager::initPSDKModule(PayloadIndexType index,
                                                     const char *name) {
  /* @TODO lock protest PSDKModule */
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    psdkMgr->setName(name);
    psdkMgr->setEnable(true);

    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType PSDKManager::deinitPSDKModule(PayloadIndexType index) {
  /* @TODO lock protest psdkMgr */
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    psdkMgr->setName(defaultPSDKName);
    psdkMgr->setEnable(false);
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void PSDKManager::deinitAllPSDKModule() {
  for (int i = 0; i < psdkModuleVector.size(); ++i) {
    psdkModuleVector[i]->setName(defaultPSDKName);
    psdkModuleVector[i]->setEnable(false);
  }
}

ErrorCode::ErrorCodeType PSDKManager::getPSDKModuleName(PayloadIndexType index,
                                                        std::string &name) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    name = psdkMgr->getName();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType PSDKManager::getPSDKModuleIndex(const char *name,
                                                         uint8_t &index) {
  PSDKModule *psdkMgr = getPSDKModule(name);
  if (psdkMgr) {
    index = psdkMgr->getIndex();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType PSDKManager::getPSDKModuleEnable(
    PayloadIndexType index, bool &enable) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    enable = psdkMgr->getEnable();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType PSDKManager::configureWidgetValueSync(
    PayloadIndexType index, uint8_t widgetIndex,
    PSDKModule::PayloadWidgetType widgetType, int widgetValue, int timeout) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->configureWidgetValueSync(widgetIndex, widgetType,
                                             widgetValue, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void PSDKManager::configureWidgetValueAsync(
    PayloadIndexType index, uint8_t widgetIndex,
    PSDKModule::PayloadWidgetType widgetType, int widgetValue,
    void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
    UserData userData) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    psdkMgr->configureWidgetValueAsync(widgetIndex, widgetType, widgetValue,
                                       UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType PSDKManager::subscribePSDKWidgetValues(
    PayloadIndexType index, PSDKModule::PSDKWidgetValuesUserCallback cb,
    UserData userData) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->subscribePSDKWidgetValues(cb, userData);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType PSDKManager::unsubscribeWidgetValues(
    PayloadIndexType index) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->unsubscribeWidgetValues();
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

VehicleCallBackHandler *PSDKManager::getSubscribeWidgetValuesHandler(
    PayloadIndexType index) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->getSubscribeWidgetValuesHandler();
  } else {
    return NULL;
  }
}

ErrorCode::ErrorCodeType PSDKManager::subscribePSDKCommonication(
    PayloadIndexType index, PSDKModule::PSDKCommunicationUserCallback cb,
    UserData userData) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->subscribePSDKCommonication(cb, userData);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType PSDKManager::unsubscribePSDKCommonication(
    PayloadIndexType index) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->unsubscribePSDKCommonication();
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

VehicleCallBackHandler *PSDKManager::getCommunicationHandler(
    PayloadIndexType index) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->getCommunicationHandler();
  } else {
    return NULL;
  }
}

ErrorCode::ErrorCodeType PSDKManager::sendDataToPSDK(PayloadIndexType index,
                                                     uint8_t *data,
                                                     uint16_t len) {
  PSDKModule *psdkMgr = getPSDKModule(index);
  if (psdkMgr) {
    return psdkMgr->sendDataToPSDK(data, len);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}