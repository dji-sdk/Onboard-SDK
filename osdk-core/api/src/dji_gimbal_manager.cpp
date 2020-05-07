/** @file dji_gimbal_manager.cpp
 *  @version 4.0
 *  @date November 2019
 *
 *  @brief Implementation of gimbal module
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

#include <dji_vehicle.hpp>
#include "dji_gimbal_manager.hpp"

using namespace DJI;
using namespace DJI::OSDK;

GimbalManager::GimbalManager(Vehicle *vehiclePtr) {
  linker = vehiclePtr->linker;
  for (int index = PAYLOAD_INDEX_0; index < PAYLOAD_INDEX_CNT; index++) {
    GimbalModule *module = new GimbalModule(linker, (PayloadIndexType)index,
                                        defaultGimbalName, false);
    gimbalModuleVector.push_back(module);
  }
}

GimbalManager::~GimbalManager() {
  for (int i = 0; i < gimbalModuleVector.size(); i++) {
    if (gimbalModuleVector[i]) {
      delete gimbalModuleVector[i];
    }
  }
}

GimbalModule *GimbalManager::getGimbalModule(PayloadIndexType index) {
  for (int i = 0; i < gimbalModuleVector.size(); ++i) {
    if (gimbalModuleVector[i]->getIndex() == index) {
      return gimbalModuleVector[i];
    }
  }
  return NULL;
}

GimbalModule *GimbalManager::getGimbalModule(std::string name) {
  for (int i = 0; i < gimbalModuleVector.size(); ++i) {
    if (gimbalModuleVector[i]->getName() == name) {
      return gimbalModuleVector[i];
    }
  }
  return NULL;
}

ErrorCode::ErrorCodeType GimbalManager::initGimbalModule(PayloadIndexType index,
                                                     const char *name) {
  /* @TODO lock protest GimbalModule */
  GimbalModule *gimbalMgr = getGimbalModule(index);
  if (gimbalMgr) {
    gimbalMgr->setName(name);
    gimbalMgr->setEnable(true);

    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType GimbalManager::deinitGimbalModule(PayloadIndexType index) {
  /* @TODO lock protest gimbalMgr */
  GimbalModule *gimbalMgr = getGimbalModule(index);
  if (gimbalMgr) {
    gimbalMgr->setName(defaultGimbalName);
    gimbalMgr->setEnable(false);
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void GimbalManager::deinitAllGimbalModule() {
  for (int i = 0; i < gimbalModuleVector.size(); ++i) {
    gimbalModuleVector[i]->setName(defaultGimbalName);
    gimbalModuleVector[i]->setEnable(false);
  }
}

ErrorCode::ErrorCodeType GimbalManager::getGimbalModuleName(PayloadIndexType index,
                                                        std::string &name) {
  GimbalModule *gimbalMgr = getGimbalModule(index);
  if (gimbalMgr) {
    name = gimbalMgr->getName();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType GimbalManager::getGimbalModuleIndex(const char *name,
                                                         uint8_t &index) {
  GimbalModule *gimbalMgr = getGimbalModule(name);
  if (gimbalMgr) {
    index = gimbalMgr->getIndex();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

ErrorCode::ErrorCodeType GimbalManager::getGimbalModuleEnable(
    PayloadIndexType index, bool &enable) {
  GimbalModule *gimbalMgr = getGimbalModule(index);
  if (gimbalMgr) {
    enable = gimbalMgr->getEnable();
    return ErrorCode::SysCommonErr::Success;
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void GimbalManager::resetAsync(PayloadIndexType index,
                               void (*UserCallBack)(
                                   ErrorCode::ErrorCodeType retCode,
                                   UserData userData),
                               UserData userData) {
  GimbalModule* gimbal = getGimbalModule(index);
  if (gimbal) {
    gimbal->resetAsync(UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType GimbalManager::resetSync(
    PayloadIndexType index, int timeout) {
  GimbalModule* gimbal = getGimbalModule(index);
  if (gimbal) {
    return gimbal->resetSync(timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}

void GimbalManager::rotateAsync(PayloadIndexType index,
                                GimbalModule::Rotation rotation,
                                void (*UserCallBack)(
                                    ErrorCode::ErrorCodeType retCode,
                                    UserData userData),
                                UserData userData) {
  GimbalModule* gimbal = getGimbalModule(index);
  if (gimbal) {
    gimbal->rotateAsync(rotation, UserCallBack, userData);
  } else {
    if (UserCallBack)
      UserCallBack(ErrorCode::SysCommonErr::AllocMemoryFailed, userData);
  }
}

ErrorCode::ErrorCodeType GimbalManager::rotateSync(
    PayloadIndexType index, GimbalModule::Rotation rotation, int timeout) {
  GimbalModule* gimbal = getGimbalModule(index);
  if (gimbal) {
    return gimbal->rotateSync(rotation, timeout);
  } else {
    return ErrorCode::SysCommonErr::AllocMemoryFailed;
  }
}
