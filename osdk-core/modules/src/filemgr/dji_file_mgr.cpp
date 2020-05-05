/** @file dji_file_mgr.cpp
 *  @version 4.0
 *  @date July 2020
 *
 *  @brief Implementation for file manager
 *
 *  @Copyright (c) 2020 DJI
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

#include "dji_file_mgr.hpp"
#include "dji_file_mgr_impl.hpp"
#include "dji_linker.hpp"
#include "dji_linker.hpp"
#include "osdk_device_id.h"
#include "dji_command.hpp"
#include "osdk_command.h"
#include "osdk_protocol.h"
#include "dji_internal_command.hpp"

using namespace DJI;
using namespace DJI::OSDK;

FileMgr::FileMgr(Linker *linker, uint8_t type, uint8_t index)
    : type(type), index(index) {
  impl = new FileMgrImpl(linker, (E_OSDKCommandDeiveType)type, index);
}

FileMgr::~FileMgr(){
  if (impl)delete impl;
}

ErrorCode::ErrorCodeType FileMgr::startReqFileList(FileListReqCBType cb, void* userData) {
  return impl->startReqFileList(cb, userData);
}

ErrorCode::ErrorCodeType FileMgr::startReqFileData(int fileIndex, std::string localPath, FileDataReqCBType cb, void* userData) {
  return impl->startReqFileData(fileIndex, localPath, cb, userData);
}
