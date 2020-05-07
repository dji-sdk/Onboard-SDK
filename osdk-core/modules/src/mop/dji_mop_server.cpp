/** @file dji_mop_server.cpp
 *  @version 4.0
 *  @date March 2020
 *
 *  @brief Implementation of the mop client
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

#include "dji_mop_server.hpp"
#include "mop.h"

using namespace std;

MopServer::MopServer() : MopPipelineManagerBase() {
}

MopServer::~MopServer() {
}

MopErrCode MopServer::accept(PipelineID id, PipelineType type, MopPipeline *&p) {
  int32_t ret;
  mop_channel_handle_t bind_handle;

  /*! 0.Find whether the pipeline object is existed or not */
  DSTATUS("/*! 0.Find whether the pipeline object is existed or not */");
  if (pipelineMap.find(id) != pipelineMap.end()) {
    return MOP_RESOCCUPIED;
  }

  /*! 1.Create handler for binding */
  DSTATUS("/*! 1.Create handler for binding */");
  ret = mop_create_channel(&bind_handle, (mop_trans_t)type);
  if (MOP_SUCCESS != ret) {
    DERROR("MOP create channel failed");
    return getMopErrCode(ret);
  }

  /*! 2.Do binding */
  DSTATUS("/*! 2.Do binding */");
  ret = mop_bind_channel(bind_handle, id);
  if (ret != MOP_SUCCESS) {
    DERROR("MOP Pipeline bind failed");
    return getMopErrCode(ret);
  }

  /*! 3.Do accepting */
  p = new MopPipeline(id, type);
  if (!p) {
    DERROR("Pipeline create failed");
    return MOP_NOMEM;
  }
  DSTATUS("/*! 3.Do accepting */");
  DSTATUS("Do accepting blocking for channel [%d] ...", id);
  ret = mop_accept_channel(bind_handle, &p->channelHandle);
  if (MOP_SUCCESS != ret) {
    DERROR("MOP accept failed");
    return getMopErrCode(ret);
  }

  /*! 4.Accept finished */
  DSTATUS("/*! 4.Accept finished */");
  pipelineMap[id] = p;
  DSTATUS("MOP channel [%d] accepted success", id);
  return MOP_PASSED;
}

MopErrCode MopServer::close(PipelineID id) {
  int32_t ret;
  if (pipelineMap.find(id) == pipelineMap.end()) {
    return MOP_PARM;
  }
  mop_channel_handle_t handler = pipelineMap[id];

  DSTATUS("Trying to close pipeline channel_id : %d", id);
  ret = mop_close_channel(handler);
  DSTATUS("Result of close pipeline channel_id:%d : %d", id, ret);

  return getMopErrCode(ret);
}