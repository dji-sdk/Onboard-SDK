/** @file dji_mop_client.cpp
 *  @version 4.0
 *  @date Jan 2020
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

#include "dji_mop_client.hpp"
#include "mop.h"

using namespace std;

MopClient::MopClient(SlotType slot) : MopPipelineManagerBase() {
  this->slot = slot;
}

MopClient::~MopClient() {
}

MopErrCode MopClient::connect(PipelineID id, PipelineType type,
                              MopPipeline *&p) {
  int32_t ret;

  /*! 1.Find whether the pipeline object created or not */
  if (pipelineMap.find(id) == pipelineMap.end()) {
    MopErrCode createRet;
    if ((createRet = create(id, p)) != MOP_PASSED) {
      DERROR("MOP Pipeline create failed");
      return createRet;
    } else {
      pipelineMap[id] = p;
    }
  }

  /*! 2.Do creating */
  if(type == UNRELIABLE) {
    ret = mop_create_channel(&p->channelHandle, MOP_TRANS_UNRELIABLE);
  } else {
    ret = mop_create_channel(&p->channelHandle, MOP_TRANS_RELIABLE);
  }

  if (MOP_SUCCESS != ret) {
    DERROR("MOP create channel failed");
    return getMopErrCode(ret);
  }

  /*! 3.Do connecting */
  do {
    DSTATUS("Trying to connect pipeline slot : %d, channel_id : %d", slot, id);
    ret = mop_connect_channel(p->channelHandle, MOP_DEVICE_PSDK, slot, id);
    DSTATUS("Result of connecting pipeline (slot:%d, channel_id:%d) : %d", slot, id, ret);
    sleep(1);
  } while (ret != MOP_SUCCESS);

  if (ret != MOP_SUCCESS) {
    DERROR("Connect Mop Channel failed, destroy mop channel");
    mop_destroy_channel(p->channelHandle);
  }

  return getMopErrCode(ret);
}

void MopClient::connect(PipelineID id, PipelineType type,
                        void (*cb)(MopErrCode errCode, MopPipeline *p,
                                   void *userData),
                        void *userData) {
  if (cb) cb(MOP_NOTIMPLEMENT, NULL, NULL);
}

MopErrCode MopClient::disconnect(PipelineID id) {
  int32_t ret;
  if (pipelineMap.find(id) == pipelineMap.end()) {
    return MOP_PARM;
  }
  mop_channel_handle_t handler = pipelineMap[id]->channelHandle;

  DSTATUS("Trying to disconnect pipeline slot : %d, channel_id : %d", slot, id);
  ret = mop_close_channel(handler);
  DSTATUS("Result of disconnecting pipeline (slot:%d, channel_id:%d) : %d", slot, id, ret);

  return getMopErrCode(ret);
}

void MopClient::disconnect(PipelineID id,
                           void (*cb)(MopErrCode errCode, void *userData),
                           void *userData) {
  if (cb) cb(MOP_NOTIMPLEMENT, NULL);
}
