/** @file dji_mop_pipeline.cpp
 *  @version 4.0
 *  @date July 2020
 *
 *  @brief Implementation of mop pipeline
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

#include "dji_mop_pipeline.hpp"
#include "mop.h"

MopPipeline::MopPipeline(PipelineID id, PipelineType type) : id(id),
                                                             type(type) {
}

MopPipeline::~MopPipeline() {
}

MopErrCode MopPipeline::sendData(DataPackType dataPacket, uint32_t *len) {
  if (this->channelHandle) {
    int32_t ret =
        mop_write_channel(this->channelHandle,
                          dataPacket.data, dataPacket.length);
    if (ret < 0) {
      return getMopErrCode(ret);
    } else {
      *len = ret;
      return MOP_PASSED;
    }
  } else {
    return MOP_UNKNOWN_ERR;
  }
}

MopErrCode MopPipeline::recvData(DataPackType dataPacket, uint32_t *len) {
  if (this->channelHandle) {
    int32_t ret =
        mop_read_channel(this->channelHandle, dataPacket.data, dataPacket.length);
    if (ret < 0) {
      return getMopErrCode(ret);
    } else {
      *len = ret;
      return MOP_PASSED;
    }
  } else {
    return MOP_UNKNOWN_ERR;
  }
}

PipelineID MopPipeline::getId() {
  return this->id;
}

PipelineType MopPipeline::getType() {
  return this->type;
}

#if 0
/*! 异步接口目前先不实现 @TODO:add和register的概念,add表示可以支持多个CB同时触发? */
void MopPipeline::addDataListener(
    void (*cb)(uint8_t *data, uint32_t *len, void *userData),
    void *userData) {

}

void MopPipeline::removeDataListener(
    void (*cb)(uint8_t *data, uint32_t *len, void *userData)) {

}

#endif
