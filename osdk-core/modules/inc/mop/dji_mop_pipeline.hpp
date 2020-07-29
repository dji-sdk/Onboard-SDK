
/** @file dji_mop_pipeline.hpp
 *  @version 4.0.0
 *  @date January 2020
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
#ifndef DJI_MOP_PIPELINE_HPP
#define DJI_MOP_PIPELINE_HPP

#include <stdint.h>
#include "dji_mop_define.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::MOP;

namespace DJI {
namespace OSDK {

/*! @brief Class providing APIs & data structures MOP pipeline operations
 */
class MopPipeline {
 public:
  MopPipeline(PipelineID id, PipelineType type);

  ~MopPipeline();

  typedef struct DataPackType {
    uint8_t* data;
    uint32_t length;
  } DataPackType;

 public:
  /*! @brief Send data packet to the pipeline
   *
   *  @platforms M300
   *  @note This is a blocking api
   *  @param dataPacket The data packet id which to be sent, ref to
   * DJI::OSDK::MopPipeline::DataPackType
   *  @param len Target len of data packet to be sent. The result of sent-byte
   *  counts will be returned by this parameter
   *  @return ref to the enum DJI::OSDK::MOP::MopErrCode
   */
  MopErrCode sendData(DataPackType dataPacket, uint32_t *len);

  /*! @brief Receive data packet to the pipeline
   *
   *  @platforms M300
   *  @note This is a blocking api
   *  @param dataPacket The data packet id which to be sent, ref to
   * DJI::OSDK::MopPipeline::DataPackType
   *  @param len Target len of data packet to be reveived. The result of
   *  reveived-byte counts will be returned by this parameter
   *  @return ref to the enum DJI::OSDK::MOP::MopErrCode
   */
  MopErrCode recvData(DataPackType dataPacket, uint32_t *len);

  void *channelHandle;

  /*! @brief Get the pipeline id of the pipeline
   *
   *  @platforms M300
   *  @return ref to the enum DJI::OSDK::MOP::PipelineID
   */
  PipelineID getId();

  /*! @brief Get the pipeline type of the pipeline
   *
   *  @platforms M300
   *  @return ref to the enum DJI::OSDK::MOP::PipelineType
   */
  PipelineType getType();
 private:
  PipelineID id;
  PipelineType type;

};
}  // namespace OSDK
}  // namespace DJI

#endif  // DJI_MOP_PIPELINE_HPP
