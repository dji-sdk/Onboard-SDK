/** @file dji_mop_server.hpp
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

#ifndef DJI_MOP_SERVER_HPP
#define DJI_MOP_SERVER_HPP

#include "dji_mop_pipeline.hpp"
#include "dji_mop_pipeline_manager_base.hpp"

namespace DJI {
namespace OSDK {

// Forward Declarations
class Vehicle;

/*! @brief Class providing APIs & data structures for acting as a MOP server
 */
class MopServer : MopPipelineManagerBase {
 public:
  MopServer();
  ~MopServer();

  /*! @brief Accept the connecting request from target device with properties of
   * a pipelineid and pipeline type. If success, a pipeline object will be
   * created.
   *  @note This is a blocking api
   *  @param id The pipeline id which to be connected, ref to
   * DJI::OSDK::MOP::PipelineID
   *  @param type The pipeline type. It can be set to be RELIABLE or UBRELIABLE
   *  ref to the enum DJI::OSDK::MOP::PipelineType
   *  @param p The pointer of pipeline. If success, it will be pointed to be the
   *  target pipeline object.
   *  @return ref to the enum DJI::OSDK::MOP::MopErrCode
   */
  MopErrCode accept(PipelineID id, PipelineType type, MopPipeline *&p);

  /*! @brief Close the target pipeline by a pipelineid.
   *  @note This is a blocking api
   *  @param id The pipeline id which to be connected, ref to
   * DJI::OSDK::MOP::PipelineID
   *  @return ref to the enum DJI::OSDK::MOP::MopErrCode
   */
  MopErrCode close(PipelineID id);
 private:
  Vehicle *vehicle;
};

}
}


#endif //DJI_MOP_SERVER_HPP