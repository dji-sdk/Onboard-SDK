/** @file dji_mop_client.hpp
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

#ifndef DJI_MOP_CLIENT_HPP
#define DJI_MOP_CLIENT_HPP

#include "dji_mop_pipeline.hpp"
#include "dji_mop_pipeline_manager_base.hpp"

namespace DJI {
namespace OSDK {

// Forward Declarations
class Vehicle;

/*! @brief Class providing APIs & data structures for acting as a MOP client
 */
class MopClient : MopPipelineManagerBase {
 public:
  MopClient(SlotType slot);
  ~MopClient();
  /*! @brief Connect the target device by a pipelineid with properties of
   * pipeline type. If success, a pipeline object will be created.
   *  @note This is a blocking api
   *  @param id The pipeline id which to be connected, ref to
   * DJI::OSDK::MOP::PipelineID
   *  @param type The pipeline type. It can be set to be RELIABLE or UBRELIABLE
   *  ref to the enum DJI::OSDK::MOP::PipelineType
   *  @param p The pointer of pipeline. If success, it will be pointed to be the
   *  target pipeline object.
   *  @return ref to the enum DJI::OSDK::MOP::MopErrCode
   */
  MopErrCode connect(PipelineID id, PipelineType type, MopPipeline *&p);

  /*! @brief Connect the target device by a pipelineid with properties of
   * pipeline type. If success, a pipeline object will be created.
   *  @note This is a non-blocking api.
   *  @todo This api is not implemented yet, it will be implemented in the
   *  future.
   *  @param id The pipeline id which to be connected, ref to
   * DJI::OSDK::MOP::PipelineID
   *  @param type The pipeline type. It can be set to be RELIABLE or UBRELIABLE
   *  ref to the enum DJI::OSDK::MOP::PipelineType
   *  @param cb Callback function defined by user
   *  @arg @b errCode is the DJI::OSDK::MOP::MopErrCode error code
   *  @arg @b p The pointer of pipeline. If success, it will be pointed to be the
   *  target pipeline object.
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void connect(PipelineID id, PipelineType type,
               void (*cb)(MopErrCode errCode, MopPipeline *p,
                          void *userData),
               void *userData);

  /*! @brief Disonnect the target device by a pipelineid.
   *  @note This is a blocking api
   *  @param id The pipeline id which to be connected, ref to
   * DJI::OSDK::MOP::PipelineID
   *  @return ref to the enum DJI::OSDK::MOP::MopErrCode
   */
  MopErrCode disconnect(PipelineID id);

  /*! @brief Disonnect the target device by a pipelineid.
   *  @note This is a non-blocking api
   *  @todo This api is not implemented yet, it will be implemented in the
   *  future.
   *  @param id The pipeline id which to be connected, ref to the enum
   *  @param cb Callback function defined by user
   *  @arg @b errCode is the DJI::OSDK::MOP::MopErrCode error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void disconnect(PipelineID id,
                  void (*cb)(MopErrCode errCode, void *userData),
                  void *userData);

 private:
  Vehicle *vehicle;
  SlotType slot;
};

}
}


#endif //DJI_MOP_CLIENT_HPP