
/** @file dji_mop_pipeline_manager_base.hpp
 *  @version 4.0
 *  @date January 2020
 *
 *  @brief Implementation of mop pipeline manager base
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
#ifndef DJI_MOP_PIPELINE_MANAGER_BASE_HPP
#define DJI_MOP_PIPELINE_MANAGER_BASE_HPP

#include "dji_mop_define.hpp"
#include "dji_mop_pipeline.hpp"
#include "dji_log.hpp"
#include <map>

using namespace DJI::OSDK;
using namespace DJI::OSDK::MOP;

using namespace std;

/*! TODO:ugly code, will be fixed in the future */
extern map<PipelineID, MopPipeline*> pipelineMap;

namespace DJI {
namespace OSDK {
class MopPipelineManagerBase {
 public:
  MopPipelineManagerBase();

  ~MopPipelineManagerBase();

  /*! TODO:MSDK 单单create的这种写法指代不明,在这个接口加上了"Pipeline"后缀 */
  MopErrCode create(PipelineID id, MopPipeline *&p);

  /*! TODO:MSDK 单单create的这种写法指代不明,在这个接口加上了"Pipeline"后缀 */
  MopErrCode destroy(PipelineID id);

};
}  // namespace OSDK
}  // namespace DJI

#endif  // DJI_MOP_PIPELINE_MANAGER_BASE_HPP
