/** @file dji_psdk_module.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of psdk module for payload node
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

#ifndef ONBOARDSDK_DJI_PSDK_MODULE_HPP
#define ONBOARDSDK_DJI_PSDK_MODULE_HPP

#include "dji_payload_base.hpp"
#include "dji_payload_link.hpp"

namespace DJI {
namespace OSDK {
class PSDKModule : public PayloadBase {
 public:
  PSDKModule(PayloadLink *payloadLink, PayloadIndexType payloadIndex,
             std::string name, bool enable);

  ~PSDKModule();

 public:
  const static uint16_t MAX_SIZE_OF_PACKAGE = 255;

 private:
  PayloadLink *payloadLink;

}; /* PSDKModule camera */
}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_PSDK_MODULE_HPP
