/** @file dji_payload_base.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Abstract protocol implementation for payload module
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

#ifndef ONBOARDSDK_DJI_PAYLOAD_BASE_HPP
#define ONBOARDSDK_DJI_PAYLOAD_BASE_HPP

#include <stdint.h>
#include <string>

namespace DJI {
namespace OSDK {

/*! @brief The payload Index of camera
 */
enum PayloadIndexType {
  PAYLOAD_INDEX_0 = 0x00,
  PAYLOAD_INDEX_1 = 0x01,
  PAYLOAD_INDEX_2 = 0x02,
  PAYLOAD_INDEX_CNT = 0x03,
  PAYLOAD_INDEX_INVALID = 0x03,
};

/*! @brief PayloadBase
 */
class PayloadBase {
 public:
  PayloadBase(PayloadIndexType index, std::string name, bool enable);

  ~PayloadBase();

 public:
  /*! @brief set the enable status of this payload module
   *
   *  @param en enable status of this payload module.
   */
  void setEnable(bool en);

  /*! @brief get the enable status of this payload module
   *
   *  @return en enable status of this payload module
   */
  bool getEnable();

  PayloadIndexType getIndex();

  std::string getName();

  void setName(std::string name);

 private:
  std::string name;
  PayloadIndexType index;
  bool enable;
};
}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_PAYLOAD_BASE_HPP
