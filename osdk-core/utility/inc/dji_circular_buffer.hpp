/** @file dji_circular_buffer.hpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Circular buffer class for the DJI OSDK
 *
 *  @Copyright (c) 2017 DJI
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

#include "dji_open_protocol.hpp"
#include "dji_vehicle_callback.hpp"
#include <cstdlib>

#if STM32
#include <stdlib.h>
#endif

namespace DJI
{
namespace OSDK
{

/*! @brief Circular buffer for callback function storage
 *
 * @details This buffer is not currently generic, so do not use it for any other
 * purpose.
 */
class CircularBuffer
{
public:
  int cbPush(CircularBuffer* CBuffer, VehicleCallBackHandler data,
             RecvContainer data2);
  int cbPop(CircularBuffer* CBuffer, VehicleCallBackHandler* data,
            RecvContainer* data2);
  CircularBuffer();
  ~CircularBuffer();
  int head;
  int tail;

private:
  VehicleCallBackHandler* buffer;
  RecvContainer*          buffer2;
  const int               maxLen;
}; // class CircularBuffer

} // namespace OSDK
} // namespace DJI
