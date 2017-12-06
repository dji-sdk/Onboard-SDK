/*! @file posix_thread.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK Linux/*NIX platforms
 *
 *  @Copyright (c) 2016-2017 DJI
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

#ifndef LINUXTHREAD_H
#define LINUXTHREAD_H

#include "dji_log.hpp"
#include "dji_thread_manager.hpp"
#include <pthread.h>
#include <string>

namespace DJI
{
namespace OSDK
{

//! Forward declaration
class Vehicle;

/*! @brief POSIX-compatible threading implementation for *NIX systems
 *
 * @details Threading is handled by the Vehicle, you do not need to
 * manage threads for nominal usage.
 *
 */
class PosixThread : public Thread
{
public:
  PosixThread();
  PosixThread(Vehicle* vehicle, int type);
  ~PosixThread()
  {
  }

  bool createThread();
  int  stopThread();

private:
  pthread_t      threadID;
  pthread_attr_t attr;

  static void* send_call(void* param);
  static void* uart_serial_read_call(void* param);
  static void* callback_call(void* param);
  static void* USB_read_call(void* param);
};

} // namespace DJI
} // namespace onboardSDK

#endif // LINUXTHREAD_H
