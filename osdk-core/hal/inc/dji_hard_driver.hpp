/*! @file dji_hard_driver.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Serial device driver abstraction. Provided as an abstract class. Please
 *  inherit and implement for individual platforms.
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

#ifndef DJI_HARDDRIVER_H
#define DJI_HARDDRIVER_H

#include "dji_log.hpp"
#include "dji_memory.hpp"
#include "dji_type.hpp"
#include <cstdint>
#include <ctime>
namespace DJI
{
namespace OSDK
{
class HardDriver
{
public:
  HardDriver();
  virtual ~HardDriver();
  /*! @note How to use
   *  In order to provide platform crossable DJI onboardSDK library,
   *  we abstract this class as a hardware level.
   *
   *  @note function descriptions:
   *
   *  void init();
   *  @brief After calling this function, HardDriver should be able to
   *  read and send correctly, through a correct UART part.
         *
         *  NOTE: STM32 does not implement support for this function.
   *
   *  uint32_t getTimeStamp();
   *  @brief returns a TimeStamp data in unit msec.
   *  The difference between the return value of the function call two times
   *  is the excat time between them in msec.
   *
   *  size_t send(const uint8_t *buf, size_t len);
   *  @brief return sent data length.
   *
   *  size_t readall(uint8_t *buf, size_t maxlen)Thread safety -  = 0;
   *  @brief return read data length.
         *
         *  void delay_nms(uint16_t time) = 0;
         *  @brief delay in milliseconds
         *
   *  void displayLog(char *buf);
   *  @brief Micro "API_LOG" invoked this function, to pass datalog.
   *  In order to pass data through different stream or channel.
   *  We abstract this virtual function for user.
   *  And different from others, this interface is not a pure virtual function.
   *  The default data-passing channel is stdout (printf).
   *  See also "DJI_HardDriver.cpp".
   *
   *  @attention
   *  when writting and reading data, there might have multi-thread problems.
   *  Abstract class HardDriver did not consider these issue.
   *  Please be careful when you are going to implement send and readall
   *  funtions.
   *
   *  @note
   *  we strongly suggest you to inherit this class in your own file, not just
   *  implement
   *  it in DJI_HardDriver.cpp or inside this class
   *
   * */
public:
  virtual void    init() = 0;
  virtual time_ms getTimeStamp() = 0;
  virtual size_t send(const uint8_t* buf, size_t len) = 0;
  virtual size_t readall(uint8_t* buf, size_t maxlen) = 0;
  virtual bool getDeviceStatus()
  {
    return true;
  }

public:
  //! @todo move to Logging class
  virtual void displayLog(const char* buf = 0);

public:
  static const int bufsize = 1024;

public:
  MMU* getMmu()
  {
    return &mmu;
  }

private:
  MMU mmu;
};
} // namespace OSDK
} // namespace DJI

#endif // DJI_HARDDRIVER_H