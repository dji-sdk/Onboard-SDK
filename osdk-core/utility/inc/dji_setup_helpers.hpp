/*! @file dji_setup_helpers.hpp
 *  @version 4.0
 *  @date Feb 19 2020
 *
 *  @brief
 *  Base helper to handle user set up environment.
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

#ifndef ONBOARDSDK_DJI_SETUP_HELPERS_H
#define ONBOARDSDK_DJI_SETUP_HELPERS_H

#include <stdint.h>

namespace DJI
{
namespace OSDK {
/*! forward declaration*/
class Linker;
class Vehicle;

class Setup {
 public:
  Setup(bool enableAdvancedSensing = false);
  ~Setup();

 public:
  bool initLinker();

  /*! add Uart channel*/
  bool addFCUartChannel(const char *device, uint32_t baudrate);
  bool addUSBACMChannel(const char *device, uint32_t baudrate);

  virtual bool initVehicle();
  virtual void setupEnvironment();

 public:
  Vehicle *vehicle;
  Linker *linker;
  bool useAdvancedSensing;
};
}
}

#endif // ONBOARDSDK_DJI_SETUP_HELPERS_H
