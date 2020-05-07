/*! @file dji_setup_helpers.cpp
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

#include "dji_setup_helpers.hpp"
#include "dji_linker.hpp"
#include "dji_vehicle.hpp"

using namespace DJI::OSDK;

Setup::Setup(bool enableAdvancedSensing) {
  this->linker = NULL;
  this->vehicle = NULL;
  this->useAdvancedSensing = enableAdvancedSensing;
}

Setup::~Setup() {
}

void Setup::setupEnvironment() {
}

bool Setup::initLinker() {
  /*! init linker */
  linker = new(std::nothrow) Linker();
  if (linker == 0) {
    DERROR("Failed to allocate memory for Linker!");
    return false;
  } else {
    if (!linker->init()) {
      DERROR("Failed to initialize Linker!");
      return false;
    }
  }

  return true;
}

bool
Setup::addFCUartChannel(const char *device, uint32_t baudrate) {
  if (linker) {
    return linker->addUartChannel(device, baudrate, FC_UART_CHANNEL_ID);
  } else {
    return false;
  }
}

bool
Setup::addUSBACMChannel(const char *device, uint32_t baudrate) {
  if (linker) {
    return linker->addUartChannel(device, baudrate, USB_ACM_CHANNEL_ID);
  } else {
    return false;
  }
}

bool Setup::initVehicle() {
  return true;
}

