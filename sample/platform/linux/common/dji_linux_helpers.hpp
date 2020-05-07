/*! @file dji_linux_helpers.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Helper functions to handle user configuration parsing, version query and
 * activation.
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

#ifndef ONBOARDSDK_HELPERS_H
#define ONBOARDSDK_HELPERS_H

#include <iostream>
#include <fstream>
#include <dji_linux_environment.hpp>
#include <dji_vehicle.hpp>
#include <dji_platform.hpp>
#include <dji_setup_helpers.hpp>

using namespace std;

class LinuxSetup : private Setup {
 public:
  LinuxSetup(int argc, char **argv, bool enableAdvancedSensing = false);
  ~LinuxSetup();

 public:
  void setupEnvironment(int argc, char** argv);
  bool initVehicle();

 public:
  DJI_Environment *getEnvironment() {
    return this->environment;
  }

  Vehicle::ActivateData *getActivateData() {
    return &this->activateData;
  }

  Vehicle *getVehicle() {
    return vehicle;
  }

 private:
  uint32_t functionTimeout;
  Vehicle::ActivateData activateData;
  DJI_Environment* environment;
};

#endif // ONBOARDSDK_HELPERS_H
