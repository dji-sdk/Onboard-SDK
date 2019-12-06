/*! @file CameraManagerSample.hpp
 *  @version 3.9
 *  @date July 29 2019
 *
 *  @brief Tesf for the CameraManager. All tests are basic on callback method.
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

#ifndef ONBOARDSDK_NEWCAMERASAMPLE_HPP
#define ONBOARDSDK_NEWCAMERASAMPLE_HPP

#include <dji_vehicle.hpp>
#include "dji_camera_manager.hpp"

enum CameraManagerTestCase {
  X5S_AT_PAYLOAD_0 = 0,
  Z30_AT_PAYLOAD_1 = 1,
  UNKNOWN_TEST_CASE,
};

int cameraManagerTest(Vehicle* vehicle, CameraManagerTestCase testCase);

#endif  // ONBOARDSDK_NEWCAMERASAMPLE_HPP
