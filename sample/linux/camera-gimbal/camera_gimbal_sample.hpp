/*! @file camera_gimbal_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Camera and Gimbal Control API usage in a Linux environment.
 *  Shows example usage of camera commands and gimbal position/speed control
 * APIs
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

#ifndef DJIOSDK_CAMERAGIMBALSAMPLE_HPP
#define DJIOSDK_CAMERAGIMBALSAMPLE_HPP

// DJI OSDK includes
#include <dji_linux_helpers.hpp>
#include <dji_vehicle.hpp>

// Be precise here
struct RotationAngle
{
  DJI::OSDK::float32_t roll;
  DJI::OSDK::float32_t pitch;
  DJI::OSDK::float32_t yaw;
};

struct GimbalContainer
{
  int           roll             = 0;
  int           pitch            = 0;
  int           yaw              = 0;
  int           duration         = 0;
  int           isAbsolute       = 0;
  bool          yaw_cmd_ignore   = false;
  bool          pitch_cmd_ignore = false;
  bool          roll_cmd_ignore  = false;
  RotationAngle initialAngle;
  RotationAngle currentAngle;
  GimbalContainer(int roll = 0, int pitch = 0, int yaw = 0, int duration = 0,
                  int isAbsolute = 0, RotationAngle initialAngle = {},
                  RotationAngle currentAngle = {})
    : roll(roll)
    , pitch(pitch)
    , yaw(yaw)
    , duration(duration)
    , isAbsolute(isAbsolute)
    , initialAngle(initialAngle)
    , currentAngle(currentAngle)
  {
  }
};

// Helper functions
void doSetGimbalAngle(DJI::OSDK::Vehicle* vehicle, GimbalContainer* gimbal);
bool gimbalCameraControl(DJI::OSDK::Vehicle* vehicle);
void displayResult(RotationAngle* currentAngle);

#endif // DJIOSDK_CAMERAGIMBALSAMPLE_HPP
