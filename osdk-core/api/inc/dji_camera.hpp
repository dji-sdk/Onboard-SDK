/** @file dji_camera.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Camera/Gimbal API for DJI onboardSDK library
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

#ifndef DJI_CAMERA_H
#define DJI_CAMERA_H

#include "dji_command.hpp"
#include "dji_type.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declaration
class Vehicle;

/*! @brief Camera class for controlling camera-related functions
 * available through open protocol
 * @deprecated This class is deprecated and replaced by
 * DJI::OSDK::CameraManager.
 */
class Camera
{
public:
  Camera(Vehicle* vehicle);
  ~Camera();

public:
  // Non-Blocking API
  /*! take a photo, check DJI Go app or SD card for photo */
  void shootPhoto();
  /*! start recording video, check DJI Go app or SD card for video */
  void videoStart();
  /*! stop recording video, check DJI Go app or SD card for video */
  void videoStop();

private:
  /*! @brief Function for commanding: Take Picture, Start Video, Stop Video
   *  @note The camera function does not return an acknowledgment.
   *  @param cmd array representing camera command
   *  Available camera commands:
   *  OpenProtocol::CMDSet::Control::cameraShot
   *  OpenProtocol::CMDSet::Control::cameraVideoStart
   *  OpenProtocol::CMDSet::Control::cameraVideoStop
   */
  void action(const uint8_t cmd[]);

private:
  Vehicle* vehicle;
}; // class camera
} // namespace OSDK
} // namespace DJI

#endif // DJI_CAMERA_H
