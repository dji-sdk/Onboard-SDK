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

// Camera zoom parameters
#pragma pack(1)
typedef struct zoom_config_t
{
  uint8_t digital_zoom_mode 	: 2;	/* 0:step 1:position 2:continuous */
  uint8_t digital_reserve 	: 1;	/* reserve */
  uint8_t digital_zoom_enable	: 1;	/* 0:not_ctrl 1:ctrl */
  uint8_t optical_zoom_mode  	: 2;	/* 0:step 1:position 2:continuous */
  uint8_t optical_reserve		: 1;	/* reserve */
  uint8_t optical_zoom_enable	: 1;	/* 0:not_ctrl 1:ctrl */
} zoom_config_t;

typedef union zoom_param_t
{
  struct
  {
    uint16_t zoom_cont_speed        : 8;	/* continuous speed 0~100 */
    uint16_t zoom_cont_direction    : 1;
    uint16_t zoom_cont_reserve      : 7;
  }cont_param;
  struct
  {
    uint16_t zoom_step_level		: 8;	/* level time * 100 = 1 times */
    uint16_t zoom_step_direction    : 1;
    uint16_t zoom_step_reserve      : 7;
  }step_param;
  struct
  {
    uint16_t zoom_pos_level;		        /* 180 = 1.8times */
  }pos_param;
} zoom_param_t;

typedef struct camera_zoom_data_type
{
  uint8_t func_index;
  uint8_t cam_index;
  zoom_config_t zoom_config;
  zoom_param_t optical_zoom_param;
  zoom_param_t digital_zoom_param;
}camera_zoom_data_type;
#pragma pack()

// Helper functions
void doSetGimbalAngle(DJI::OSDK::Vehicle* vehicle, GimbalContainer* gimbal);
bool gimbalCameraControl(DJI::OSDK::Vehicle* vehicle);
void displayResult(RotationAngle* currentAngle);

void cameraZoomControl(Vehicle* vehicle);
void z30_zoom_test(Vehicle* vehicle, camera_zoom_data_type *zoom);
void z30_zoom_cb(Vehicle* vehiclePtr, RecvContainer recvFrame,UserData userData);
#endif // DJIOSDK_CAMERAGIMBALSAMPLE_HPP
