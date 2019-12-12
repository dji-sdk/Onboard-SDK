/** @file dji_gimbal_module.hpp
 *  @version 4.0
 *  @date November 2019
 *
 *  @brief Implementation of gimbal module for payload node
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

#ifndef ONBOARDSDK_DJI_GIMBAL_MODULE_HPP
#define ONBOARDSDK_DJI_GIMBAL_MODULE_HPP

#include <vector>
#include "dji_payload_base.hpp"
#include "dji_payload_link.hpp"

namespace DJI {
namespace OSDK {
/*! @brief gimbal module
 */
class GimbalModule : public PayloadBase {
 public:

  /*! @brief the work mode of gimbal
   */
  enum GimbalWorkMode : uint8_t {
    WORK_MODE_FREE = 0, /*!< Free mode */
    WORK_MODE_FPV = 1,  /*!< FPV mode */
    WORK_MODE_TAW_FOLLOW = 2, /*!< Follow mode */
    WORK_MODE_DONT_CHANGE = 254, /*! Don't change the gimbal work mode */
  };

  /*! @brief the work mode of gimbal
   */
  enum ReturnCenterCMD : uint8_t {
    CENTER = 1,  /*!< yaw return center for drone, pry return center for osmo */
    SELFIE = 2,  /*!< pry return toward user */
    PITCH_AND_YAW = 3, /*!< pitch and yaw return center */
    PITCH_ONLY = 4, /*!< only pitch return center */
    ROLL_ONLY = 5, /*!< only roll return center */
    YAW_ONLY = 6, /*!< only yaw return center */
  };

#pragma pack(1)
  typedef uint8_t retCodeType;

  typedef struct Rotation {
    uint8_t rotationMode;
    float pitch;
    float roll;
    float yaw;
    double time;
  } Rotation;

  typedef struct gimbalAngleSetting
  {
    /*! yaw angle in degree, unit : 0.1 deg
     */
    int16_t yaw_angle;
    /*! roll angle in degree, unit : 0.1 deg
     */
    int16_t roll_angle;
    /*! pitch angle in degree, unit : 0.1 deg
     */
    int16_t pitch_angle;
    /*! allowance in degree, unit : 0.01 deg
     */
    int16_t allowance;
    /*! reserve bit, must be 0
     */
    uint8_t reserved                :1;
    /*! yaw follow the drone or not, 1 mean plus drone current yaw control.
     */
    uint8_t yaw_follow_enabled      :1;
    /*! enable pitch with control or not
     */
    uint8_t is_pitch_control_invalid:1;
    /*! enable roll with control or not
     */
    uint8_t is_roll_control_invalid :1;
    /*! enable yaw with control or not
     */
    uint8_t is_yaw_control_invalid  :1;
    /*! coordinate choice
     *  0 : drone coordinate, reference bit will work
     *  1 : ground coordinate, reference bit will not work
     */
    uint8_t coordinate              :1;
    /*! 0 = execute angle command based on the previously set reference point
     *  1 = execute angle command based on the current point
     */
    uint8_t reference               :1;
    /*! control authority setting
     *  0 = release control authority
     *  1 = obtain control authority
     */
    uint8_t is_control              :1;
    /*! It is used to set the timeout. Unit second. The default is 2 seconds
     * (when it is 0). When the valid range is 1-10 seconds. 253, it will be
     * regarded as never timeout. When it is required to end, iscontrol sends 0
     */
    uint8_t timeout;
    /*! Unit: 0.01s execution time, final control speed
     */
    uint16_t time_for_action;
    uint8_t reserved_13_byte :7;
    /*! Input angle is attitude angle or joint angle. 0 = attitude angle,
     * 1 = joint angle
     */
    uint8_t atti_or_joint_ref:1;
  } gimbalAngleSetting;

  typedef struct gimbalWorkModeAndReturnCenterSetting
  {
    GimbalWorkMode workMode;
    ReturnCenterCMD returnCenterCmd;
  } gimbalWorkModeAndReturnCenterSetting;
#pragma pack()



 public:
  GimbalModule(Linker *linker, PayloadIndexType payloadIndex,
               std::string name, bool enable);

  ~GimbalModule();

  void resetAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                       UserData userData),
                  UserData userData);

  ErrorCode::ErrorCodeType resetSync(int timeout);

  void rotateAsync(Rotation rotation,
                   void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                        UserData userData),
                   UserData userData);

  ErrorCode::ErrorCodeType rotateSync(Rotation rotation, int timeout);

 private:
  Linker *linker;
  static void handlerCB(const T_CmdInfo *cmdInfo, const uint8_t *cmdData,
                        void *userData, E_OsdkStat cb_type);

}; /* GimbalModule */
}  // namespace OSDK
}  // namespace DJI

#endif  // ONBOARDSDK_DJI_GIMBAL_MODULE_HPP
