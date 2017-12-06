/** @file dji_broadcast.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Broadcast Telemetry API for DJI onboardSDK library
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

#ifndef DJIBROADCAST_H
#define DJIBROADCAST_H

#include "dji_telemetry.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

/*! @brief Telemetry API through asynchronous "Broadcast"-style messages
 *
 *  @details Broadcast telemetry is sent by the FC as push data - whenever an
 *  OES connects to the API port on FC, broadcast telemetry starts pushing data
 *  to the OES.
 *
 *  Frequencies can be set through DJI Assistant 2 or through these APIs.
 *
 *  @note Broadcast-style telemetry is an old feature, and will not see many
 *  updates.
 */
class DataBroadcast
{
public:
  /*
   * Frequency values
   */
  enum FREQ
  {
    FREQ_0HZ   = 0,
    FREQ_1HZ   = 1,
    FREQ_10HZ  = 2,
    FREQ_50HZ  = 3,
    FREQ_100HZ = 4,
    FREQ_200HZ = 6,
    FREQ_400HZ = 7,
    FREQ_HOLD  = 5,
  };

  /*
   * @note this enable flag helps you clarify what data
   *        is coming in from data broadcast
   *        If the user specify GPS @50Hz and gyro @10Hz
   *        The whole data packet from broadcast will come
   *        in @50 Hz but gyro will only come in every 5
   *        data packets.
   *        Use this enum with the passFlag then you can
   *        check what data is available
   *        EX:
   *        if (passFlag & DataBroadcast::DATA_ENABLE_FLAG::HAS_W){
   *          // you get gyro data
   *        }
   */
  enum DATA_ENABLE_FLAG
  {
    HAS_TIME = 0x0001,
    HAS_Q    = 0x0002,
    HAS_A    = 0x0004,
    HAS_V    = 0x0008,
    HAS_W    = 0x0010,
    HAS_POS  = 0x0020,
    // below flags are for M100
    HAS_MAG     = 0x0040,
    HAS_RC      = 0x0080,
    HAS_GIMBAL  = 0x0100,
    HAS_STATUS  = 0x0200,
    HAS_BATTERY = 0x0400,
    HAS_DEVICE  = 0x0800,
    // below flags are for A3
    A3_HAS_GPS     = 0x0040,
    A3_HAS_RTK     = 0x0080,
    A3_HAS_MAG     = 0x0100,
    A3_HAS_RC      = 0x0200,
    A3_HAS_GIMBAL  = 0x0400,
    A3_HAS_STATUS  = 0x0800,
    A3_HAS_BATTERY = 0x1000,
    A3_HAS_DEVICE  = 0x2000
  };

public:
  DataBroadcast(Vehicle* vehicle = 0);
  ~DataBroadcast();

public:
  // Non-Blocking local-cache API
  // clang-format off

  /*! Get timestamp from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @return Telemetry::TimeStamp data structure with the newest value.
   */
  Telemetry::TimeStamp    getTimeStamp()          ;

  /*! Get software sync timestamp from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details Note that this is unrelated to the hardware sync subscription.
   *  @return Telemetry::SyncStamp data structure with the newest value.
   */
  Telemetry::SyncStamp    getSyncStamp()          ;

  /*! Get quaternion data from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @return Telemetry::Quaternion data structure with the newest value.
   *  @details Data accuracy:
   *  | Angle        | Unit | Accuracy   | Notes                                           |
   *  |--------------|------|------------|-------------------------------------------------|
   *  | pitch, roll  | deg  | <1         | in non-ahrs mode                                |
   *  | yaw          | deg  | <3         | for well-calibrated compass with fine alignment |
   *  | yaw with rtk | deg  | around 1.2 | in RTK heading fixed mode with 1 meter baseline |
   */
  Telemetry::Quaternion    getQuaternion()         ;

  /*! Get acceleration from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details The acceleration may be in body or ground frame, fused or raw,
   *           based on settings on DJI Assistant 2's SDK page.
   *  @return Telemetry::Vector3f data structure with the newest value.
   */
  Telemetry::Vector3f    getAcceleration()       ;

  /*! Get velocity from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details Data accuracy:
   * | Axis     | Unit | Accuracy                                                                                    |
   * |----------|------|---------------------------------------------------------------------------------------------|
   * | vgx, vgy | m/s  | Around 5cm/s for GNSS navigation. Around 3cm/s with VO at 1 meter height                    |
   * | vgz      | m/s  | 10cm/s only with barometer in steady air. 3cm/s with VO at 1 meter height with 8cm baseline |
   *
   *  The velocity may be in body or ground frame
   *           based on settings on DJI Assistant 2's SDK page.
   *  @return Telemetry::Vector3f data structure with the newest value.
   */
  Telemetry::Vector3f    getVelocity()           ;

  /*! Get Angular Rates from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details The angular rates may be in body or ground frame, fused or raw,
   *           based on settings on DJI Assistant 2's SDK page.
   *  @return Telemetry::Vector3f data structure with the newest value.
   */
  Telemetry::Vector3f   getAngularRate()          ;

  /*! Get Velocity Info (health) from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details This data is received along with velocity.
   *  @return Telemetry::VelocityInfo data structure with the newest value.
   */
  Telemetry::VelocityInfo    getVelocityInfo()       ;

  /*! Get Globalc Position (LLA and metadata) from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details Data accuracy:
   *  | Axis | Unit | Position Sensor | Accuracy                                         |
   *  |------|------|-----------------|--------------------------------------------------|
   *  | x, y | m    | GPS             | <3m with open sky without multipath              |
   *  | z    | m    | GPS             | <5m with open sky without multipath              |
   *  | x, y | m    | RTK             | around 2cm with fine alignment and fix condition |
   *  | z    | m    | RTK             | around 3cm with fine alignment and fix condition |
   *
   *  The returned Lat/Lon values are in rad.
   *  @return Telemetry::GlobalPosition data structure with the newest value.
   */
  Telemetry::GlobalPosition    getGlobalPosition()     ;

  /*! Get Obstacle info around the vehicle from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details The returned value is relative to your home location.
   *  @return Telemetry::RelativePosition data structure with the newest value.
   */
  Telemetry::RelativePosition    getRelativePosition()   ;

  /*! Get GPS Info from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details This feature provides detailed GPS info. Available on A3/N3/M600.
   *           You need to enable it separately on DJI Assistant 2's SDK page.
   *  @return Telemetry::GPSInfo data structure with the newest value.
   */
  Telemetry::GPSInfo     getGPSInfo()            ;

  /*! Get RTK data from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details This feature provides detailed RTK info. Available on A3/M600.
   *           You need to enable it separately on DJI Assistant 2's SDK page.
   *  @return Telemetry::RTK data structure with the newest value.
   *
   *
   *  @note RTK Position Info and Yaw Info State is shown in the table below
   *
   *  | State | Meaning          | Description                                            |
   *  |-------|------------------|--------------------------------------------------------|
   *  | 0     | NONE             | No solution                                            |
   *  | 1     | FIXEDPOS         | Position has been fixed by the FIX POSITION command    |
   *  | 2     | FIXEDHEIGHT      | Position has been fixed by the FIX HEIGHT/AUTO command |
   *  | 8     | DOPPLER_VELOCITY | Velocity computed using instantaneous Doppler          |
   *  | 16    | Single           | Single point position                                  |
   *  | 17    | PSRDIFF          | Pseudorange differential solution                      |
   *  | 18    | WAAS             | Solution calculated using corrections from an SBAS     |
   *  | 19    | PROPAGATED       | Propagated by a Kalman filter without new observations |
   *  | 20    | OMNISTAR         | OmniSTAR VBS position (L1 sub-metre)                   |
   *  | 32    | L1_FLOAT         | Floating L1 ambiguity solution                         |
   *  | 33    | IONOFREE_FLOAT   | Floating ionospheric-free ambiguity solution           |
   *  | 34    | NARROW_FLOAT     | Floating narrow-lane ambiguity solution                |
   *  | 48    | L1_INT           | Integer L1 ambiguity solution                          |
   *  | 49    | WIDE_INT         | Integer wide-lane ambiguity solution                   |
   *  | 50    | NARROW_INT       | Integer narrow-lane ambiguity solution                 |
   *
   */
  Telemetry::RTK     getRTKInfo()            ;

  /*! Get Magnetometer data from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details The returned value is calibrated mag data,
   *           1000 < |mag| < 2000 for normal operation
   *  @return Telemetry::Mag data structure with the newest value.
   */
  Telemetry::Mag     getMag()                ;

  /*! Get RC channel data from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @return Telemetry::RC data structure with the newest value.
   */
  Telemetry::RC     getRC()                 ;

  /*! Get Gimbal data from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *  @details Data Accuracy: 0.1 deg in all axes
   *
   *  @return Telemetry::Gimbal data structure with the newest value.
   */
  Telemetry::Gimbal     getGimbal()             ;

  /*! Get Status (flight status, mode, gear and error) from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *
   *  @return Telemetry::Status data structure with the newest value.
   */
  Telemetry::Status     getStatus()             ;

  /*! Get Battery Info from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *
   *  @return Telemetry::Battery data structure with the newest value.
   */
  Telemetry::Battery     getBatteryInfo()        ;

  /*! Get SDK Control Mode/Authority info from local cache
   *  @note This getter function is only available with Broadcast, not with Subscribe telemetry
   *
   *  @return Telemetry::SDKInfo data structure with the newest value.
   */
  Telemetry::SDKInfo     getSDKInfo()            ;
  // clang-format on

public:
  /*! Non-blocking call for Frequency setting
   *
   *  @param dataLenIs16 Array of length 16 that has frequency values for each
   *  topic
   *  @param callback Callback function you want called upon ACK
   *  @param userData Additional data you want the callback function to have
   *  access to
   */
  void setBroadcastFreq(uint8_t* dataLenIs16, VehicleCallBack callback = 0,
                        UserData userData = 0);
  /*! Blocking call for Frequency setting
   *
   *  @param dataLenIs16 Array of length 16 that has frequency values for each
   *  topic
   *  @param wait_timeout Time(in s) you want the function to wait for an ACK
   *  @return ACK::ErrorCode struct containing the ACK and metadata
   */
  ACK::ErrorCode setBroadcastFreq(uint8_t* dataLenIs16, int wait_timeout);

  /*! Non-Blocking call for setting default frequencies
   *
   */
  void setBroadcastFreqDefaults();

  /*! Blocking call for setting default frequencies
   *
   *  @param wait_timeout Time(in s) you want the function to wait for an ACK
   *  @return ACK::ErrorCode struct containing the ACK and metadata
   */
  ACK::ErrorCode setBroadcastFreqDefaults(int timeout);

  /*! Non-Blocking call for setting all frequencies to zero
   *
   */
  void setBroadcastFreqToZero();

  /*! getter function for passFlag
   *
   * @return uint16_t passFlag
   */
  uint16_t getPassFlag();

public:
  Vehicle* getVehicle() const;
  void setVehicle(Vehicle* vehiclePtr);
  /*
   * @note Distinguish between different FW versions
   */
  void setVersionDefaults(uint8_t* frequencyBuffer);
  void setFreqDefaultsM100_31(uint8_t* frequencyBuffer);
  void setFreqDefaults(uint8_t* frequencyBuffer);

public:
  void setUserBroadcastCallback(VehicleCallBack callback, UserData userData);
  VehicleCallBackHandler unpackHandler;

public:
  static void unpackCallback(Vehicle* vehicle, RecvContainer recvFrame,
                             UserData userData);
  static void setFrequencyCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                   UserData userData);

private:
  // clang-format off
  typedef enum FLAG {
    // Common to A3/N3 and M100
    FLAG_TIME           = 0X0001,
    FLAG_QUATERNION     = 0X0002,
    FLAG_ACCELERATION   = 0X0004,
    FLAG_VELOCITY       = 0X0008,
    FLAG_ANGULAR_RATE   = 0X0010,
    FLAG_POSITION       = 0X0020,

    // Following are A3/N3 specific
    FLAG_GPSINFO        = 0X0040,
    FLAG_RTKINFO        = 0X0080,
    FLAG_MAG            = 0X0100,
    FLAG_RC             = 0X0200,
    FLAG_GIMBAL         = 0X0400,
    FLAG_STATUS         = 0X0800,
    FLAG_BATTERY        = 0X1000,
    FLAG_DEVICE         = 0X2000,

    // Following are M100 specific
    FLAG_M100_MAG       = 0x0040,
    FLAG_M100_RC        = 0x0080,
    FLAG_M100_GIMBAL    = 0x0100,
    FLAG_M100_STATUS    = 0x0200,
    FLAG_M100_BATTERY   = 0x0400,
    FLAG_M100_DEVICE    = 0x0800
  } FLAG;
  // clang-format on

private:
  /*!
   * @brief Extract broadcast data for A3/N3/M600
   * @param recvFrame: pointer to the raw data payload
   */
  void unpackData(RecvContainer* recvFrame);

  /*!
   * @brief Extract broadcast data for M100
   * @param recvFrame: pointer to the raw data payload
   */
  void unpackM100Data(RecvContainer* pRecvFrame);

  /*!
   * @brief Extract broadcast data for M600 FW 3.2.41.5
   * @param recvFrame: pointer to the raw data payload
   */
  void unpackOldM600Data(RecvContainer* recvFrame);

  inline void unpackOne(FLAG flag, void* data, uint8_t*& buf, size_t size);

public:
  void setBroadcastLength(uint16_t length);
  uint16_t getBroadcastLength();

private:
  // clang-format off
  Telemetry::TimeStamp           timeStamp   ;
  Telemetry::SyncStamp           syncStamp   ;
  Telemetry::Quaternion          q           ;
  Telemetry::Vector3f            a           ;
  Telemetry::Vector3f            v           ;
  Telemetry::Vector3f            w           ;
  Telemetry::VelocityInfo        vi          ;
  Telemetry::GlobalPosition      gp          ;
  Telemetry::RelativePosition    rp          ;
  Telemetry::GPSInfo             gps         ;
  Telemetry::RTK                 rtk         ;
  Telemetry::Mag                 mag         ;
  Telemetry::RC                  rc          ;
  Telemetry::Gimbal              gimbal      ;
  Telemetry::Status              status      ;
  Telemetry::Battery             battery     ;
  Telemetry::SDKInfo             info        ;
  /*
   * @note Broadcast data for Matrice 100/600 older firmware that is fundamentally
   * different from the A3/N3/M600 newer firmware
   */
  Telemetry::LegacyTimeStamp	    legacyTimeStamp;
  Telemetry::LegacyVelocity       legacyVelocity;
  Telemetry::LegacyStatus         legacyStatus;
  Telemetry::LegacyBattery        legacyBattery;
  Telemetry::LegacyGPSInfo        legacyGPSInfo;
  // clang-format on
private:
  Vehicle* vehicle;
  uint16_t passFlag;
  uint16_t broadcastLength;

  VehicleCallBackHandler userCbHandler;
};

} // OSDK
} // DJI
#endif // DJIBROADCAST_H
