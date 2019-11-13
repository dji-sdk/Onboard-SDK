/** @file dji_telemetry.hpp
 *  @version 3.7
 *  @date Jul 2018
 *
 *  @brief Enumeration of all telemetry data types, structures and maps.
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

#ifndef ONBOARDSDK_DJI_Telemetry_H
#define ONBOARDSDK_DJI_Telemetry_H

#include "dji_type.hpp"

/*!
 * Top-level namespace
 */
namespace DJI
{
/*!
 * Onboard SDK related commands
 */
namespace OSDK
{
/*! @brief This namespace encapsulates all available telemetry topics through
 * either
 * Broadcast or Subscribe
 */
namespace Telemetry
{
/*!
 * @brief enum TopicName is the interface for user to create packages and access
 * data
 * It is also used as index for TopicDataBase.
 *
 * @note Please see dji_telemetry_doc.hpp (Or the API reference on the developer website) for detailed documentation.
 *
 */
// clang-format off
typedef enum
{
  TOPIC_QUATERNION,
  TOPIC_ACCELERATION_GROUND,
  TOPIC_ACCELERATION_BODY,
  TOPIC_ACCELERATION_RAW,
  TOPIC_VELOCITY,
  TOPIC_ANGULAR_RATE_FUSIONED,
  TOPIC_ANGULAR_RATE_RAW,
  TOPIC_ALTITUDE_FUSIONED,
  TOPIC_ALTITUDE_BAROMETER,
  TOPIC_HEIGHT_HOMEPOINT,
  TOPIC_HEIGHT_FUSION,
  TOPIC_GPS_FUSED,
  TOPIC_GPS_DATE,
  TOPIC_GPS_TIME,
  TOPIC_GPS_POSITION,
  TOPIC_GPS_VELOCITY,
  TOPIC_GPS_DETAILS,
  TOPIC_RTK_POSITION,
  TOPIC_RTK_VELOCITY,
  TOPIC_RTK_YAW,
  TOPIC_RTK_POSITION_INFO,
  TOPIC_RTK_YAW_INFO,
  TOPIC_COMPASS,
  TOPIC_RC,
  TOPIC_GIMBAL_ANGLES,
  TOPIC_GIMBAL_STATUS,
  TOPIC_STATUS_FLIGHT,
  TOPIC_STATUS_DISPLAYMODE,
  TOPIC_STATUS_LANDINGGEAR,
  TOPIC_STATUS_MOTOR_START_ERROR,
  TOPIC_BATTERY_INFO,
  TOPIC_CONTROL_DEVICE,
  TOPIC_HARD_SYNC,
  TOPIC_GPS_SIGNAL_LEVEL,
  TOPIC_GPS_CONTROL_LEVEL,
  TOPIC_RC_FULL_RAW_DATA,
  TOPIC_RC_WITH_FLAG_DATA,
  TOPIC_ESC_DATA,
  TOPIC_RTK_CONNECT_STATUS,
  TOPIC_GIMBAL_CONTROL_MODE,
  TOPIC_FLIGHT_ANOMALY,
  TOPIC_POSITION_VO,
  TOPIC_AVOID_DATA,
  TOPIC_HOME_POINT_SET_STATUS,
  TOPIC_HOME_POINT_INFO,
  TOTAL_TOPIC_NUMBER              // Always put this line in the end
} TopicName;
// clang-format on

/*!
 * @brief enum TOPIC_UID is the UID that is accepted by the FC
 */
// clang-format off
typedef enum
{
  UID_QUATERNION               = 0xa493281f,
  UID_ACCELERATION_GROUND      = 0x8696c85b,
  UID_ACCELERATION_BODY        = 0xbb17d5fe,
  UID_ACCELERATION_RAW         = 0xc3503a6e,
  UID_VELOCITY                 = 0x18fb271d,
  UID_ANGULAR_RATE_FUSIONED    = 0x3599c4be,
  UID_ANGULAR_RATE_RAW         = 0x700389ee,
  UID_ALTITUDE_FUSIONED        = 0x11e9c81a,
  UID_ALTITUDE_BAROMETER       = 0x27396a39,
  UID_HEIGHT_HOMEPOINT         = 0x252c164b,
  UID_HEIGHT_FUSION            = 0x87cf419d,
  UID_GPS_FUSED                = 0x4b19a8c7,
  UID_GPS_DATE                 = 0x598f79bc,
  UID_GPS_TIME                 = 0xd48912c9,
  UID_GPS_POSITION             = 0x0c949e94,
  UID_GPS_VELOCITY             = 0x7ac7eb80,
  UID_GPS_DETAILS              = 0x81fed54e,
  UID_RTK_POSITION             = 0x1df9a6b6,
  UID_RTK_VELOCITY             = 0x763d13c3,
  UID_RTK_YAW                  = 0xf45d73fd,
  UID_RTK_POSITION_INFO        = 0xda4a57b5,
  UID_RTK_YAW_INFO             = 0xcb72b9e3,
  UID_HOME_POINT_SET_STATUS    = 0xb5c2211f,
  UID_HOME_POINT_INFO          = 0xbfe4b520,
  UID_COMPASS                  = 0xdf3d72b7,
  UID_RC                       = 0x739f7fe4,
  UID_GIMBAL_ANGLES            = 0x01f71678,
  UID_GIMBAL_STATUS            = 0x8b6cd45c,
  UID_STATUS_FLIGHT            = 0x20cfb02a,
  UID_STATUS_DISPLAYMODE       = 0x1a67d6a1,
  UID_STATUS_LANDINGGEAR       = 0x772d6e22,
  UID_STATUS_MOTOR_START_ERROR = 0x3a41e909,
  UID_BATTERY_INFO             = 0x69779dd9,
  UID_CONTROL_DEVICE           = 0x667ba86a,
  UID_HARD_SYNC                = 0xecbef06d,
  UID_GPS_SIGNAL_LEVEL         = 0xa6a0395f,
  UID_GPS_CONTROL_LEVEL        = 0xe30b17b0,
  UID_RC_FULL_RAW_DATA         = 0x16ec4d0e,
  UID_RC_WITH_FLAG_DATA        = 0xfe04cfcd,
  UID_ESC_DATA                 = 0xaaa0f589,
  UID_RTK_CONNECT_STATUS       = 0x6f349326,
  UID_GIMBAL_CONTROL_MODE      = 0x326a446d,
  UID_FLIGHT_ANOMALY           = 0x0a624b4b,
  UID_POSITION_VO              = 0xd3462697,
  UID_AVOID_DATA               = 0xf6405daa
} TOPIC_UID;

// clang-format on
#pragma pack(1)

/*!
 * @brief struct for TopicInfo data used to subscribe packages from the FC
 */
typedef struct
{
  const TopicName name;
  const uint32_t  uid;
  const size_t    size;    /* The size of actual data for the topic */
  const uint16_t  maxFreq; /* max freq in Hz for the topic provided by FC */
  uint16_t        freq;    /* Frequency at which the topic is subscribed */
  uint8_t         pkgID;   /* Package ID in which the topic is subscribed */
  /* Point to topic's address in the data buffer which stores the latest data */
  uint8_t* latest;
} TopicInfo; // pack(1)

/*!
 * @brief struct for TOPIC_QUATERNION
 */
typedef struct Quaternion
{
  float32_t q0; /*!< w */
  float32_t q1; /*!< x */
  float32_t q2; /*!< y */
  float32_t q3; /*!< z */
} Quaternion;   // pack(1)

/*!
 * @brief struct for multiple Topics
 */
typedef struct Vector3f
{
  float32_t x;
  float32_t y;
  float32_t z;
} Vector3f; // pack(1)

/*!
 * @brief struct for multiple Topics
 *
 * @note for TOPIC_GPS_POSITION, data type: (uint32)deg*10^7
 */
typedef struct Vector3d
{
  int32_t x;
  int32_t y;
  int32_t z;
} Vector3d; // pack(1)

/*!
 * @brief struct for data broadcast, timestamp from local cache
 *
 * @note not available in data subscription
 */
typedef struct TimeStamp
{
  uint32_t time_ms;
  uint32_t time_ns;
} TimeStamp; // pack(1)

/*!
 * @brief struct for data broadcast, software sync timestamp from local cache
 *
 * @note not available in data subscription and different from Hardware sync
 */
typedef struct SyncStamp
{
  uint32_t time_2p5ms; /*!< relative sync time */
  uint16_t tag;
  uint8_t  flag;
} SyncStamp; // pack(1)

/*!
 * @brief struct indicates the signal level of GPS velocity info <br>
 *
 */
typedef struct VelocityInfo
{

  uint8_t health : 1; /*!< 1 - using GPS, 0 - not using GPS */
  uint8_t reserve : 7;
} VelocityInfo; // pack(1)

/*!
 * @brief struct for TOPIC_VELOCITY
 *
 * @note The velocity may be in body or ground frame
 * based on settings in DJI Assistant 2's SDK page.
 */
typedef struct Velocity
{
  Vector3f data;
  /*! scale from 0 - 5 signifying gps signal strength <br>
   *  greater than 3 for strong signal
   */
  VelocityInfo info;
} Velocity; // pack(1)

/*!
 * @brief struct for data broadcast, return GPS data
 *
 * @note not available in data subscription
 */
typedef struct GlobalPosition
{
  float64_t latitude;  /*!< unit: rad */
  float64_t longitude; /*!< unit: rad */
  float32_t altitude;  /*!< Measured by barometer: WGS 84 reference ellipsoid */
  float32_t height;    /*!< Ultrasonic height in meters */
  uint8_t   health;    /*!< scale from 0 - 5 signifying gps signal strength <br>
                        * greater than 3 for strong signal */
} GlobalPosition;      // pack(1)

/*!
 * @brief struct for TOPIC_GPS_FUSED
 *
 * @note fusion data from GPS and IMU, return in gps format
 */
typedef struct GPSFused
{
  float64_t longitude;              /*!< unit: rad */
  float64_t latitude;               /*!< unit: rad */
  float32_t altitude;               /*!< WGS 84 reference ellipsoid */
  uint16_t  visibleSatelliteNumber; /*!< number of visible satellite */
} GPSFused;                         // pack(1)

/*!
 * @brief struct for data broadcast and subscription, return obstacle info around the vehicle
 *
 * @note available in M210 (front, up, down)
 */
typedef struct RelativePosition
{
  float32_t down;            /*!< distance from obstacle (m) */
  float32_t front;           /*!< distance from obstacle (m) */
  float32_t right;           /*!< distance from obstacle (m) */
  float32_t back;            /*!< distance from obstacle (m) */
  float32_t left;            /*!< distance from obstacle (m) */
  float32_t up;              /*!< distance from obstacle (m) */
  uint8_t   downHealth : 1;  /*!< Down sensor flag: 0 - not working, 1 - working */
  uint8_t   frontHealth : 1; /*!< Front sensor flag: 0 - not working, 1 - working */
  uint8_t   rightHealth : 1; /*!< Right sensor flag: 0 - not working, 1 - working */
  uint8_t   backHealth : 1;  /*!< Back sensor flag: 0 - not working, 1 - working */
  uint8_t   leftHealth : 1;  /*!< Left sensor flag: 0 - not working, 1 - working */
  uint8_t   upHealth : 1;    /*!< Up sensor health flag: 0 - not working, 1 - working */
  uint8_t   reserved : 2;    /*!< Reserved sensor health flag*/
} RelativePosition; // pack(1)

/*!
 * @brief Timestamp for GPS and RTK
 *
 * @note: Data and time are GMT+8
 */
typedef struct PositionTimeStamp
{
  uint32_t date;     /*!< yyyymmdd E.g.20150205 means February 5th,2015 (GMT+8)*/
  uint32_t time;     /*!< hhmmss E.g. 90209 means 09:02:09 (GMT+8)*/
} PositionTimeStamp; // pack(1)

/*!
 * @brief struct for TOPIC_RTK_POSITION and sub struct for RTK of data broadcast
 */
typedef struct PositionData
{
  float64_t longitude; /*!< deg */
  float64_t latitude;  /*!< deg */
  float32_t HFSL;      /*!< height above mean sea level (m) */
} PositionData;        // pack(1)

typedef struct HomeLocationData
{
  float64_t latitude;  /*!< unit: rad */
  float64_t longitude; /*!< unit: rad */
}HomeLocationData; // pack(1)

typedef struct HomeLocationSetStatus
{
  uint8_t status;     /*!<0:fail, 1:success*/
}HomeLocationSetStatus;// pack(1)
/*!
 * @brief struct for TOPIC_GPS_DETAILS and sub struct for GPSInfo of data
 * broadcast
 *
 * @note only work outside of simulation
 */
typedef struct GPSDetail
{
  float32_t hdop;       /*!< horizontal dilution of precision */
  float32_t pdop;       /*!< position dilution of precision */
  float32_t fix;        /*!< the state of GPS fix */
  float32_t gnssStatus; /*!< vertical position accuracy (mm) */
  float32_t hacc;       /*!< horizontal position accuracy (mm) */
  float32_t sacc;       /*!< the speed accuracy (cm/s) */
  uint32_t  usedGPS;    /*!< the number of GPS satellites used for pos fix */
  uint32_t  usedGLN; /*!< the number of GLONASS satellites used for pos fix */
  uint16_t  NSV;     /*!< the total number of satellites used for pos fix */
  uint16_t  GPScounter; /*!< the accumulated times of sending GPS data  */
} GPSDetail;            // pack(1)

/*!
 * @brief struct for GPSInfo of data broadcast
 *
 * @note only work outside of simulation
 */
typedef struct GPSInfo
{
  PositionTimeStamp time;
  int32_t           longitude;   /*!< 1/1.0e7deg */
  int32_t           latitude;    /*!< 1/1.0e7deg */
  int32_t           HFSL;        /*!< height above mean sea level (mm) */
  Vector3f          velocityNED; /*!< cm/s */
  GPSDetail         detail;
} GPSInfo; // pack(1)

/*!
 * @brief sub struct for RTK of data broadcast
 */
typedef struct PositionFrame
{
  PositionTimeStamp time;
  PositionData      data;
} PositionFrame; // pack(1)

/*!
 * @brief struct for data broadcast, return RTK info
 *
 * @note Available on A3/M600, need to enable it separately on DJI Assistant 2
 */
typedef struct RTK
{
  /*! Timestamp and GPS coordinates */
  PositionFrame pos;
  /*! NED velocity measured by RTK */
  Vector3f      velocityNED;
  /*! the azimuth measured by RTK */
  int16_t yaw;
  /*!
   * 0 - no solution <br>
   * 1 - Position has been fixed by the FIX POSITION command  <br>
   * 2 - Position has been fixed by the FIX HEIGHT/AUTO command <br>
   * 8 - Velocity computed using instantaneous Doppler <br>
   * 16 - Single point position <br>
   * 17 - Pseudorange differential solution <br>
   * 18 - Solution calculated using corrections from an SBAS <br>
   * 19 - Propagated by a Kalman filter without new observations <br>
   * 20 - OmniSTAR VBS position (L1 sub-metre) <br>
   * 32 - Floating L1 ambiguity solution <br>
   * 33 - Floating ionospheric-free ambiguity solution <br>
   * 34 - Floating narrow-lane ambiguity solution <br>
   * 48 - Integer L1 ambiguity solution <br>
   * 49 - Integer wide-lane ambiguity solution <br>
   * 50 - Integer narrow-lane ambiguity solution <br>
   */
  uint8_t posHealthFlag;
  uint8_t yawHealthFlag; /*!< same as posHealthFlag */
} RTK;                   // pack(1)

/*!
 * @brief struct for data broadcast, return magnetometer reading
 *
 * @note returned value is calibrated mag data,
 * 1000 < |mag| < 2000 for normal operation
 */
typedef struct Mag
{
  int16_t x;
  int16_t y;
  int16_t z;
} Mag; // pack(1)

/*!
 * @brief struct for data broadcast and data subscription, return RC reading
 */
typedef struct RC
{
  int16_t roll;     /*!< [-10000,10000] */
  int16_t pitch;    /*!< [-10000,10000] */
  int16_t yaw;      /*!< [-10000,10000] */
  int16_t throttle; /*!< [-10000,10000] */
  int16_t mode;     /*!< [-10000,10000] */
                    /*!< M100 [P: -8000, A: 0, F: 8000] */
  int16_t gear;     /*!< [-10000,10000] */
                    /*!< M100 [Up: -10000, Down: -4545] */
} RC;               // pack(1)

/*!
 * @brief struct for TOPIC_GIMBAL_STATUS
 */
typedef struct GimbalStatus
{
  uint32_t mountStatus : 1; /*!< 1 - gimbal mounted, 0 - gimbal not mounted*/
  uint32_t isBusy : 1;
  uint32_t pitchLimited : 1;           /*!< 1 - axis reached limit, 0 - no */
  uint32_t rollLimited : 1;            /*!< 1 - axis reached limit, 0 - no */
  uint32_t yawLimited : 1;             /*!< 1 - axis reached limit, 0 - no */
  uint32_t calibrating : 1;            /*!< 1 - calibrating, 0 - no */
  uint32_t prevCalibrationgResult : 1; /*!< 1 - success, 0 - fail */
  uint32_t installedDirection : 1;     /*!< 1 - reversed for OSMO, 0 - normal */
  uint32_t disabled_mvo : 1;
  uint32_t gear_show_unable : 1;
  uint32_t gyroFalut : 1;       /*!< 1 - fault, 0 - normal */
  uint32_t escPitchStatus : 1;  /*!< 1 - Pitch data is normal, 0 - fault */
  uint32_t escRollStatus : 1;   /*!< 1 - Roll data is normal, 0 - fault */
  uint32_t escYawStatus : 1;    /*!< 1 - Yaw data is normal , 0 - fault */
  uint32_t droneDataRecv : 1;   /*!< 1 - normal , 0 - at fault */
  uint32_t initUnfinished : 1;  /*!< 1 - init complete, 0 - not complete */
  uint32_t FWUpdating : 1;      /*!< 1 - updating, 0 - not updating */
  uint32_t reserved2 : 15;
} GimbalStatus; // pack(1)

/*!
 * @brief struct for data broadcast, return gimbal angle
 */
typedef struct Gimbal
{

  float32_t roll;            /*!< degree */
  float32_t pitch;           /*!< degree */
  float32_t yaw;             /*!< degree */
  uint8_t   pitchLimit : 1;  /*!< 1 - axis reached limit, 0 - no */
  uint8_t   rollLimit  : 1;  /*!< 1 - axis reached limit, 0 - no */
  uint8_t   yawLimit   : 1;  /*!< 1 - axis reached limit, 0 - no */
  uint8_t   reserved   : 5;
} Gimbal; // pack(1)

/*!
 * @brief struct for data broadcast, return flight status
 */
typedef struct Status
{
  uint8_t flight; /*!<  See FlightStatus/M100FlightStatus in dji_status.hpp */
  uint8_t mode;   /*!<  enum MODE */
  uint8_t gear;   /*!<  See LandingGearMode in dji_status.hpp */
  uint8_t error;  /*!<  enum DJI_ERROR_CODE */
} Status;         // pack(1)

/*!
 * @brief struct for TOPIC_BATTERY_INFO and data broadcast, return battery
 * status
 */
typedef struct Battery
{
  uint32_t capacity;
  int32_t  voltage;
  int32_t  current;
  uint8_t  percentage;
} Battery; // pack(1)

/*!
 * @brief struct for TOPIC_CONTROL_DEVICE and data broadcast, return SDK info
 */
typedef struct SDKInfo
{
  uint8_t controlMode;      /*!< See CtlrMode in dji_status.hpp*/
  uint8_t deviceStatus : 3; /*!< 0->rc  1->app  2->serial*/
  uint8_t flightStatus : 1; /*!< 1->opensd  0->close */
  uint8_t vrcStatus : 1;
  uint8_t reserved : 3;
} SDKInfo; // pack(1)

/*!
 * @brief sub struct for TOPIC_HARD_SYNC
 */
typedef struct SyncTimestamp
{
  uint32_t time2p5ms; /*!< clock time in multiples of 2.5ms. Sync timer runs at
                         400Hz, this field increments in integer steps */
  uint32_t time1ns;   /*!< nanosecond time offset from the 2.5ms pulse */
  uint32_t resetTime2p5ms; /*!< clock time in multiple of 2.5ms elapsed since the
                              hardware sync started */
  uint16_t index;   /*!< This is the tag field you filled out when using the
                       setSyncFreq API above; use it to identify the packets that
                       have sync data. This is useful when you call the
                       setSyncFreq API with freqInHz = 0, so you get a single
                       pulse that can be uniquely identified with a tag - allowing
                       you to create your own pulse train with uniquely
                       identifiable pulses. */
  uint8_t flag;     /*!< This is true when the packet corresponds to a hardware
                       pulse and false otherwise. This is useful because you can
                       request the software packet to be sent at a higher frequency
                       that the hardware line.*/
} SyncTimestamp;    // pack(1)

/*!
 * @brief struct for TOPIC_HARD_SYNC
 */
typedef struct HardSyncData
{
  SyncTimestamp ts; /*!< time stamp for the incoming data */
  Quaternion    q;  /*!< quaternion */
  Vector3f      a;  /*!< accelerometer reading unit: g */
  Vector3f      w;  /*!< gyro reading unit: rad/sec */
} HardSyncData;     // pack(1)

/*!
 * @brief struct indicating RTK GPS Connection
 */
typedef struct RTKConnectStatus
{
  uint16_t rtkConnected : 1;
  uint16_t reserve         : 15;
} RTKConnectStatus;

/*!
 * @brief struct for TOPIC_RC_WITH_FLAG_DATA
 */
typedef struct RCWithFlagData
{
  float32_t pitch;       /*!< down = -0.999, middle = 0.000, up   =0.999 */
  float32_t roll;        /*!< left = -0.999, middle = 0.000, right=0.999 */
  float32_t yaw;         /*!< left = -0.999, middle = 0.000, right=0.999 */
  float32_t throttle;    /*!< down = -0.999, middle = 0.000, up   =0.999 */
  struct
  {
    uint8_t logicConnected  :1;  /*!< 0 if sky or ground side is disconnected for 3 seconds   */
    uint8_t skyConnected    :1;  /*!< Sky side is connected, i.e., receiver is connected to FC */
    uint8_t groundConnected :1;  /*!< Ground side is connected, i.e., RC is on and connected to FC */
    uint8_t appConnected    :1;  /*!< Mobile App is connected to RC */
    uint8_t reserved        :4;
  } flag;
} RCWithFlagData;

/*!
 * @brief struct for status of each individual esc
 */
typedef struct ESCStatusIndividual
{
  int16_t  current;              /*!< ESC current, unit: mA */
  int16_t  speed;                /*!< ESC speed, unit: rpm */
  uint16_t voltage;              /*!< Input power from battery to ESC, unit: mV */
  int16_t  temperature;          /*!< ESC temperature, unit: degree C */
  uint16_t stall            : 1; /*!< Motor is stall */
  uint16_t empty            : 1; /*!< Motor has no load */
  uint16_t unbalanced       : 1; /*!< Motor speed is unbalanced */
  uint16_t escDisconnected  : 1; /*!< ESC is disconnected */
  uint16_t temperatureHigh  : 1; /*!< Temperature is high */
  uint16_t reserved         : 11;
} ESCStatusIndividual;

#define MAX_ESC_NUM 8
/*!
 * @brief struct for TOPIC_ESC_DATA
 */
typedef struct EscData
{
  ESCStatusIndividual esc[MAX_ESC_NUM];
} EscData;

/*!
 * @brief struct for the light bridge 2 part of TOPIC_RC_FULL_RAW_DATA
 */
typedef struct LB2RcFullRawData
{
  int16_t roll;             /*!< left = 364, middle = 1024, right=1684 */
  int16_t pitch;            /*!< down = 364, middle = 1024, up=1684 */
  int16_t yaw;              /*!< left = 364, middle = 1024, right=1684 */
  int16_t throttle;         /*!< down = 364, middle = 1024, up=1684 */
  int16_t mode;             /*!< right(P) = 364, middle(A) = 1024, left(F)=1684 */
  int16_t gear;             /*!< up = 1684,  down = 1354 */
  int16_t camera;           /*!< press_down = 1684, release = 364 */
  int16_t video;            /*!< press_down = 1684, release = 364 */
  int16_t videoPause;       /*!< press_down = 1684, release = 364 */
  int16_t goHome;           /*!< Always 364 */
  int16_t leftWheel;        /*!< left = 364, middle = 1024, right=16844 */
  int16_t rightWheelButton; /*!< press_down = 1684, release = 364 */
  int16_t rcC1;             /*!< press_down = 1684, release = 364 */
  int16_t rcC2;             /*!< press_down = 1684, release = 364 */
  int16_t rcD1;             /*!< rcD1 - 8 is used by sbus */
  int16_t rcD2;
  int16_t rcD3;
  int16_t rcD4;
  int16_t rcD5;
  int16_t rcD6;
  int16_t rcD7;
  int16_t rcD8;
} LB2RcFullRawData;

#define SDK_LB2_CHANNEL_NUM (sizeof(LB2RcFullRawData)/sizeof(int16_t))
#define SDK_SBUS_CHANNEL_NUM (16)
/*!
 * @brief struct for the sbus part of TOPIC_RC_FULL_RAW_DATA
 */
typedef struct SBUSFullRawData
{
  int16_t data[SDK_SBUS_CHANNEL_NUM];
  int16_t reserved[SDK_LB2_CHANNEL_NUM - SDK_SBUS_CHANNEL_NUM];
} SBUSFullRawData;

/*!
 * @brief union for TOPIC_RC_FULL_RAW_DATA
 */
typedef union
{
  LB2RcFullRawData lb2;
  SBUSFullRawData  sbus;
} RCFullRawData;

typedef uint8_t GimbalControlMode;

/*!
 * @brief struct for TOPIC_FLIGHT_ANOMALY
 */
typedef struct FlightAnomaly
{
  uint32_t impactInAir               : 1;  /*!< Impact happens in Air */
  uint32_t randomFly                 : 1;  /*!< Randomly fly in GPS mode without stick input*/
  uint32_t heightCtrlFail            : 1;  /*!< Height control failed */
  uint32_t rollPitchCtrlFail         : 1;  /*!< Tilt control failed */
  uint32_t yawCtrlFail               : 1;  /*!< Yaw control failed */
  uint32_t aircraftIsFalling         : 1;  /*!< Aircraft is falling */
  uint32_t strongWindLevel1          : 1;  /*!< There is wind that FC views as strong level 1*/
  uint32_t strongWindLevel2          : 1;  /*!< There is wind that FC views as strong level 2*/
  uint32_t compassInstallationError  : 1;  /*!< Compass installation error */
  uint32_t imuInstallationError      : 1;  /*!< IMU installation error */
  uint32_t escTemperatureHigh        : 1;  /*!< ESC temperature is high */
  uint32_t atLeastOneEscDisconnected : 1;  /*!< At least one ESC is disconnected */
  uint32_t gpsYawError               : 1;  /*!< GPS yaw error */
  uint32_t reserved                  : 19;
} FlightAnomaly;

/*!
 * @brief struct for TOPIC_POSITION_VO
 */
typedef struct LocalPositionVO
{
    float32_t x;              /*!< North (best effort), unit: m */
    float32_t y;              /*!< East (best effort),  unit: m */
    float32_t z;              /*!< Down,  unit: m */
    uint8_t   xHealth   : 1;
    uint8_t   yHealth   : 1;
    uint8_t   zHealth   : 1;
    uint8_t   reserved  : 5;
} LocalPositionVO;
//---------------------------------Legacy commands below-----------------------------------//

/*!
 * @brief Matrice 100 Timestamp data, available in Broadcast telemetry (only for
 * M100)
 */
typedef struct LegacyTimeStamp
{
  uint32_t time;
  uint32_t nanoTime;
  uint8_t  syncFlag;
} LegacyTimeStamp; // pack(1)

/*!
 * @brief Matrice 100 Velocity struct, returned in Broadcast telemetry (only for
 * M100)
 * @note The velocity may be in body or ground frame
 * based on settings in DJI Assistant 2's SDK page.
 */
typedef struct LegacyVelocity
{
  float32_t x;
  float32_t y;
  float32_t z;
  /*! scale from 0 - 5 signifying gps signal strength <br>
   *  greater than 3 for strong signal
   */
  uint8_t health   : 1;
  uint8_t sensorID : 4;
  uint8_t reserve  : 3;
} LegacyVelocity; // pack(1)

typedef uint16_t EnableFlag; // pack(1)

/*!
 * @brief Return type for flight status data broadcast (only for M100). Returns
 * VehicleStatus::M100FlightStatus.
 */
typedef uint8_t  LegacyStatus;  // pack(1)
/*!
 * @brief Return type for battery data broadcast (only for M100). Returns percentage.
 */
typedef uint8_t  LegacyBattery; // pack(1)

/*!
 * @brief struct for GPSInfo of data broadcast
 *
 * @note only work outside of simulation
 */
typedef struct LegacyGPSInfo
{
  PositionTimeStamp time;
  int32_t           longitude;   /*!< 1/1.0e7deg */
  int32_t           latitude;    /*!< 1/1.0e7deg */
  int32_t           HFSL;        /*!< height above mean sea level (mm) */
  Vector3f          velocityNED; /*!< cm/s */
} LegacyGPSInfo; // pack(1)

#pragma pack()

extern TopicInfo TopicDataBase[];

/*! @brief template struct maps a topic name to the corresponding data
 * type
 *
 */
template <TopicName T>
struct TypeMap
{
  typedef void type;
};

// clang-format off
template <> struct TypeMap<TOPIC_QUATERNION               > { typedef Quaternion      type;};
template <> struct TypeMap<TOPIC_ACCELERATION_GROUND      > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ACCELERATION_BODY        > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ACCELERATION_RAW         > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_VELOCITY                 > { typedef Velocity        type;};
template <> struct TypeMap<TOPIC_ANGULAR_RATE_FUSIONED    > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ANGULAR_RATE_RAW         > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ALTITUDE_FUSIONED        > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_ALTITUDE_BAROMETER       > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_HEIGHT_HOMEPOINT         > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_HEIGHT_FUSION            > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_GPS_FUSED                > { typedef GPSFused        type;};
template <> struct TypeMap<TOPIC_GPS_DATE                 > { typedef uint32_t        type;};
template <> struct TypeMap<TOPIC_GPS_TIME                 > { typedef uint32_t        type;};
template <> struct TypeMap<TOPIC_GPS_POSITION             > { typedef Vector3d        type;};
template <> struct TypeMap<TOPIC_GPS_VELOCITY             > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_GPS_DETAILS              > { typedef GPSDetail       type;};
template <> struct TypeMap<TOPIC_RTK_POSITION             > { typedef PositionData    type;};
template <> struct TypeMap<TOPIC_RTK_VELOCITY             > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_RTK_YAW                  > { typedef int16_t         type;};
template <> struct TypeMap<TOPIC_RTK_POSITION_INFO        > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_RTK_YAW_INFO             > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_COMPASS                  > { typedef Mag             type;};
template <> struct TypeMap<TOPIC_RC                       > { typedef RC              type;};
template <> struct TypeMap<TOPIC_GIMBAL_ANGLES            > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_GIMBAL_STATUS            > { typedef GimbalStatus    type;};
template <> struct TypeMap<TOPIC_STATUS_FLIGHT            > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_STATUS_DISPLAYMODE       > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_STATUS_LANDINGGEAR       > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_STATUS_MOTOR_START_ERROR > { typedef uint16_t        type;};
template <> struct TypeMap<TOPIC_BATTERY_INFO             > { typedef Battery         type;};
template <> struct TypeMap<TOPIC_CONTROL_DEVICE           > { typedef SDKInfo         type;};
template <> struct TypeMap<TOPIC_HARD_SYNC                > { typedef HardSyncData    type;};
template <> struct TypeMap<TOPIC_GPS_SIGNAL_LEVEL         > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_GPS_CONTROL_LEVEL        > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_RC_FULL_RAW_DATA         > { typedef RCFullRawData   type;};
template <> struct TypeMap<TOPIC_RC_WITH_FLAG_DATA        > { typedef RCWithFlagData  type;};
template <> struct TypeMap<TOPIC_ESC_DATA                 > { typedef EscData         type;};
template <> struct TypeMap<TOPIC_RTK_CONNECT_STATUS       > { typedef RTKConnectStatus type;};
template <> struct TypeMap<TOPIC_GIMBAL_CONTROL_MODE      > { typedef GimbalControlMode type;};
template <> struct TypeMap<TOPIC_FLIGHT_ANOMALY           > { typedef FlightAnomaly   type;};
template <> struct TypeMap<TOPIC_POSITION_VO              > { typedef LocalPositionVO type;};
template <> struct TypeMap<TOPIC_AVOID_DATA               > { typedef RelativePosition type;};
template <> struct TypeMap<TOPIC_HOME_POINT_SET_STATUS    > { typedef HomeLocationSetStatus type;};
template <> struct TypeMap<TOPIC_HOME_POINT_INFO          > { typedef HomeLocationData    type;};
// clang-format on
}
}
}
#endif // ONBOARDSDK_DJI_Telemetry_H
