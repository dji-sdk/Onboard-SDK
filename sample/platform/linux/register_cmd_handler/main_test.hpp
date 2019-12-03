#ifndef DJI_MAIN_ASYNC_HPP
#define DJI_MAIN_ASYNC_HPP

#include <dji_linux_helpers.hpp>

typedef enum FLAG {
  FLAG_TIME           = 0X0001,
  FLAG_QUATERNION     = 0X0002,
  FLAG_ACCELERATION   = 0X0004,
  FLAG_VELOCITY       = 0X0008,
  FLAG_ANGULAR_RATE   = 0X0010,
  FLAG_POSITION       = 0X0020,
  FLAG_GPSINFO        = 0X0040,
  FLAG_RTKINFO        = 0X0080,
  FLAG_MAG            = 0X0100,
  FLAG_RC             = 0X0200,
  FLAG_GIMBAL         = 0X0400,
  FLAG_STATUS         = 0X0800,
  FLAG_BATTERY        = 0X1000,
  FLAG_DEVICE         = 0X2000,
} FLAG;

typedef float  float32_t;
typedef double float64_t;

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

E_OsdkStat unpackData(struct _CommandHandle *cmdHandle, 
                      const T_CmdInfo *cmdInfo, 
                      const uint8_t *cmdData,
                      void *userData);


#endif // DJI_MAIN_ASYNC_HPP
