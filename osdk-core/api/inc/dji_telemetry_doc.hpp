/** @file dji_telemetry_doc.hpp
 *  @version 3.7
 *  @date Jul 2018
 *
 *  @brief Enumeration of all telemetry data types, structures and maps.
 *
 *  @Copyright (c) 2018 DJI
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

#ifndef ONBOARDSDK_DJI_TELEMETRY_DOC_HPP
#define ONBOARDSDK_DJI_TELEMETRY_DOC_HPP

#include "dji_telemetry.hpp"

/*
 * This file is only for having all the subscription TOPIC_XX_XX documentation in a single place.
 */

using namespace DJI::OSDK::Telemetry;

/*!
 * @defgroup telem Telemetry Topics
 * @brief This group documents all data telemetry topics that can be subscribed to.
 * @note This list does not include the data that is available through the older 'broadcast'-style telemetry
 *
 */


// Detailed doxygen comments begin below

//! @ingroup telem
//! @enum TopicName
//! Enum, containing all the telemetry values

/*!
 * @var TOPIC_QUATERNION
 * @brief Provides quaternion representing body frame \b (FRD) to ground frame \b (NED) rotation @ up to 200Hz
 * @details The DJI quaternion follows Hamilton convention (q0 = w, q1 = x, q2 = y, q3 = z)
 * @perf
 * | Angle        | Unit | Accuracy   | Notes                                           |
   |--------------|------|------------|-------------------------------------------------|
   | pitch, roll  | deg  | <1         | in non-ahrs mode                                |
   | yaw          | deg  | <3         | in well-calibrated compass with fine aligned    |
   | yaw with rtk | deg  | around 1.2 | in RTK heading fixed mode with 1 meter baseline |
 * @sensors IMU, Visual Odometry (M210 only)
 * @platforms A3,N3,M600,M210
 * @units rad (when converted to a rotation matrix or Euler angles)
 * @datastruct \ref Quaternion
 *
 */

/*!
 * @var TOPIC_ACCELERATION_GROUND
 * @brief Provides aircraft's acceleration w.r.t a ground-fixed \b NEU frame @ up to 200Hz
 * @warning Please note that this data is not in a conventional right-handed frame of reference.
 * @details This is a fusion output from the flight control system. The output is in a right-handed NED frame, but the
 * sign of the Z-axis acceleration is flipped before publishing to this topic. So if you are looking to get acceleration
 * in an NED frame, simply flip the sign of the z-axis value. Beyond that, you can convert using rotations to
 * any right-handed frame of reference.
 * @platforms A3,N3,M600,M210
 * @units m/s<SUP>2</SUP>
 * @datastruct \ref Vector3f

 */

/*!
 * @var TOPIC_ACCELERATION_BODY
 * @brief Provides aircraft's acceleration w.r.t a body-fixed \b FRU frame @ up to 200Hz
 * @warning Please note that this data is not in a conventional right-handed frame of reference.
 * @details This is a fusion output from the flight control system.
 * @platforms A3,N3,M600,M210
 * @units m/s<SUP>2</SUP>
 * @datastruct \ref Vector3f
 */


/*!
 * @var TOPIC_ACCELERATION_RAW
 * @brief Provides aircraft's acceleration in an IMU-centered, body-fixed \b FRD frame @ up to 400Hz
 * @details This is a filtered output from the IMU on board the flight control system.
 * @sensors IMU
 * @platforms A3,N3,M600,M210
 * @units m/s<SUP>2</SUP>
 * @datastruct \ref Vector3f
 */


/*!
 * @var TOPIC_VELOCITY
 * @brief Provides aircraft's velocity in a ground-fixed \b NEU frame @ up to 200Hz.
 * @warning Please note that this data is not in a conventional right-handed frame of reference.
 * @details This is a fusion output from the flight control system. The output is in a right-handed NED frame, but the
 * sign of the Z-axis velocity is flipped before publishing to this topic. So if you are looking to get velocity
 * in an NED frame, simply flip the sign of the z-axis value. Beyond that, you can convert using rotations to
 * any right-handed frame of reference.
 * @perf
 * | Axis     | Unit | Accuracy                                                                                    |
   |----------|------|---------------------------------------------------------------------------------------------|
   | vgx, vgy | m/s  | Around 5cm/s for GNSS navigation. Around 3cm/s with VO at 1 meter height                    |
   | vgz      | m/s  | 10cm/s only with barometer in steady air. 3cm/s with VO at 1 meter height with 8cm baseline |
 * @sensors IMU, GPS, Baro, RTK (if available), Visual Odometry (M210 only), TOF (M210 Only)
 * @platforms A3,N3,M600,M210
 * @units m/s
 * @datastruct \ref Velocity
 */

/*!
 * @var TOPIC_ANGULAR_RATE_FUSIONED
 * @brief Provides aircraft's angular velocity in a ground-fixed \b NED frame @ up to 200Hz
 * @details This is a fusion output from the flight control system.
 * @platforms A3,N3,M600,M210
 * @units rad/s
 * @datastruct \ref Vector3f
 */

/*!
 * @var TOPIC_ANGULAR_RATE_RAW
 * @brief Provides aircraft's angular velocity in an IMU-centered, body-fixed \b FRD frame @ up to 400Hz
 * @details This is a filtered output from the IMU on board the flight control system.
 * @sensors IMU
 * @platforms A3,N3,M600,M210
 * @units rad/s
 * @datastruct \ref Vector3f
 */

/*!
 * @var TOPIC_ALTITUDE_FUSIONED
 * @brief Provides aircraft's fused altitude from sea level using the ICAO model @ up to 200Hz
 * @details
 * This is a fusion output from the flight control system, and is the \b recommended source of altitude data.
 *
 * The ICAO model gives an MSL altitude of 1013.25mBar at 15&deg; C and a temperature lapse rate of -6.5&deg; C
 * per 1000m. In your case, it may be possible that the take off altitude of the aircraft is recording a higher pressure
 * than 1013.25mBar. Let's take an example - a weather station shows that SFO (San Francisco International Airport) had
 * recently recorded a pressure of 1027.1mBar. SFO is 4m above MSL, yet, if you calculate the Pressure Altitude using
 * the ICAO model, it relates to -114m. You can use an online calculator to similarly calculate the Pressure Altitude
 * in your area.
 *
 * Another factor that may affect your altitude reading is manufacturing differences in the barometer - it is not
 * uncommon to have a variation of &plusmn;30m readings at the same physical location with two different aircraft. For a given
 * aircraft, these readings will be consistent, so you will need to calibrate the offset of your system if your code
 * relies on the accuracy of the absolute value of altitude.
 * @sensors GPS, Barometer, IMU
 * @platforms A3,N3,M600,M210
 * @units m
 * @datastruct \ref float32_t
 */



/*!
 * @var TOPIC_ALTITUDE_BAROMETER
 * @brief Provides aircraft's pressure altitude from sea level using the ICAO model @ up to 200Hz
 * @details
 * This is a filetered output from the barometer without any further fusion.
 *
 * The ICAO model gives an MSL altitude of 1013.25mBar at 15&deg; C and a temperature lapse rate of -6.5&deg; C
 * per 1000m. In your case, it may be possible that the take off altitude of the aircraft is recording a higher pressure
 * than 1013.25mBar. Let's take an example - a weather station shows that SFO (San Francisco International Airport) had
 * recently recorded a pressure of 1027.1mBar. SFO is 4m above MSL, yet, if you calculate the Pressure Altitude using
 * the ICAO model, it relates to -114m. You can use an online calculator to similarly calculate the Pressure Altitude
 * in your area.
 *
 * Another factor that may affect your altitude reading is manufacturing differences in the barometer - it is not
 * uncommon to have a variation of &plusmn;30m readings at the same physical location with two different aircraft. For a given
 * aircraft, these readings will be consistent, so you will need to calibrate the offset of your system if your code
 * relies on the accuracy of the absolute value of altitude.
 * @sensors GPS, Barometer, IMU
 * @platforms A3,N3,M600,M210
 * @units m
 * @datastruct \ref float32_t
 *
 */

/*!
 * @var TOPIC_HEIGHT_HOMEPOINT
 * @brief Provides the altitude from sea level when the aircraft last took off.
 * @details
 * This is a fusion output from the flight control system, and also uses the ICAO model.
 *
 * The ICAO model gives an MSL altitude of 1013.25mBar at 15&deg; C and a temperature lapse rate of -6.5&deg; C
 * per 1000m. In your case, it may be possible that the take off altitude of the aircraft is recording a higher pressure
 * than 1013.25mBar. Let's take an example - a weather station shows that SFO (San Francisco International Airport) had
 * recently recorded a pressure of 1027.1mBar. SFO is 4m above MSL, yet, if you calculate the Pressure Altitude using
 * the ICAO model, it relates to -114m. You can use an online calculator to similarly calculate the Pressure Altitude
 * in your area.
 *
 * Another factor that may affect your altitude reading is manufacturing differences in the barometer - it is not
 * uncommon to have a variation of &plusmn;30m readings at the same physical location with two different aircraft. For a given
 * aircraft, these readings will be consistent, so you will need to calibrate the offset of your system if your code
 * relies on the accuracy of the absolute value of altitude.
 *
 * @note This value is updated each time the drone takes off.
 *
 * @sensors Visual Odometry (M210 only), Barometer, IMU
 * @platforms A3,N3,M600,M210
 * @units m
 * @datastruct \ref float32_t
 *
 */

/*!
 * @var TOPIC_HEIGHT_FUSION
 * @brief Provides the relative height above ground at up to 100Hz.
 * @details
 * This is a fusion output from the flight control system. The height is a direct estimate of the closest large object below the aircraft's ultrasonic sensors.
 * A large object is something that covers the ultrasonic sensor for an extended duration of time.
 *
 * @warning This topic does not come with a 'valid' flag - so if the aircraft is too far from an object for the
 * ultrasonic sensors/VO to provide any meaningful data, the values will latch and there is no way for user code to
 * determine if the data is valid or not. Use with caution.
 * @sensors Visual Odometry, Ultrasonic
 * @platforms M210
 * @units m
 * @datastruct \ref float32_t
 *
 */



/*!
 * @var TOPIC_GPS_FUSED
 * @brief Provides aircraft's GPS/IMU fused X-Y position and barometric altitude (put together in a single topic for convenience) @ up to 50Hz
 * @warning Please note that if GPS signal is weak (low visibleSatelliteNumber, see below), the
 * latitude/longitude values won't be updated but the altitude might still be. There is currently no way to know if
 * the lat/lon update is healthy.
 * @details The most important component of this topic is the \ref GPSFused::visibleSatelliteNumber "visibleSatelliteNumber".
 * Use this to track your GPS satellite coverage and build some heuristics for when you might expect to lose GPS updates.
 * @perf
 *   | Axis | Unit | Position Sensor | Accuracy                                         |
     |------|------|-----------------|--------------------------------------------------|
     | x, y | m    | GPS             | <3m with open sky without multipath              |
     | z    | m    | GPS             | <5m with open sky without multipath              |
 *
 * @sensors GPS/IMU (x,y), Barometer(z)
 * @platforms A3,N3,M600,M210
 * @units rad (Lat,Lon), m (Alt)
 * @datastruct \ref GPSFused
 */



/*!@var TOPIC_GPS_DATE
 * @brief Provides raw date information from GPS @ up to 5Hz
 * @details Format : yyyymmdd
 * @sensors GPS
 * @platforms A3,N3,M600,M210
 * @units -
 * @datastruct \ref uint32_t
 */

/*!@var TOPIC_GPS_TIME
 * @brief Provides raw time information from GPS @ up to 5Hz
 * @details Format : hhmmss
 * @sensors GPS
 * @platforms A3,N3,M600,M210
 * @units -
 * @datastruct \ref uint32_t
 */


/*!
 * @var TOPIC_GPS_POSITION
 * @brief Provides aircraft's raw GPS LLA @ up to 5Hz
 * @perf
 *   | Axis | Position Sensor | Accuracy                                         |
     |------|-----------------|--------------------------------------------------|
     | x, y | GPS             | <3m with open sky without multipath              |
     | z    | GPS             | <5m with open sky without multipath              |
 *
 * @sensors GPS
 * @platforms A3,N3,M600,M210
 * @units rad*10<SUP>-7</SUP> (Lat,Lon), mm (Alt)
 * @datastruct \ref Vector3d
 * @note The data structure for this UID is too generic for the data itself - please note that in the vector3d struct,
 * x = Latitude, y = Longitude, z = Altitude
 */

/*!
 * @var TOPIC_GPS_VELOCITY
 * @brief Provides aircraft's raw GPS velocity @ up to 5Hz
 * @sensors GPS
 * @platforms A3,N3,M600,M210
 * @units cm/s
 * @datastruct \ref Vector3f
 */

/*!
 * @var TOPIC_GPS_DETAILS
 * @brief Provides aircraft's raw GPS status and other details @ up to 5Hz
 * @sensors GPS
 * @platforms A3,N3,M600,M210
 * @datastruct \ref GPSDetail
 */


/*!
 * @var TOPIC_RTK_POSITION
 * @brief Provides aircraft's raw Real-Time Kinematic (RTK) LLA @ up to 5Hz
 * @perf
 *   | Axis | Position Sensor | Accuracy                                         |
     |------|-----------------|--------------------------------------------------|
     | x, y | RTK             | ~2cm with fine alignment and fix condition       |
     | z    | RTK             | ~3cm with fine alignment and fix condition       |
 *
 * @sensors RTK
 * @platforms A3,M600,M210 (in each case, if RTK is installed)
 * @units deg (x, y), m(z)
 * @datastruct \ref PositionData
 */

/*!
 * @var TOPIC_RTK_VELOCITY
 * @brief Provides aircraft's raw Real-Time Kinematic (RTK) velocity @ up to 5Hz
 * @sensors RTK
 * @platforms A3,M600,M210 (in each case, if RTK is installed)
 * @units cm/s
 * @datastruct \ref Vector3f
 */

/*!
 * @var TOPIC_RTK_YAW
 * @brief Provides aircraft's raw Real-Time Kinematic (RTK) yaw @ up to 5Hz
 * @details The RTK yaw will provide the vector from ANT1 to ANT2 as configured in DJI Assistant 2. In the M210, this
 * means that the value of RTK yaw will be 90&deg; offset from the yaw of the aircraft. For the M600 w/D-RTK, it will
 * depend on how you mount your antennae. In preconfigured units, the mounting will be such that the RTK yaw is 90&deg;
 * offset from the aircraft yaw.
 * @sensors RTK
 * @platforms A3,M600,M210 (in each case, if RTK is installed)
 * @units deg
 * @datastruct int16_t
 */

/*!
 * @var TOPIC_RTK_POSITION_INFO
 * @brief Provides a status on aircraft's Real-Time Kinematic (RTK) positioning solution @ up to 5Hz
 * @details
 *
 * | Value | Meaning                                              |
 * |----|---------------------------------------------------------|
 * | 0  | no solution                                             |
 * | 1  | Position has been fixed by the FIX POSITION command     |
 * | 2  | Position has been fixed by the FIX HEIGHT/AUTO command  |
 * | 8  | Velocity computed using instantaneous Doppler           |
 * | 16 | Single point position                                   |
 * | 17 | Pseudorange differential solution                       |
 * | 18 | Solution calculated using corrections from an SBAS      |
 * | 19 | Propagated by a Kalman filter without new observations  |
 * | 20 | OmniSTAR VBS position (L1 sub-metre)                    |
 * | 32 | Floating L1 ambiguity solution                          |
 * | 33 | Floating ionospheric-free ambiguity solution            |
 * | 34 | Floating narrow-lane ambiguity solution                 |
 * | 48 | Integer L1 ambiguity solution                           |
 * | 49 | Integer wide-lane ambiguity solution                    |
 * | 50 | Integer narrow-lane ambiguity solution                  |
 * @sensors RTK
 * @platforms A3,M600,M210 (in each case, if RTK is installed)
 * @datastruct uint8_t
 */

/*!
 * @var TOPIC_RTK_YAW_INFO
 * @brief Provides a status on aircraft's Real-Time Kinematic (RTK) yaw solution @ up to 5Hz
 * @details
 *
 * | Value | Meaning                                              |
 * |----|---------------------------------------------------------|
 * | 0  | no solution                                             |
 * | 1  | Position has been fixed by the FIX POSITION command     |
 * | 2  | Position has been fixed by the FIX HEIGHT/AUTO command  |
 * | 8  | Velocity computed using instantaneous Doppler           |
 * | 16 | Single point position                                   |
 * | 17 | Pseudorange differential solution                       |
 * | 18 | Solution calculated using corrections from an SBAS      |
 * | 19 | Propagated by a Kalman filter without new observations  |
 * | 20 | OmniSTAR VBS position (L1 sub-metre)                    |
 * | 32 | Floating L1 ambiguity solution                          |
 * | 33 | Floating ionospheric-free ambiguity solution            |
 * | 34 | Floating narrow-lane ambiguity solution                 |
 * | 48 | Integer L1 ambiguity solution                           |
 * | 49 | Integer wide-lane ambiguity solution                    |
 * | 50 | Integer narrow-lane ambiguity solution                  |
 *
 * @sensors RTK
 * @platforms A3,M600,M210 (in each case, if RTK is installed)
 * @datastruct uint8_t
 */


/*!
 * @var TOPIC_COMPASS
 * @brief Provides aircraft's magnetometer reading, fused with IMU and GPS @ up to 100Hz
 * @details This reading is the magnetic field recorded by the magnetometer in x,y,z axis, calibrated such that
 * 1000 < |m| < 2000, and fused with IMU and GPS for robustness
 * @sensors Magnetometer, IMU, GPS
 * @platforms A3,N3, M600,M210
 * @units N/A
 * @datastruct \ref Mag
 */

/*!
 * @var TOPIC_RC
 * @brief Provides remote controller stick inputs @ up to 100Hz
 * @details This topic will give you:
 * - Stick inputs (R,P,Y,Thr)
 * - Mode switch (P/A/F)
 * - Landing gear switch (Up/Down)
 *
 * @platforms A3,N3, M600,M210
 * @datastruct \ref RC
 * @also \ref TOPIC_RC_WITH_FLAG_DATA
 */


/*!
 * @var TOPIC_GIMBAL_ANGLES
 * @brief Provides gimbal pitch, roll, yaw @ up to 50Hz
 * @details
 * The reference frame for gimbal angles is a NED frame attached to the gimbal.
 * This topic uses a data structure, Vector3f, that is too generic for the topic. The order of angles is :
 * |Data Structure Element| Meaning|
 * |----------------------|--------|
 * |Vector3f.x            |pitch   |
 * |Vector3f.y            |roll    |
 * |Vector3f.z            |yaw     |
 *
 * @perf
 * 0.1 deg accuracy in all axes
 *
 * @sensors Gimbal Encoder, IMU, Magnetometer
 * @platforms A3,N3, M600,M210
 * @units deg
 * @datastruct \ref Vector3f
 * @also \ref TOPIC_GIMBAL_STATUS, \ref TOPIC_GIMBAL_CONTROL_MODE
 */

/*!
 * @var TOPIC_GIMBAL_STATUS
 * @brief Provides gimbal status and error codes @ up to 50Hz
 * @details Please see the \ref GimbalStatus struct for the details on what data you can receive.
 *
 * @platforms A3,N3, M600,M210
 * @datastruct \ref GimbalStatus
 * @also \ref TOPIC_GIMBAL_ANGLES, \ref TOPIC_GIMBAL_CONTROL_MODE
 */


/*!
 * @var TOPIC_STATUS_FLIGHT
 * @brief Provides the aircraft's internal flight state @ up to 50Hz.
 * @details The state machine for this topic has the following transition diagram:
 *
 * \imageSize{state_machine.png,height:232px;width:800px;}
 * \image html state_machine.png
 *
 *
 * @platforms A3,N3, M600,M210
 * @datastruct uint8_t (For detailed enumerations see \ref VehicleStatus::FlightStatus "FlightStatus")
 * @also \ref TOPIC_STATUS_DISPLAYMODE
 */

/*!
 * @var TOPIC_STATUS_DISPLAYMODE
 * @brief Provides a granular state representation for various tasks/flight modes @ up to 50Hz
 * @details Typically, use this topic together with \ref TOPIC_STATUS_FLIGHT to get a
 * better understanding of the overall status of the aircraft.
 *
 * @platforms A3,N3, M600,M210
 * @datastruct uint8_t (For detailed enumerations see \ref VehicleStatus::DisplayMode "DisplayMode")
 * @also \ref TOPIC_STATUS_FLIGHT
 */

/*!
 * @var TOPIC_STATUS_LANDINGGEAR
 * @brief Provides status for the landing gear state @ up to 50Hz
 *
 * @platforms A3,N3,M600
 * @datastruct uint8_t (For detailed enumerations see \ref VehicleStatus::LandingGearMode "LandingGearMode")
 */

/*!
 * @var TOPIC_STATUS_MOTOR_START_ERROR
 * @brief If motors failed to start, this topic provides reasons why. Available @ up to 50Hz
 * @platforms A3,N3, M600,M210
 * @datastruct uint8_t (For detailed enumerations see \ref ErrorCode::CommonACK "CommonACK", starting from the 6th element)
 * \note These enumerations show up in the ErrorCode class because they can also be returned as acknowledgements
 * for APIs that start the motors, such as \ref Control::takeoff "Takeoff" or \ref Control::armMotors "Arm"
 */

/*!
 * @var TOPIC_BATTERY_INFO
 * @brief Provides various data about the battery @ up to 50Hz
 * @note Most of these details need a DJI Intelligent battery to work correctly
 * (this is usually not the case with A3/N3 based setups)
 * @details Please be aware that some of the data elements in this topic may not be able to update
 * at high rates due to the limitations of the sensing for that data. e.g. current can only update @ 1 Hz.
 * @platforms A3,N3, M600,M210
 * @datastruct \ref Battery
 */

/*!
 * @var TOPIC_CONTROL_DEVICE
 * @brief Provides states of the aircraft related to SDK/RC control
 * @details The following information is available in this topic:
 * |Data Structure Element| Meaning|
 * |----------------------|--------|
 * |controlMode           |The modes in which the aircraft is being controlled (control loops being applied on horizontal, vertical and yaw axes of the aircraft)|
 * |deviceStatus          |Which device is controlling the motion of the aircraft: RC (Manual control), MSDK (Missions kicked off through mobile), OSDK (Missions kicked off through onboard/ low-level flight control)    |
 * |flightStatus          |Has the OSDK been granted control authority? Since MSDK and RC have precedence, it is possible that deviceStatus shows RC or MSDK actually controlling the aircraft but this value is 1.     |
 * |vrcStatus             |Deprecated|
 * @platforms A3,N3, M600,M210
 * @datastruct \ref SDKInfo
 */

/*!
 * @var TOPIC_HARD_SYNC
 * @brief Provides IMU and quaternion data time-synced with a hardware clock signal @ up to 400Hz.
 * @details This is the only data which can be synchronized with external software or hardware systems. If you want to
 * fuse an external sensor's data with the aircraft's IMU, this data along with a hardware trigger from the A3/N3's
 * expansion ports is how you would do it. You can see detailed documentation on how this process works in the [Hardware
 * Sync Guide](https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-hardware-sync.html).
 * @sensors IMU, sensor fusion output
 * @platforms A3,N3
 * @units
 * |Data Structure Element| Units|
 * |----------------------|--------|
 * |Timestamp |2.5ms, 1ns (See \ref SyncTimestamp)|
 * |Quaternion |rad (after converting to rotation matrix)|
 * |Acceleration |g|
 * |Gyroscope |rad/sec|
 * @datastruct \ref HardSyncData
 *
 */

/*!
 * @var TOPIC_GPS_SIGNAL_LEVEL
 * @brief Provides a measure of the quality of GPS signal @ up to 50Hz
 * @details The level varies from 0 to 5, with 0 being the worst and 5 the best GPS signal. Closely related to
 * TOPIC_GPS_CONTROL_LEVEL
 * @sensors GPS
 * @platforms A3,N3, M600,M210
 * @datastruct uint8_t
 * @also \ref TOPIC_GPS_CONTROL_LEVEL
 */


/*!
 * @var TOPIC_GPS_CONTROL_LEVEL
 * @brief Provides a measure of the quality of GPS signal, with a mechanism for guarding against unset homepoint @ up to 50Hz
 * @details The level varies from 0 to 5, with 0 being the worst and 5 the best GPS signal. The key difference between
 * this and TOPIC_GPS_SIGNAL_LEVEL is that this topic always returns 0 if the homepoint is not set. Once the home point is
 * set, the behavior is exactly the same as TOPIC_GPS_SIGNAL_LEVEL.
 * @sensors GPS
 * @platforms A3,N3, M600,M210
 * @datastruct uint8_t
 * @also \ref TOPIC_GPS_SIGNAL_LEVEL
 */

/*!
 * @var TOPIC_RC_FULL_RAW_DATA
 * @brief Provides raw remote controller stick, buttons and switch data @ up to 50Hz
 * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
 * M600 platform support will be added after a firmware update is released.
 * @details For LB2 and SBUS RCs, this topic will give you:
 * - Stick inputs (R,P,Y,Thr)
 * - Mode switch (P/A/F)
 * - Landing gear switch (Up/Down)
 * - All other buttons (Camera, Video, C1, C2, etc.)
 * @platforms A3,N3,M210
 * @datastruct \ref RCFullRawData, \ref LB2RcFullRawData, \ref SBUSFullRawData
 * @also \ref TOPIC_RC_WITH_FLAG_DATA, TOPIC_RC
 */

/*!
 * @var TOPIC_RC_WITH_FLAG_DATA
 * @brief Provides normalized remote controller stick input data, along with connection status @ up to 50Hz
 * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
 * M600 platform support will be added after a firmware update is released.
 * @details This topic will give you:
 * - Stick inputs (R,P,Y,Thr)
 * - Mode switch (P/A/F)
 * - Landing gear switch (Up/Down)
 * - Connection status for air system, ground system and MSDK apps. The connection status also includes a
 * logicConnected element, which will change to false if either the air system or the ground system radios
 * are disconnected for >3s.
 * - Deadzones near the center of the stick positions are also handled in this topic.
 *
 * @platforms A3,N3,M210
 * @datastruct \ref RCWithFlagData
 * @also \ref TOPIC_RC_FULL_RAW_DATA, TOPIC_RC
 */


/*!
 * @var TOPIC_ESC_DATA
 * @brief Provides raw data from the ESCs @ up to 50Hz
 * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
 * M600 platform support will be added after a firmware update is released.
 * @details This topic supports reporting data for up to 8 ESCs; note that only DJI Intelligent ESCs are supported
 * for this reporting feature. Use this topic to get data on elements close to the hardware - e.g. motor speeds,
 * ESC current and voltage, error flags at the ESC level etc.
 * @platforms A3,N3,M210
 * @datastruct \ref ESCStatusIndividual, \ref EscData
 */


/*!
 * @var TOPIC_RTK_CONNECT_STATUS
 * @brief Provides RTK connection status @ up to 50Hz
 * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
 * M600 platform support will be added after a firmware update is released.
 * @details This topic will update in real time whether the RTK GPS system is connected or not; typical uses
 * include app-level logic to switch between GPS and RTK sources of positioning based on this flag.
 * @platforms A3,M210
 * @datastruct \ref RTKConnectStatus
 */

/*!
 * @var TOPIC_GIMBAL_CONTROL_MODE
 * @brief Provides the mode in which the gimbal will interpret control commands @ up to 50Hz
 * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
 * M600 platform support will be added after a firmware update is released.
 * @details This topic will report the current control mode which can be set in the
 * DJI Go app, MSDK apps, or through Onboard SDK gimbal control APIs (see \ref Gimbal::AngleData "AngleData" struct
 * for more information)
 * @platforms A3,N3,M210
 * @datastruct \ref GimbalControlMode
 */

/*!
 * @var TOPIC_FLIGHT_ANOMALY
 * @brief Provides a number of flags which report different errors the aircraft may encounter in flight @ up to 50Hz
 * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
 * M600 platform support will be added after a firmware update is released.
 * @warning Most of the errors reported by this topic are cases where immediate action is required; you can use these
 * as a baseline for implementing safety-related error-handling routines.
 * @platforms A3,N3,M210
 * @datastruct \ref FlightAnomaly
 */

/*!
 * @var TOPIC_POSITION_VO
 * @brief Provides aircraft's position in a Cartesian frame @ up to 50Hz, without the need for GPS
 * @warning This topic does not follow a standard co-ordinate convention. Please read the details below for usage.
 * @details This is the only topic which can provide positioning information without having a GPS fix; though this
 * can be a big enabler please note the caveats of using this topic:
 * - The topic will use an origin that does not have a global reference, and is not published to the SDK.
 * - The topic uses a combination of VO and compass heading to identify the X-Y axes of its reference frame. This means
 * that if your compass performance is not good in an environment, there is no guarantee the X-Y axes will point to
 * North and East.
 * - The actual directions of the X-Y axes are currently not published to the SDK.
 * - If during a flight the compass performance were to change dramatically, the orientation of the X-Y axes may change
 * to re-align with North-East. The aircraft's position in X and Y may exhibit discontinuities in these cases.
 * - The reference frame is referred to as the Navigation Frame - Cartesian X,Y axes aligned with N,E directions on a best-effort
 * basis, and Z aligned to D (down) direction.
 * - A health flag for each axis provides some granularity on whether this data is valid or not.
 *
 * The key takeaway from these details is that this topic provides a best-effort attempt at providing position
 * information in the absence of absolute references (GPS, compass etc.), without guarantees of consistency if
 * environmental conditions change. So if your application is confined to a stable environment, or if you will
 * have GPS and compass available at all times, this topic can still provide useful data that cannot be otherwise
 * had. If using for control, make sure to have guards checking for the continuity of data.
 *
 * @note Since this topic relies on visual features and/or GPS, if your environment does not provide any of these
 * sources of data, the quality of this topic will reduce significantly. VO data quality will reduce if you are too high
 * above the ground. Make sure that the Vision Positioning System is enabled in DJI Go 4 before using this topic
 * (by default it is enabled).
 * @sensors IMU, VO, GPS(if available), compass
 * @platforms A3,N3,M210
 * @units m
 * @datastruct \ref LocalPositionVO
 */

#endif //ONBOARDSDK_DJI_TELEMETRY_DOC_HPP
