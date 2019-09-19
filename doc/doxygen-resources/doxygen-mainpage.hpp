//
// Created by rohit on 6/7/17.
//

#ifndef ONBOARDSDK_DOXYGEN_MAINPAGE_HPP
#define ONBOARDSDK_DOXYGEN_MAINPAGE_HPP

/*! @mainpage
 * This section provides API Reference for the DJI Onboard SDK (OSDK). The OSDK
 * provides a set of APIs for implementing the various functionality available through
 * the DJI Open Protocol on compatible products from the Matrice series and stand-alone flight controllers.
 *
 * @section intro_sec Introduction
 *
 * Click on the Files/Classes/Modules tabs above to see more information about the
 * API within the OSDK. To get started, follow the [Getting Started](#get-started) section below. \n
 * The complete documentation for the SDK can be found on the [DJI Developer
 * Website](https://developer.dji.com/onboard-sdk/documentation/).
 *
 * @section get-started Getting Started
 *
 * To get started, click on the various links here to go to the API reference
 * of the main features of the SDK
 *
 * ## Vehicle Class
 * This is your most important resource for any SDK application - an object of type \ref DJI::OSDK::Vehicle "Vehicle" instantiates a serial driver, threads, a protocol parser and objects for all the features described below.
 *
 * ## Features
 *
 * Developers can gain an end-to-end understanding of each feature, from the high-level ideas all the way to the API documentation, using this table:
 *
 * | Component                                    | Feature Documentation                                                                                                                                                                           | Related API Groups                                         | Description                                                                                              |
 * |----------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------|----------------------------------------------------------------------------------------------------------|
 * | **Data Telemetry**                           | [Guide](/onboard-sdk/documentation/guides/component-guide-telemetry.html) and [Sample](/onboard-sdk/documentation/sample-doc/telemetry.html)                                                    | \ref  telem "Telemetry"                                    | Namespace encapsulating all telemetry topics and data structures                                         |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::DataBroadcast "Broadcast"                  | Class providing old broadcast-style push data telemetry (APIs and data structures)                       |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::DataSubscription "Subscription"            | Class providing new subscription-style telemetry (APIs and data structures)                              |
 * | **Flight Control**                           | [Guide](/onboard-sdk/documentation/guides/component-guide-flight-control.html) and [Sample](/onboard-sdk/documentation/sample-doc/flight-control.html)                                          | \ref DJI::OSDK::Control "Control"                          | Class with APIs & data structures for low-level control modes and high-level flight actions, this class will be replaced by FlightController step by step |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::FlightController "FlightController"        | Class with APIs & data structures for flight assistant and flight actions.                               |
 * | **GPS Missions**                             | [Guide](/onboard-sdk/documentation/guides/component-guide-missions.html) and [Sample](/onboard-sdk/documentation/sample-doc/missions.html)                                                      | \ref DJI::OSDK::MissionManager "MissionManager"            | Manager class that handles waypoint and hotpoint mission instantiations.                                 |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::WaypointMission "WaypointMission"          | Class exposing APIs for GPS waypoint missions                                                            |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::HotpointMission "HotpointMission"          | Class exposing APIs for GPS hotpoint (Point of Interest) missions                                        |
 * | **Camera/Gimbal**                            | [Guide](/onboard-sdk/documentation/guides/component-guide-camera-and-gimbal.html) and [Sample](/onboard-sdk/documentation/sample-doc/camera-gimbal-control.html)                                | \ref DJI::OSDK::Camera "Camera"                            | Class exposing camera action APIs & data structures                                                      |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::Gimbal "Gimbal"                            | Class exposing gimbal motion control APIs & data structures                                              |
 * |                                              | [Sample](/onboard-sdk/documentation/sample-doc/what-is-camera-manager.html)                                                                                                                     | \ref DJI::OSDK::CameraManager "CameraManager"              | Class exposing APIs & data structures for camera actions, parameters setting and getting                 |
 * | **Multi-Function IO**                        | [Guide](/onboard-sdk/documentation/guides/component-guide-multi-function-io.html) and [Sample](/onboard-sdk/documentation/sample-doc/mfio.html)                                                 | \ref DJI::OSDK::MFIO "MFIO"                                | Class with APIs & data structures for configuring, reading from and writing to MFIO channels             |
 * | **Hardware Synchronization**                 | [Guide](/onboard-sdk/documentation/guides/component-guide-hardware-sync.html)                                                                                                                   | \ref DJI::OSDK::HardwareSync "HardwareSync"                | Class for configuring a hardware sync pulse from a physical port                                         |
 * | **[Mobile SDK](/mobile-sdk/) Communication** | [Guide](/onboard-sdk/documentation/guides/component-guide-mobile-communication.html) and [Sample](/onboard-sdk/documentation/sample-doc/msdk-comm.html)                                         | \ref DJI::OSDK::MobileCommunication "MobileCommunication"  | Class providing APIs & data structures for upstream and downstream communication with the DJI Mobile SDK |
 * | **[Payload SDK](/paylod-sdk/) Communication**| [Guide](/onboard-sdk/documentation/guides/component-guide-payload-communication.html) and [Sample](/onboard-sdk/documentation/sample-doc/psdk-comm.html)                                        | \ref DJI::OSDK::PayloadDevice "PayloadCommunication"       | Class providing APIs & data structures for upstream and downstream communication with the DJI Payload SDK |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::PSDKManager "PSDKManager"                  | Class providing APIs & data structures for upstream and downstream data and widgets values communication with the DJI Payload SDK |
 * | **Status, Error and ACKs**                   | Each API in this API reference documentation has corresponding ACKs & errors. Statuses are obtained from telemetry.                                                                             | \ref DJI::OSDK::VehicleStatus "VehicleStatus"              | Enums for the flight mode, landing gear and API control mode                                             |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::ErrorCode "ErrorCode"                      | Sub-Classes that group API return values by command set (equivalent to a component)                      |
 * |                                              |                                                                                                                                                                                                 | \ref DJI::OSDK::ACK "ACK"                                  | Class containing data structures for non-standard ACKs that a few APIs receive from the aircraft/FC      |
 * |                                              |                                                                                                                                                                                                 | \ref dji_mission_type.hpp "dji_mission_type.hpp"           | File containing data structures shared between GPS Mission commands (OSDK --> FC) and ACKs (FC --> OSDK) |
 * | **Advanced Sensing**                         | [Guide](/onboard-sdk/documentation/guides/component-guide-advanced-sensing-stereo-camera.html) and [Sample](/onboard-sdk/documentation/sample-doc/advanced-sensing-stereo-images.html)          | \ref DJI::OSDK::AdvancedSensing "AdvancedSensing"          | Class providing access to stereo camera, FPV camera and main camera from M210 and M210-RTK               |
 *
 */

#endif //ONBOARDSDK_DOXYGEN_MAINPAGE_HPP
