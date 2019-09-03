/** @file dji_ack.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK ACK parsing
 *  @brief ACK error API getError and getErrorCodeMessage
 *  to decode received ACK(s).
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

#ifndef DJI_ACK_HPP
#define DJI_ACK_HPP

#include "dji_command.hpp"
#include "dji_mission_type.hpp"
#include "dji_type.hpp"
#include "dji_version.hpp"
#include <map>

namespace DJI
{
namespace OSDK
{
/*! @brief Class for handling acknowledgements from the aircraft
 * @details The constructs in this class are used to extract data and metadata
 * from the incoming packets,
 * and pack this data into usable return types for blocking API calls.
 *
 */
class ACK
{
public:
#pragma pack(1)
  typedef struct Entry
  {
    uint8_t           cmd_set;
    uint8_t           cmd_id;
    uint16_t          len;
    uint8_t*          buf;
    uint8_t           seqNumber;
    Version::FirmWare version;
  } Entry; // pack(1)

  /*
   * ACK structures not exposed to user
   */

  typedef struct HotPointStartInternal
  {
    uint8_t   ack;
    float32_t maxRadius;
  } HotPointStartInternal; // pack(1)

  typedef struct HotPointReadInternal
  {
    uint8_t          ack;
    HotPointSettings data;

    // TODO fix/remove once verified with FC team
    uint8_t extraByte;
  } HotPointReadInternal; // pack(1)

  typedef struct WayPointAddPointInternal
  {
    uint8_t ack;
    uint8_t index;
  } WayPointAddPointInternal; // pack(1)

  typedef struct WayPointIndexInternal
  {
    uint8_t          ack;
    WayPointSettings data;
  } WayPointIndexInternal; // pack(1)

  typedef struct WayPoint2CommonRsp
  {
    Entry   info;
    bool    updated = false;
  } WayPoint2CommonRsp;

  typedef struct ExtendedFunctionRsp{
    Entry   info;
    bool    updated = false;
  } ExtendedFunctionRsp;

  typedef struct WayPointVelocityInternal
  {
    uint8_t   ack;
    float32_t idleVelocity;
  } WayPointVelocityInternal; // pack(1)

  typedef struct WayPointInitInternal
  {
    uint8_t              ack;
    WayPointInitSettings data;
  } WayPointInitInternal; // pack(1)

  typedef struct MFIOGetInternal
  {
    uint8_t  result;
    uint32_t value;
  } MFIOGetInternal; // pack(1)

  typedef struct ParamAckInternal
  {
    uint8_t retCode;       /*!< return code */
    uint32_t hashValue;    /*!< parameter hash value*/
    uint8_t paramValue[8]; /*!< parameter value*/
  } ParamAckInternal;      // pack(1)

  /*
   * ACK structures exposed to user
   */

  /*! @brief This struct is returned from all blocking calls, except certain
   * mission calls that have explicit types defined later in this file
   * @note NOT TO BE CONFUSED with class \ref DJI::OSDK::ErrorCode "ErrorCode"
   * that contains parsing for acknowledgements
   *
   */
  typedef struct ErrorCode
  {
    Entry    info;
    uint32_t data;
  } ErrorCode; // pack(1)

  /*! @brief This struct is returned from the
   * DJI::OSDK::Control::writeParameterByHash
   * blocking API
   */
  typedef struct ParamAck
  {
    Entry info;
    ParamAckInternal data;
    bool updated = false;
  } ParamAck;  // pack(1)

  /*! @brief This struct is returned from the DJI::OSDK::MFIO::getValue
   * blocking API
   *
   */
  typedef struct MFIOGet
  {
    ErrorCode ack;
    uint32_t  value;
  } MFIOGet; // pack(1)

  /*! @brief This struct is returned from the DJI::OSDK::HotpointMission::start
   * blocking API
   *
   */
  typedef struct HotPointStart
  {
    ErrorCode ack;
    float32_t maxRadius;
  } HotPointStart; // pack(1)

  /*! @brief This struct is returned from the DJI::OSDK::HotpointMission::readData
   * blocking API
   *
   */
  typedef struct HotPointRead
  {
    ErrorCode        ack;
    HotPointSettings data;

    // TODO fix/remove once verified with FC team
    uint8_t extraByte;
  } HotPointRead; // pack(1)

  /*! @brief This struct is returned from the
   * DJI::OSDK::WaypointMission::waypointIndexDownload blocking API
   *
   */
  typedef struct WayPointIndex
  {
    ErrorCode        ack;
    WayPointSettings data;
  } WayPointIndex; // pack(1)

  /*! @brief This struct is returned from the
   * DJI::OSDK::WaypointMission::uploadIndexData blocking API
   *
   */
  typedef struct WayPointAddPoint
  {
    ErrorCode ack;
    uint8_t   index;
  } WayPointAddPoint; // pack(1)

  /*! @brief This struct is returned from the
   * DJI::OSDK::WaypointMission::updateIdleVelocity blocking API
   *
   */
  typedef struct WayPointVelocity
  {
    ErrorCode ack;
    float32_t idleVelocity;
  } WayPointVelocity; // pack(1)

  /*! @brief This struct is used in the readInitData non-blocking API callback
   *
   */
  typedef struct WayPointInit
  {
    ErrorCode            ack;
    WayPointInitSettings data;
  } WayPointInit; // pack(1)

  /*! @brief This struct is returned from the
   * DJI::OSDK::Vehicle::getDroneVersion blocking API
   *
   */
  typedef struct DroneVersion
  {
    ACK::ErrorCode       ack;
    Version::VersionData data;
  } DroneVersion; // pack(1)

  /*!
   * @brief This struct captures PushData while ground-station is enabled on
   * Assistant's SDK Page, CMD: 0x02, 0x04
   */
  typedef struct WayPointReachedData
  {
    uint8_t incident_type;  /*! see WayPointIncidentType */
    uint8_t waypoint_index; /*! the index of current waypt mission */
    uint8_t current_status; /*! 4: pre-action, 6: post-action */
    uint8_t reserved_1;
    uint8_t reserved_2;
  } WayPointReachedData; // pack(1)

  /*!
   * @brief This struct captures PushData while ground-station is enabled on
   * Assistant's SDK Page, CMD: 0x02, 0x03
   */
  typedef struct WayPointStatusPushData{
    uint8_t mission_type;      /*! see WayPointPushDataIncidentType */
    uint8_t waypoint_index;    /*! the index of current waypt mission */
    uint8_t current_status;    /*! 0: pre-mission, 1: in-action, 5: first waypt , 6: reached */
    uint8_t error_notification;
    uint16_t reserved_1;
  } WayPointStatusPushData;
  /*!
   * @brief This constant variable defines number of pixels for QVGA images
   */
  static const int IMG_240P_SIZE = 240 * 320;
  typedef uint8_t  Image[IMG_240P_SIZE];
  /*!
   * @brief sub-struct for stereo image with raw data and camera name
   */
  typedef struct ImageMeta
  {
    Image image;
    char  name[12];
  } ImageMeta; // pack(1)
  /*!
   * @brief This struct captures PushData when subscribe to QVGA images
   */
  typedef struct StereoImgData
  {
    uint32_t frame_index;
    uint32_t time_stamp;
    uint8_t  num_imgs;
    /*
     * There could be 50 different kinds of images coming from the drone,
     * 5 camera pairs and 10 images types.
     * Here we use an uint64_t to describe which image is coming
     * from the USB line, each bit represents if there's data or not
     * Please use AdvancedSensing::ReceivedImgDesc to match them
     * For M210, we support up to 4 images at the same time
     */
    uint64_t img_desc;
    // @note for M210, at most 4 imgs come at the same time.
    ImageMeta img_vec[4];
  } StereoImgData; // pack(1)
  /*!
   * @brief This constant variable defines number of pixels for VGA images
   */
  static const int IMG_VGA_SIZE = 640 * 480;
  typedef uint8_t  VGAImage[IMG_VGA_SIZE];
  /*!
   * @brief This struct captures PushData when subscribe to VGA images
   */
  typedef struct StereoVGAImgData
  {
    uint32_t frame_index;
    uint32_t time_stamp;
    uint8_t  num_imgs;
    uint8_t  direction;
    // @note VGA imgs always come in pair
    VGAImage img_vec[2];
  } StereoVGAImgData; // pack(1)

  /*!
   * @brief This struct captures PushData when subscribe to UTC & FC time in hardware sync
   */
  typedef struct FCTimeInUTC
  {
    uint32_t fc_timestamp_us;
    uint32_t utc_yymmdd;
    uint32_t utc_hhmmss;
  } FCTimeInUTC; // pack(1)

  typedef union TypeUnion {
    uint8_t  raw_ack_array[MAX_INCOMING_DATA_SIZE];
    uint8_t  versionACK[MAX_ACK_SIZE];
    uint16_t ack;
    uint8_t  commandACK;
    uint8_t  missionACK;
    uint8_t  subscribeACK;
    uint8_t  mfioACK;

    /*
     * ACK(s) containing ACK data plus extra payload
     */
    HotPointStartInternal    hpStartACK;
    HotPointReadInternal     hpReadACK;
    WayPointInitInternal     wpInitACK;
    WayPointAddPointInternal wpAddPointACK;
    WayPointIndexInternal    wpIndexACK;
    WayPointVelocityInternal wpVelocityACK;
    MFIOGetInternal          mfioGetACK;
    ParamAckInternal         paramAckData;

    /*
     * Push Data in ground-station mode
     */
    WayPointReachedData wayPointReachedData;
    WayPointStatusPushData wayPointStatusPushData;

    /*
     * Push Data from AdvancedSensing protocol
     */
    StereoImgData           *stereoImgData;
    StereoVGAImgData        *stereoVGAImgData;
    /*
     * Push Data from GPS or RTK
     */
    FCTimeInUTC             fcTimeInUTC;
    uint8_t                 ppsSourceType;

  } TypeUnion; // pack(1)

#pragma pack()

public:
  //! @brief ACK::getError return type when blocking call is successful
  static const bool SUCCESS;
  //! @brief ACK::getError return type when blocking call is unsuccessful
  static const bool FAIL;

  // Since control authority is a single command, we keep track of state in the
  // ack-handling
  // to unify next steps on receiving a control authority ack.
  static const uint8_t OBTAIN_CONTROL  = 1;
  static const uint8_t RELEASE_CONTROL = 0;

public:
  /*! @brief Call this function with an ACK::ErrorCode returned from a blocking
   * call
   *         to find out if the call succeeded or not.
   * @details If the call did not succeed, recommended workflow is to call
   *          getErrorCodeMessage(ack) to parse the failure ack.
   * @param ack ACK::ErrorCode returned from a blocking call
   * @return bool that is SUCCESS if the call was successful, FAIL if the call
   * was not.
   */
  static bool getError(ErrorCode ack);

  /*! @brief Call this function to get a human-readable message that tells you
   * the
   *         meaning of the ACK. Most useful when your getError(ack) call
   * returns FAIL.
   *
   * @param ack ACK::ErrorCode returned from a blocking call.
   * @param func char array for receiving the error message string;
   * @warning param func is not supported in this release, please pass
   * \__func\__ as the value for this argument.
   */
  static void getErrorCodeMessage(ErrorCode ack, const char* func);

private:
  static void getCMDSetActivationMSG(ACK::ErrorCode ack);
  static void getCommonErrorCodeMessage(ACK::ErrorCode ack);
  static void getCMDSetSubscribeMSG(ACK::ErrorCode ack);
  static void getCMDSetControlMSG(ACK::ErrorCode ack);
  static void getMotorErrorMessage(ACK::ErrorCode ack);
  static void getCMDIDSetControlMSG(uint8_t ack, Version::FirmWare version);
  static void getCMDIDControlMSG(ACK::ErrorCode ack);
  static void getCMDIDTaskMSG(ACK::ErrorCode ack);
  static void getCMDIDSetArmMSG(ACK::ErrorCode ack);
  static void getSetBroadcastMSG(ACK::ErrorCode ack);
  static void getCMDSetMissionMSG(ACK::ErrorCode ack);
  static void getCMDSetSyncMSG(ACK::ErrorCode ack);
  static void getCMDSetVirtualRCMSG(ACK::ErrorCode ack);
  static void getCMDSetMFIOMSG(ACK::ErrorCode ack);

  static const std::map<const uint32_t, const char*> createCommonErrorCodeMap();
  static const std::map<const uint32_t, const char*>
  createActivateErrorCodeMap();
  static const std::map<const uint32_t, const char*>
  createSubscribeErrorCodeMap();
  static const std::map<const uint32_t, const char*>
  createSetControlErrorCodeMap();
  static const std::map<const uint32_t, const char*> createTaskErrorCodeMap();
  static const std::map<const uint32_t, const char*>
  createMissionErrorCodeMap();
  static const std::map<const uint32_t, const char*> createMFIOErrorCodeMap();
  static const std::map<const uint32_t, const char*> createSetArmErrorCodeMap();
  static const std::map<const uint32_t, const char*>
  createLegacyTaskErrorCodeMap();
}; // class ACK

} // namespace OSDK
} // namespace DJI
#endif // DJI_ACK_HPP
