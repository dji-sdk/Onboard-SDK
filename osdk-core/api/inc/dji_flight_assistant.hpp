/** @file dji_flight_assistant.hpp
 *  @version 3.9
 *  @date July 2019
 *
 *  @brief Implementation of flight assistant
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

#ifndef DJI_FLIGHT_ASSISTANT_HPP
#define DJI_FLIGHT_ASSISTANT_HPP

#include "dji_open_protocol.hpp"
namespace DJI {
namespace OSDK {
#define MAX_FLIGHT_HEIGHT 500
#define MIN_GO_HOME_HEIGHT 20
#define MAX_PARAMETER_VALUE_LENGTH 8
#define MAX_FLY_RADIUS 20000
class ControlLink;

/*! @brief Flight control assistant API: set or get parameter
 *
 */
class FlightAssistant {
 public:
  FlightAssistant(Vehicle *vehicle);
  ~FlightAssistant();

 public:
  enum ParamHashValue : uint32_t {
    USE_RTK_DATA = 1288992843,    /*! Set rtk switch on or off*/
    GO_HOME_ALTITUDE = 952919004, /*! Set return home altitude*/
  };

  enum RtkEnableData : uint8_t {
    RTK_DISABLE = 0, /*!< 0: disable */
    RTK_ENABLE = 1,  /*!< 1: enable  */
  };

  typedef uint16_t GoHomeAltitude;

#pragma pack(1)

  enum HomePointType {
    DJI_HOMEPOINT_AIRCRAFT_LOACTON =
        0, /*!< Make aircraft current position as the home point*/
    DJI_HOMEPOINT_SDK_SET_LOCAIION =
        3, /*!< Make custom location as home point */
  };

  typedef struct ParameterData {
    uint32_t hashValue; /*!< parameter's hash value */
    uint8_t paramValue[MAX_PARAMETER_VALUE_LENGTH];
  } ParameterData;

  /*! set parameter and get parameter's return data is same*/
  typedef struct RtkEnableAck {
    uint8_t retCode;
    uint32_t hashValue;
    uint8_t rtkEnable;
  } RtkEnableAck;

  /*! set parameter and get parameter's return data is same*/
  typedef struct GoHomeAltitudeAck {
    uint8_t retCode;
    uint32_t hashValue;
    GoHomeAltitude altitude;
  } GoHomeAltitudeAck;

  typedef struct AvoidObstacleData {
    uint8_t frontBrakeFLag : 1;  /*!< emergency brake flag for front direction,
                                  0:disable, 1:enable*/
    uint8_t rightBrakeFlag : 1;  /*!< emergency brake flag for right direction,
                                  0:disable, 1:enable*/
    uint8_t backBrakeFlag : 1;   /*!< emergency brake flag for back direction,
                                  0:disable, 1:enable*/
    uint8_t leftBrakeFlag : 1;   /*!< emergency brake flag for left direction,
                                  0:disable, 1:enable*/
    uint8_t activeAvoidFlag : 1; /*!< active avoid flag, 0:disable, 1:enable*/
    uint8_t reserve : 3;         /*!< reserve*/
  } AvoidObstacleData;           // pack(1)

  typedef struct SetHomepointData {
    uint8_t type;      /*!< enum-type: HomePointType      */
    double latitude;   /*!< unit:rad, range: -pi/2 ~ pi/2 */
    double longitude;  /*!< unit:rad, range: -pi ~ pi     */
    uint8_t healthy;   /*!< reverse data                  */
  } SetHomepointData;  // pack(1)

#pragma pack()

  /*! @brief type of callback only deal the retCode for user
   */
  typedef struct UCBRetCodeHandler {
    void (*UserCallBack)(ErrorCode::ErrCodeType errCode, UserData userData);
    UserData userData;
  } UCBRetCodeHandler;
  static const int maxSize = 32;
  UCBRetCodeHandler ucbHandler[maxSize];

  /*! @brief struct of callback deal the param and retCode for user
 　*/
  template <typename T>
  struct UCBRetParamStruct {
    void (*UserCallBack)(ErrorCode::ErrCodeType errCode, T param,
                         UserData userData);
    UserData userData;
  };

  /*! @brief type of callback deal the param and retCode for user
   */
  template <typename T>
  using UCBRetParamHandler = UCBRetParamStruct<T>;

  UCBRetCodeHandler *allocUCBHandler(void *callback, UserData userData);

  /*! @brief Set RTK enable or disable, blocking calls
   *
   *  @param rtkEnable RtkEnableData  0: disable, 1: enable
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType setRtkEnableSync(RtkEnableData rtkEnable, int timeout);

  /*! @brief Set RTK enable or disable, non-blocking calls
   *
   *  @param rtkEnable rtkEnableData, 0: disable, 1: enable
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setRtkEnableAsync(RtkEnableData rtkEnable,
                         void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Get rtk enable or disable, blocking calls
   *
   *  @param rtkEnable rtkEnableData, 0: disable, 1: enable
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType getRtkEnableSync(RtkEnableData &rtkEnable,
                                          int timeout);

  /*! @brief get RTK enable or disable, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b rtkEnable rtkEnableData, 0: disable, 1: enable
   *  @arg @b userData the interface to trans userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getRtkEnableAsync(void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                                              RtkEnableData rtkEnable,
                                              UserData userData),
                         UserData userData);

  /*! @brief Set go home altitude, blocking calls
   *
   *  @note If current altitude is higher than settings, aircraft will go home
   *  by current altitude. the altitude setting is between 20m to 500m, if
   *  setting exceed this range, for example setting is 10m or 510m, it will
   * remind you
   * ErrorCode::FlightControllerErr::ParamReadWirteErr::InvalidParameter.
   *  @param altitude go home altitude settings must between MIN_GO_HOME_HEIGHT
   * and MAX_FLIGHT_HEIGHT
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType setGoHomeAltitudeSync(GoHomeAltitude altitude,
                                               int timeout);

  /*! @brief Set go home altitude, non-blocking calls
   *
   *  @note If current altitude is higher than settings, aircraft will go home
   *  by current altitude. the altitude setting is between 20m to 500m, if
   *  setting exceed this range, for example setting is 10m or 510m, it will
   * remind you
   * ErrorCode::FlightControllerErr::ParamReadWirteErr::InvalidParameter.
   *  @param altitude go home altitude
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setGoHomeAltitudeAsync(
      GoHomeAltitude altitude,
      void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Get go home altitude, blocking calls
   *
   *  @param altitude go home altitude
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType getGoHomeAltitudeSync(GoHomeAltitude &altitude,
                                               int timeout);

  /*! @brief Get go home altitude, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b altitude go home altitude
   *  @arg @b userData the interface to trans userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getGoHomeAltitudeAsync(
      void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                           GoHomeAltitude altitude, UserData userData),
      UserData userData);

  /*! @brief Set homepoint position, blocking calls
   *
   *  @note  Set homepoint failed reason may as follows:
   *  1 Use the type DJI_HOMEPOINT_AIRCRAFT_LOACTON, but aircraft's gps level
   *  can't reach the status of record homepoint.
   *  2 The distance between new home point and init home point is larger than
   *  MAX_FLY_RADIUS(20km)
   *  @param homePoint SetHomepointData include latitude and longitude
   *  @param timeout blocking timeout in seconds
   *  @return  OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType setHomePointSync(SetHomepointData homePoint,
                                          int timeout);

  /*! @brief Set home point position, non-blocking calls
   *
   *  @note  Set home point failed reason may as follows:
   *  1. Use the type DJI_HOMEPOINT_AIRCRAFT_LOACTON, but aircraft's gps level
   *  can't reach the status of record homepoint.
   *  2. The distance between new home point and init home point is larger than
   *  MAX_FLY_RADIUS(20km)
   *  @param homePoint  SetHomepointData include latitude and longitude
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param when UserCallBack is called, used in UserCallBack
   */
  void setHomePointAsync(SetHomepointData homePoint,
                         void (*UserCallBack)(ErrorCode::ErrCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Set avoid obstacle switch enable or disable, blocking calls
   *
   *  @param avoidObstacle reference in FlightAssistant::AvoidObstacleData
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrCodeType error code
   */
  ErrorCode::ErrCodeType setAvoidObstacleSwitchSync(
      AvoidObstacleData avoidObstacle, int timeout);

  /*! @brief Set set avoid obstacle switch enable or disable, non-blocking calls
   *
   *  @param avoidObstacle reference in FlightAssistant::AvoidObstacleData
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  OSDK ErrorCode::ErrCodeType error code
   *  @arg @b userData the interface to trans userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setAvoidObstacleSwitchAsync(
      AvoidObstacleData avoidObstacle,
      void (*UserCallBack)(ErrorCode::ErrCodeType retCode, UserData userData),
      UserData userData);

 private:
  ControlLink *controlLink;

  /*! @brief Read parameter table by parameter's hash value, blocking calls
   *
   *  @param hashValue data's hash value
   *  @param data pointer of data
   *  @param data data's length
   *  @param timeout blocking timeout in seconds
   *  @return ParamAck struct of ParamAck
   */
  ErrorCode::ErrCodeType readParameterByHashSync(ParamHashValue hashValue,
                                                 void *param, int timeout);

  template <typename DataT>
  void readParameterByHashAsync(
      ParamHashValue hashValue,
      void (*ackDecoderCB)(Vehicle *vehicle, RecvContainer recvFrame,
                           UCBRetParamHandler<DataT> *ucb),
      void (*userCB)(ErrorCode::ErrCodeType, DataT data, UserData userData),
      UserData userData, int timeout = 500, int retry_time = 2);

  /*! @brief Write parameter table by parameter's hash value, blocking calls
   *
   *  @param hashValue data's hash value
   *  @param data pointer of data
   *  @param data data's length
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrCodeType struct of ErrorCode::ErrCodeType
   */
  ErrorCode::ErrCodeType writeParameterByHashSync(uint32_t hashValue,
                                                  void *data, uint8_t len,
                                                  int timeout);
  /*! @brief Write parameter table by parameter's hash value, non-blocking
   * calls
   *
   *  @param hashValue data's hash value
   *  @param data pointer of data
   *  @param data data's length
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void writeParameterByHashAsync(
      uint32_t hashValue, void *data, uint8_t len,
      void (*ackDecoderCB)(Vehicle *vehicle, RecvContainer recvFrame,
                           UCBRetCodeHandler *ucb),
      void (*userCB)(ErrorCode::ErrCodeType, UserData userData),
      UserData userData, int timeout = 500, int retry_time = 2);
  static void setHomePointAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                     UCBRetCodeHandler *ucb);

  static void avoidObstacleAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                      UCBRetCodeHandler *ucb);
  template <typename AckT>
  static ErrorCode::ErrCodeType commonDataUnpacker(RecvContainer recvFrame,
                                                   AckT &ack);

  static void setParameterDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                  UCBRetCodeHandler *ucb);

  static void getRtkEnableDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                  UCBRetParamHandler<RtkEnableData> *ucb);

  static void getGoHomeAltitudeDecoder(Vehicle *vehicle,
                                       RecvContainer recvFrame,
                                       UCBRetParamHandler<GoHomeAltitude> *ucb);

  /*! @brief Check the altitude of go home setting is valid or not,
   *
   *  @param altitude go home altitude
   *  @return false: invalid, true:valid
   */
  static bool goHomeAltitudeValidCheck(uint16_t altitude);
};
}  // namespace OSDK
}  // namespace DJI
#endif  // DJI_FLIGHT_ASSISTANT_HPP
