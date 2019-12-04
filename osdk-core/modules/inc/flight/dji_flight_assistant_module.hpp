/** @file dji_flight_assistant_module.hpp
 *  @version 3.9
 *  @date August 2019
 *
 *  @brief Implementation of flight assistant module
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

#ifndef DJI_FLIGHT_ASSISTANT_MODULE_HPP
#define DJI_FLIGHT_ASSISTANT_MODULE_HPP

#include "dji_vehicle_callback.hpp"

namespace DJI {
namespace OSDK {

#define MAX_PARAMETER_VALUE_LENGTH 8
class FlightLink;
class FlightAssistant {
 public:
  FlightAssistant(Vehicle *vehicle);
  ~FlightAssistant();

 public:
  const static uint16_t MAX_FLIGHT_HEIGHT = 500;
  const static uint16_t MIN_GO_HOME_HEIGHT = 20;
  const static uint32_t MAX_FLY_RADIUS = 20000;

  enum ParamHashValue : uint32_t {
    USE_RTK_DATA = 1288992843,    /*! Set rtk switch on or off*/
    GO_HOME_ALTITUDE = 952919004, /*! Set return home altitude*/
  };

  enum RtkEnableData : uint8_t {
    RTK_DISABLE = 0, /*!< 0: disable */
    RTK_ENABLE = 1,  /*!< 1: enable  */
  };

  enum HomeLocationType : uint8_t {
    DJI_HOMEPOINT_AIRCRAFT_LOACTON =
        0, /*!< Make aircraft current position as the home location*/
    DJI_HOMEPOINT_SDK_SET_LOCAIION =
        3, /*!< Make custom location as home location */
  };
  typedef uint16_t GoHomeAltitude; /*!< unit:meter, range 20~500*/

#pragma pack(1)
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

  typedef struct SetHomeLocationData {
    HomeLocationType homeType; /*!< enum-type: HomeLocationType   */
    double latitude;           /*!< unit:rad, range: -pi/2 ~ pi/2 */
    double longitude;          /*!< unit:rad, range: -pi ~ pi     */
    uint8_t healthy;           /*!< reverse data, useless         */
  } SetHomeLocationData;       // pack(1)

#pragma pack()

  /*! @brief type of callback only deal the retCode for user
   */
  typedef struct UCBRetCodeHandler {
    void (*UserCallBack)(ErrorCode::ErrorCodeType errCode, UserData userData);
    UserData userData;
  } UCBRetCodeHandler;

  static const int maxSize = 32;
  UCBRetCodeHandler ucbHandler[maxSize];

  /*! @brief struct of callback deal the param and retCode for user
  　*/
  template <typename T>
  struct UCBRetParamStruct {
    void (*UserCallBack)(ErrorCode::ErrorCodeType errCode, T param,
                         UserData userData);
    UserData userData;
  };

  /*! @brief type of callback deal the param and retCode for user
   */
  template <typename T>
  using UCBRetParamHandler = UCBRetParamStruct<T>;

  UCBRetCodeHandler *allocUCBHandler(void *callback, UserData userData);

  /*! @brief Write parameter table by parameter's hash value, blocking calls
   *
   *  @param hashValue data's hash value
   *  @param data pointer of data
   *  @param len data's length
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType writeParameterByHashSync(uint32_t hashValue,
                                                    void *data, uint8_t len,
                                                    int timeout);
  /*! @brief Write parameter table by parameter's hash value, non-blocking
   * calls
   *
   *  @param hashValue data's hash value
   *  @param data pointer of data
   *  @param len data's length
   *  @param (*ackDecoderCB)function() decoder function
   *  @param (*userCB)function() callback function defined by user
   */
  void writeParameterByHashAsync(
      uint32_t hashValue, void *data, uint8_t len,
      void (*ackDecoderCB)(Vehicle *vehicle, RecvContainer recvFrame,
                           UCBRetCodeHandler *ucb),
      void (*userCB)(ErrorCode::ErrorCodeType, UserData userData),
      UserData userData, int timeout = 500, int retry_time = 2);

  /*! @brief Read parameter table by parameter's hash value, blocking calls
   *
   *  @param hashValue data's hash value
   *  @param data pointer of data
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType readParameterByHashSync(ParamHashValue hashValue,
                                                   void *param, int timeout);

  template <typename DataT>
  void readParameterByHashAsync(
      ParamHashValue hashValue,
      void (*ackDecoderCB)(Vehicle *vehicle, RecvContainer recvFrame,
                           UCBRetParamHandler<DataT> *ucb),
      void (*userCB)(ErrorCode::ErrorCodeType, DataT data, UserData userData),
      UserData userData, int timeout = 500, int retry_time = 2);

  /*! @brief Set RTK enable or disable, blocking calls
   *
   *  @param rtkEnable RtkEnableData  RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setRtkEnableSync(RtkEnableData rtkEnable,
                                            int timeout);

  /*! @brief Set RTK enable or disable, non-blocking calls
   *
   *  @param rtkEnable rtkEnableData, RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode  OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   *  is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setRtkEnableAsync(RtkEnableData rtkEnable,
                         void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              UserData userData),
                         UserData userData);

  /*! @brief Get rtk enable or disable, blocking calls
   *
   *  @param rtkEnable rtkEnableData, RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getRtkEnableSync(RtkEnableData &rtkEnable,
                                            int timeout);

  /*! @brief get RTK enable or disable, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b rtkEnable rtkEnableData, RTK_DISABLE: disable, RTK_ENABLE: enable
   *  @arg @b userData the interface to pass userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getRtkEnableAsync(void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                                              RtkEnableData rtkEnable,
                                              UserData userData),
                         UserData userData);

  /*! @brief Set go home altitude, blocking calls
   *
   *  @note If aircraft's current altitude is higher than the setting value of
   * go home altitude, aircraft will go home using current altitude. Otherwise,
   * it will climb to setting of go home altitude ,and then execute go home
   * action. The details could be find in the documentation.
   * Go home altitude setting is between MIN_GO_HOME_HEIGHT to
   * MAX_FLIGHT_HEIGHT.
   * @param altitude go home altitude settings, unit: meter
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setGoHomeAltitudeSync(GoHomeAltitude altitude,
                                                 int timeout);

  /*! @brief Set go home altitude, non-blocking calls
   *
   *  @note If aircraft's current altitude is higher than the setting value of
   * go home altitude, aircraft will go home using current altitude. Otherwise,
   * it will climb to setting of go home altitude ,and then execute go home
   * action. The details could be find in the documentation.
   * Go home altitude setting is between MIN_GO_HOME_HEIGHT to
   * MAX_FLIGHT_HEIGHT.
   * @param altitude go home altitude settings, unit: meter
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to pass userData in when the callback is
   * called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setGoHomeAltitudeAsync(
      GoHomeAltitude altitude,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

  /*! @brief Get go home altitude, blocking calls
   *
   *  @param altitude go home altitude
   *  @param timeout blocking timeout in seconds
   *  @return OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType getGoHomeAltitudeSync(GoHomeAltitude &altitude,
                                                 int timeout);

  /*! @brief Get go home altitude, non-blocking calls
   *
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b altitude go home altitude
   *  @arg @b userData the interface to pass userData in when the callback is
   *  called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getGoHomeAltitudeAsync(
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode,
                           GoHomeAltitude altitude, UserData userData),
      UserData userData);

  /*! @brief Set home location, blocking calls
   *
   *  @note  Set home location failed reason may as follows:
   *  1 Set init home location failed after start aircraft.
   *  2 Use the type DJI_HOMEPOINT_AIRCRAFT_LOACTON, but aircraft's gps level
   *  can't reach the status of record home location.
   *  3 Use the type DJI_HOMEPOINT_SDK_SET_LOCAIION, but the distance between
   *  new home location and last home location is larger than
   *  MAX_FLY_RADIUS(20km)
   *  @param homeLocation reference in  SetHomeLocationData
   *  @param timeout blocking timeout in seconds
   *  @return  OSDK ErrorCode::ErrorCodeType error code
   */
  ErrorCode::ErrorCodeType setHomeLocationSync(SetHomeLocationData homeLocation,
                                               int timeout);

  /*! @brief Set home location, non-blocking calls
   *
   *  @note  Set home location failed reason may as follows:
   *  1 Set init home location failed after start aircraft.
   *  2 Use the type DJI_HOMEPOINT_AIRCRAFT_LOACTON, but aircraft's gps level
   *  can't reach the status of record home location.
   *  3 Use the type DJI_HOMEPOINT_SDK_SET_LOCAIION, but the distance between
   *  new home location and last home location is larger than
   *  MAX_FLY_RADIUS(20km)
   *  @param homeLocation reference in  SetHomeLocationData
   *  @param homeLocation  reference in SetHomeLocationData
   *  @param UserCallBack callback function defined by user
   *  @arg @b retCode the OSDK ErrorCode::ErrorCodeType error code
   *  @arg @b userData the interface to transfer userData in when the callback
   * is called
   *  @param when UserCallBack is called, used in UserCallBack
   */
  void setHomeLocationAsync(
      SetHomeLocationData homeLocation,
      void (*UserCallBack)(ErrorCode::ErrorCodeType retCode, UserData userData),
      UserData userData);

 private:
  FlightLink *flightLink;

  template <typename AckT>
  static ErrorCode::ErrorCodeType commonDataUnpacker(RecvContainer recvFrame,
                                                     AckT &ack);
  static void setParameterDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                  UCBRetCodeHandler *ucb);

  static void getRtkEnableDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                  UCBRetParamHandler<RtkEnableData> *ucb);

  static void getGoHomeAltitudeDecoder(Vehicle *vehicle,
                                       RecvContainer recvFrame,
                                       UCBRetParamHandler<GoHomeAltitude> *ucb);
  static void setHomePointAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                     UCBRetCodeHandler *ucb);
  /*! @brief Check the altitude of go home setting is valid or not,
   *
   *  @param altitude go home altitude
   *  @return false: invalid, true:valid
   */
  static bool goHomeAltitudeValidCheck(uint16_t altitude);
};
}
}

#endif  // DJI_FLIGHT_ASSISTANT_MODULE_HPP
