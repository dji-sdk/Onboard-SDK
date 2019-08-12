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
#include "dji_control_link.hpp"
namespace DJI {
namespace OSDK {
#define MAX_FLIGHT_HEIGHT 500
#define MAX_PARAMETER_VALUE_LENGTH 8
class ControlLink;
class FlightAssistant {
 public:
  FlightAssistant(ControlLink *controlLink);
  ~FlightAssistant();

 public:
  enum ParamHashValue {
    USE_RTK_DATA = 1288992843,    /*! Set rtk switch on or off*/
    GO_HOME_ALTITUDE = 952919004, /*! Set return home altitude*/
  };

  enum rtkEnableData {
    RTK_DISABLE = 0,         /*!< 0:disable */
    RTK_ENABLE = 1,          /*!< 1:enable  */
    RTK_STATUS_KNOWN = 0xFF, /*!< 0xFF: unknown status*/
  };
  typedef uint16_t goHomeAltitude;

  typedef uint8_t rtkPositionHealth; /*!< 0:not ready,16:single point, 17:rtd,
                                        34:float solution, 50:fixed solution*/

  typedef uint8_t yawHealth; /*!< 0:not ready,16:single point, 17:rtd,34:float
                              solution, 50:fixed solution*/

  typedef uint16_t rtkYaw;

#pragma pack(1)

  typedef struct ParameterData {
    uint32_t hashValue; /*!< parameter's hash value */
    uint8_t paramValue[MAX_PARAMETER_VALUE_LENGTH];
  } ParameterData;

  typedef struct rtkRawPositionData {
    float64_t latitude;  /*!< unit: rad */
    float64_t longitude; /*!< unit: rad */
    float32_t hmsl;      /*!< TODO: 补充含义 */
  } rtkRawPositionData;  // pack(1)

  typedef struct rtkRawVelocityData {
    float32_t velocityNorth;
    float32_t velocityEast;
    float32_t velocityDown;
  } rtkRawVelocityData;  // pack(1)

  typedef struct rtkEnableAck {
    uint8_t retCode;
    uint32_t hashValue;
    uint8_t rtkEnable;
  } rtkEnableAck;

  typedef struct goHomeAltitudeAck {
    uint8_t retCode;
    uint32_t hashValue;
    goHomeAltitude altitude;
  } goHomeAltitudeAck;

  enum HomePointType {
    DJI_HOMEPOINT_AIRCRAFT_LOACTON =
        0, /*!< Make aircraft current position as the homepoint*/
    DJI_HOMEPOINT_RC_LOCATION = 1,  /*!< Make RC's position as the homepoint */
    DJI_HOMEPOINT_APP_LOCATION = 2, /*!< Make APP's position as the homepoint */
    DJI_HOMEPOINT_SDK_SET_LOCAIION =
        3, /*!< Make custom location as homepoint */
  };

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

  typedef struct CommonAck {
    uint8_t ret_code; /*!< original return code from vehicle */
  } CommonAck;        // pack(1)

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

  /*! @brief set RTK enable or disable, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return OSDK united error code
   */
  ErrorCode::ErrCodeType setRtkEnableSync(rtkEnableData rtkEnable, int timeout);

  /*! @brief set RTK eanble or disable, non-blocking calls
   *
   *  @param rtkEnable rtk enable data, 0:disable, 1:enable
   *  @param UserCallBack callback function defined by user
   *  ParamTable is the OSDK united error code
   *  userData the interface to trans userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setRTKEnableAsync(rtkEnableData rtkEnable,
                         void (*UserCallBack)(ErrorCode::ErrCodeType,
                                              UserData userData),
                         UserData userData);

  /*! @brief get rtk enable or disable, blocking calls
   *
   *  @param timeout blocking timeout in seconds
   *  @return OSDK united error code
   */
  ErrorCode::ErrCodeType getRtkEnableSync(rtkEnableData &rtkEnable,
                                          int timeout);
  /*! @brief get RTK eanble or disable, non-blocking calls
   *
   *  @param rtkEnable rtk enable data, 0:disable, 1:enable
   *  @param UserCallBack callback function defined by user
   *  ParamTable is the OSDK united error code
   *  userData the interface to trans userData in when the callback is called
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getRTKEnableAsync(void (*UserCallBack)(ErrorCode::ErrCodeType,
                                              rtkEnableData rtkEnable,
                                              UserData userData),
                         UserData userData);

  /*! @brief Set go home altitude, blocking calls
   *
   *  @note if current altitude is higher than settings, aircraft will go home
   * by current altitude
   *  @param altitude go home altitude
   *  @param timeout blocking timeout in seconds
   *  @return ACK::ParamAck struct of ParamAck
   */
  ErrorCode::ErrCodeType setGoHomeAltitudeSync(goHomeAltitude altitude,
                                               int timeout);

  /*! @brief Set go home altitude, non-blocking calls
   *
   *  @param altitude go home altitude
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setGoHomeAltitudeAsync(goHomeAltitude altitude,
                              void (*UserCallBack)(ErrorCode::ErrCodeType,
                                                   UserData userData),
                              UserData userData);

  /*! @brief Get go home altitude, blocking calls
   *
   *  @param altitude go home altitude
   *  @param timeout blocking timeout in seconds
   *  @return ACK::ParamAck struct of ParamAck
   */
  ErrorCode::ErrCodeType getGoHomeAltitudeSync(goHomeAltitude &altitude,
                                               int timeout);

  /*! @brief Get go home altitude, non-blocking calls
   *
   *  @param altitude go home altitude
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void getGoHomeAltitudeAsync(void (*UserCallBack)(ErrorCode::ErrCodeType,
                                                   goHomeAltitude altitude,
                                                   UserData userData),
                              UserData userData);

  /*! @brief Set homepoint position, blocking calls
   *
   *  @ruturn ACK::ErrorCode's data: 0: set successfully, 1: set failed
   *  @note  Set homepoint failed reason may as follows:
   *  1 Use the type DJI_HOMEPOINT_AIRCRAFT_LOACTON, but aircraft's gps level
   *  can't reach the status of record homepoint.
   *  2 The distance between new home point and init home point is larger than
   *  MAX_FLY_RADIUS(20km)
   */
  ErrorCode::ErrCodeType setHomePointSync(SetHomepointData homePoint,
                                          int wait_timeout);

  /*! @brief Set homepoint position, non-blocking calls
   *
   *  @param data reference in struct of SetHomepointData
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setHomePointAsync(SetHomepointData homePoint,
                         void (*UserCallBack)(ErrorCode::ErrCodeType,
                                              UserData userData),
                         UserData userData);

  /*! @brief Set avoid obstacle switch enable or disable, blocking calls
   *
   *  @param data reference in struct of AvoidObstacleData
   *  @param timeout blocking timeout in seconds
   *  @return ACK::ErrorCode struct with the acknowledgement from the FC
   */
  ErrorCode::ErrCodeType setAvoidObstacleSwitchSync(AvoidObstacleData req,
                                                    int timeout);

  /*! @brief Set set avoid obstacle switch enable or disable, non-blocking calls
   *
   *  @param data reference in struct of AvoidObstacleData
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
  void setAvoidObstacleSwitchAsync(AvoidObstacleData data,
                                   void (*UserCallBack)(ErrorCode::ErrCodeType,
                                                        UserData userData),
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
  /*! @brief Read parameter table by parameter's hash value, non-blocking calls
   *
   *  @param hashValue data's hash value
   *  @param data data's length
   *  @param UserCallBack callback function defined by user
   *  @param userData when UserCallBack is called, used in UserCallBack
   */
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
  static void commonAckDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                               UCBRetCodeHandler *ucb);
  template <typename AckT>
  static ErrorCode::ErrCodeType commonDataUnpacker(RecvContainer recvFrame,
                                                   AckT &ack);
  static void getRTKEnableDecoder(Vehicle *vehicle, RecvContainer recvFrame,
                                  UCBRetParamHandler<rtkEnableData> *ucb);

  static void getGoHomeAltitudeDecoder(Vehicle *vehicle,
                                       RecvContainer recvFrame,
                                       UCBRetParamHandler<goHomeAltitude> *ucb);

  /*! @brief Check the altitude of go home setting is valid or not, non-blocking
   * calls
   *
   *  @param altitude go home altitude
   *  @return 0: invalid, 1:valid
   */
  static bool goHomeAltitudeValidCheck(uint16_t altitude);
};
}  // namespace OSDK
}  // namespace DJI
#endif  // DJI_FLIGHT_ASSISTANT_HPP
