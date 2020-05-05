/** @file dji_hms.hpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief HMS(Health Management System) API for DJI OSDK
 *  @Details Provide API to subscribe Flight's Health State.
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

#ifndef ONBOARDSDK_DJI_HMS_H
#define ONBOARDSDK_DJI_HMS_H

#include "dji_type.hpp"
#include <vector>


namespace  DJI{
namespace OSDK{

// Forward Declarations
class Vehicle;
class DJIHMSImpl;

#pragma pack(1)
/*! the type of error code list in HMS's raw pushing data*/
typedef  struct ErrList{
    uint32_t alarmID;     /*! error code*/
    uint8_t  sensorIndex; /*! fault sensor's index*/
    uint8_t  reportLevel; /*! fault level ,0-4,0 is no error,4 is highest*/
} ErrList;

/*! the type of HMS's raw pushing data*/
typedef struct HMSPushData {
    uint8_t msgVersion;           /*! version of message.default:0*/
    uint8_t globalIndex;          /*! cycle message sequence number*/
    uint8_t msgEnd : 1;           /*! end flag bit of message subcontract push. last package of subcontracting push is 1.*/
    uint8_t msgIndex : 7;         /*! Message serial number*/
    std::vector<ErrList> errList; /*! error code list in each pushing*/
} HMSPushData;
#pragma pack()

/*! the type of HMS's pushing data with a time stamp*/
typedef struct HMSPushPacket
{
    HMSPushData  hmsPushData; /*! HMS's raw pushing data*/
    uint32_t     timeStamp;   /*! timestamp of the packet*/
} HMSPushPacket;

typedef enum
{
    CameraIndex1 = 1,
    CameraIndex2 = 2,
    CameraIndex3 = 3,
} CameraIndex;

typedef enum
{
    GimbalIndex1 = 1,
    GimbalIndex2 = 2,
    GimbalIndex3 = 3,
    InvalidIndex = 0xff,
} GimbalIndex;

typedef enum
{
    ResetALLSubcriberExceptAPP = 0,
    OSDKSubALLData = 1,
} HMSSubcribeCmd;

typedef enum
{
    OSDKSubSuccess = 0,
} HMSSubcribeStatus;

/*! @brief DJI health manager system of drone
 */
class DJIHMS {
public:
    DJIHMS(Vehicle *vehicle = 0);

    ~DJIHMS();

  /*! @remark Blocks until ACK frame arrives or timeout occurs
   *
   *  @brief Send subscribe request to your flight controller
   *         to get HMS(Health Management System)'s information,
   *         blocking calls
   *
   *  @param enable whether subscribe HMS Info
   *       false:reset aLL subcriber except APP
   *       true:subscribe all HMS's information
   *  @param timeOutMs blocking time to wait for ACK, unit:ms
   *
   *  @return bool subscribe's ack result
   *     true:subscribe/unsubscribe success
   *    false:subscribe/unsubscribe failed
   *
   *  @note if the function return true,it will print "Subscribe/Unsubscribe all flight data success!",
   *        else,it will printf "Subscribe/Unsubscribe all flight data failed!"
   */
    bool subscribeHMSInf(bool enable, uint32_t timeOutMs = 500);

  /*! @brief The interface of getting hms version
   *
   *  @return std::string hms version.(for example,HMS1.0.0)
   */
    std::string getHMSVersion();

  /*! @brief The interface of getting HMS's pushing data which has a timestamp
   *
   *  @return HMSPushPacket the private parameter hmsPushPacket which
   *  represents HMS's pushing data with a time stamp.
   *
   *  @note The push data consists of the raw pushing data and a timestamp.
   *  After successful subscribing and registering, the data will be valid.
   */
    HMSPushPacket getHMSPushPacket();

  /*! @brief The interface of getting device(camera or gimbal) index
   *
   *  @return uint8_t the private parameter deviceIndex which
   *  represents camera's or gimbal's index.
   *
   *  @note After successful subscribing and registering, the data will be valid.
   *  if device is camera(payload) or gimbal ,the data will be valid;otherwise it will be 0xff(invalid).
   */
    uint8_t  getDeviceIndex();

private:
    Vehicle *vehicle;
    DJIHMSImpl *djiHMSImpl;

  /*! @brief A register callback function for getting HMS(Health Management System)'s push data
   *
   *  @details After successful subscribing, this function would be used to register link's
   *  information for getting HMS's pushing data.(1HZ)
   *
   *  @return bool whether register success
   *  true:success
   */
    bool enableListeningHmsData(bool enable);
};
  }
}
#endif //ONBOARDSDK_DJI_HMS_H
