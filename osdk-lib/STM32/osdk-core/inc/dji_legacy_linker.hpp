/** @file dji_legacy_linker.hpp
 *  @version 4.0
 *  @date November 2019
 *
 *  @brief
 *  Legacy adapting in linker for OSDK 3.9
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

#ifndef LEGACY_LINKER_H_
#define LEGACY_LINKER_H_

#include "dji_vehicle_callback.hpp"

/*! Platform includes:
 *  This set of macros figures out which files to include based on your
 *  platform.
 */
#ifdef __linux__
#include <cstring>
#elif STM32
//! handle array of characters
#include <stdlib.h>
#include <string.h>
#endif

namespace DJI
{
namespace OSDK
{
// Forward Declaration
class Vehicle;

/****************************Globals**************************************/

#define MSG_ENABLE_FLAG_LEN 2

//----------------------------------------------------------------------
// App layer function
//----------------------------------------------------------------------

typedef struct
{
  uint16_t sequence_number;
  uint8_t  session_id : 5;
  uint8_t  need_encrypt : 1;
  uint8_t  reserve : 2;
} req_id_t;

#define SET_CMD_SIZE (2u)

//----------------------------------------------------------------------
// Session Management
//----------------------------------------------------------------------

#define ACK_SESSION_IDLE 0
#define ACK_SESSION_PROCESS 1
#define ACK_SESSION_USING 2
#define CMD_SESSION_0 0
#define CMD_SESSION_1 1
#define CMD_SESSION_AUTO 32

#define POLL_TICK 20 // unit is ms

//----------------------------------------------------------------------
// Codec Management
//----------------------------------------------------------------------

// @todo replace this
#define _SDK_U32_SET(_addr, _val) (*((uint32_t*)(_addr)) = (_val))
#define _SDK_U16_SET(_addr, _val) (*((uint16_t*)(_addr)) = (_val))

class OpenProtocol {

 public:

  /********************* Useful protocol related constants ******************/
  static const int ACK_SIZE = 10;
  static const uint8_t SOF = 0xAA;
  static const int CRCHead = sizeof(uint16_t);
  static const int CRCData = sizeof(uint32_t);
  static const int CRCHeadLen = sizeof(OpenHeader) - CRCHead;
  static const int PackageMin = sizeof(OpenHeader) + CRCData;
};

class LegacyLinker
{
public:
  //! Constructor
  LegacyLinker(Vehicle* vehicle);

  //! Destructor
  ~LegacyLinker();

  /************************** Init ******************************************/
public:
  void init();

  /*******************************Send Pipeline*****************************/
public:
  void send(const uint8_t cmd[], void *pdata, size_t len);

  void sendAsync(const uint8_t cmd[], void *pdata, size_t len, int timeout,
                 int retry_time, VehicleCallBack callback, UserData userData);

  void* sendSync(const uint8_t cmd[], void *pdata, size_t len,
                          int timeout, int retry_time);

  bool registerCMDCallback(uint8_t cmdSet, uint8_t cmdID,
                           VehicleCallBack &callback, UserData &userData);

 private:
  Vehicle* vehicle;

  void initX5SEnableThread();
  void *decodeAck(E_OsdkStat ret, uint8_t cmdSet, uint8_t cmdId,
                  RecvContainer recvFrame);
 private:
  uint8_t rawVersionACK[MAX_ACK_SIZE];

  // User space ACK types
  ACK::ErrorCode     ackErrorCode;
  ACK::DroneVersion  droneVersionACK;
  ACK::HotPointStart hotpointStartACK;
  ACK::HotPointRead  hotpointReadACK;
  /*!WayPoint download command
   * @note Download mission setting*/
  ACK::WayPointInit waypointInitACK;
  /*!WayPoint index download command ACK
   * @note Download index settings*/
  ACK::WayPointIndex      waypointIndexACK;
  ACK::WayPoint2CommonRsp wayPoint2CommonRspACK;
  /*!WayPoint add point command ACK*/
  ACK::WayPointAddPoint waypointAddPointACK;
  ACK::MFIOGet          mfioGetACK;
  ACK::ExtendedFunctionRsp extendedFunctionRspAck;
  ACK::ParamAck         paramAck;
  ACK::SetHomeLocationAck setHomeLocationAck;
  /*! Heart Beat Ack*/
  ACK::HeartBeatAck     heartBeatAck;

  T_OsdkTaskHandle legacyX5SEnableHandle;
  static void *legacyX5SEnableTask(void *arg);
}; // class LegacyLinker

} // namespace OSDK
} // namespace DJI

#endif // LEGACY_LINKER_H_
