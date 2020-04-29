/** @file dji_hms_impl.hpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief HMS(Health Management System) API for DJI OSDK implement
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

#ifndef RESOURCES_DJI_HMS_IMPL_H
#define RESOURCES_DJI_HMS_IMPL_H

#include <iostream>
#include <string>
#include "dji_type.hpp"
#include "dji_hms.hpp"
#include "dji_telemetry.hpp"

namespace  DJI{
namespace OSDK{

// Forward Declarations
class Vehicle;

class DJIHMSImpl {
public:
    DJIHMSImpl(Vehicle *vehicle);
    ~DJIHMSImpl();

    Vehicle* vehicle;

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
    HMSPushPacket& getHMSPushPacket();

  /*! @brief The interface of getting device(camera or gimbal) index
   *
   *  @return uint8_t the private parameter deviceIndex which
   *  represents camera's or gimbal's index.
   *
   *  @note After successful subscribing and registering, the data will be valid.
   *  if device is camera(payload) or gimbal ,the data will be valid;otherwise it will be 0xff(invalid).
   */
    uint8_t  getDeviceIndex();

    void setHMSPushData(const uint8_t *hmsPushData, uint16_t dataLen);
    void setHMSTimeStamp();
    void setDeviceIndex(uint8_t sender);

    bool createHMSInfoLock();
    bool lockHMSInfo();
    bool freeHMSInfo();
    bool destroyHMSInfoLock();
private:
    HMSPushPacket hmsPushPacket;
    /*! camera(payload) or gimbal index*/
    uint8_t  deviceIndex;

    T_OsdkMutexHandle m_hmsLock;

    /*! @brief get camera(payload)'s or gimbal's index(same with deviceindex) by sender
     *
     * @return uint8_t component index.(cameral index or gimbal index)
     *
     */
    uint8_t getComponentIndex(uint8_t sender);
};
    }// OSDK
} //DJI


#endif //RESOURCES_DJI_HMS_IMPL_H