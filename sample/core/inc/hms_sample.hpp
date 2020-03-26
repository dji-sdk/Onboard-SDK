/** @file dji_hms.hpp
 *  @version 4.0
 *  @date Dec 2019
 *
 *  @brief
 *  HMS sample use HMS API in a Linux environment..
 *  Provides a number of helpful additions to core API calls,
 *  like API of subscribing flight status.
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

#ifndef ONBOARDSDK_HMSSAMPLE_H
#define ONBOARDSDK_HMSSAMPLE_H

#include <dji_linux_helpers.hpp>
#include "dji_vehicle.hpp"
#include <string>

class HMSSample {
public:
    HMSSample(Vehicle *vehicle = 0);
    ~HMSSample();

    Vehicle *vehicle;

  /*! @brief The interface of getting hms version
   *
   *  @return std::string hms version.(for example,HMS1.0.0)
   */
    std::string getHMSVersion();

  /*! @remark Blocks until ACK frame arrives or timeout occurs
   *
   *  @brief Send subscribe request to your flight controller
   *         to get HMS(Health Management System)'s information,
   *         blocking calls
   *
   *  @param enable whether subscribe HMS Info
   *       0:reset aLL subcriber except APP
   *       1:subscribe all HMS's information
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

  /*! @brief subscribe flight's status in 10HZ
   *
   *  @param pkgIndex index of subscribe package
   *
   *  @return bool whether subscribe success
   *  true:success
   */
    bool subscribeFlightStatus(const int pkgIndex);

    /*! @brief unsubscribe flight's status
     *
     *  @param pkgIndex index of subscribe package
     *
     *  @return bool whether teardown success
     *  true:success
     */
    bool unsubscribeFlightStatus(const int pkgIndex);

    /*! @brief The interface of getting flight status
     *
     *  @return uint8_t flight status.
     *  0:STOPED
     *  1:ON_GROUND
     *  2:IN_AIR
     *
     *  @note After successful subscribing, the data will be valid.
     */
    uint8_t getFlightStatus(void);

    /*! @brief print all raw pushed data
     *
     *  @note After successful subscribing, the data will be valid.
     */
    void printAllError(void);
};

#endif //ONBOARDSDK_HMSSAMPLE_H
