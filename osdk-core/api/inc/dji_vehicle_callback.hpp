/** @file dji_vehicle_callback.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Type definition for new Vehicle-style callbacks
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

#ifndef DJI_VEHICLECALLBACK_H
#define DJI_VEHICLECALLBACK_H

#include "dji_open_protocol.hpp"

namespace DJI
{
namespace OSDK
{
class Vehicle;

//! @todo move definition below to class Vehicle
//! so that we could remove this file

/*! @brief Function prototype for all callback functions used in the OSDK
 *
 * @details If you want to register a function as a callback funtion, make sure
 * it matches this prototype.
 *
 */
typedef void (*VehicleCallBack)(Vehicle* vehicle, RecvContainer recvFrame,
                                UserData userData);

/*! @brief The CallBackHandler struct allows users to encapsulate callbacks and
 * data in one struct
 *
 */
typedef struct VehicleCallBackHandler
{
  VehicleCallBack callback;
  UserData        userData;
} VehicleCallBackHandler;

} // namespace OSDK
} // namespace DJI
#endif /* DJI_VEHICLECALLBACK_H */
