/** @file dji_mobile_communication.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of DJI Mobile-Onboard SDK Communication (MOC)
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

#ifndef MOC_H
#define MOC_H

#include "dji_macros.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declarations
class Vehicle;

/*! @brief APIs for Mobile-Onboard SDK Communication
 *
 * @details This class is deprecated, please use the new class \ref MobileDevice
 * instead.
 * This class implements the Onboard SDK side of Data Transparent Transmission
 * functionality.
 * You must implement APIs available in the Mobile SDK to have full
 * functionality on both directions of the
 * pipeline.
 */
class MobileCommunication
{
public:
  MobileCommunication(Vehicle* vehicle = 0);
  ~MobileCommunication();

  Vehicle* getVehicle() const;
  void setVehicle(Vehicle* value);

public:
  /*!
   * @brief sending data from OSDK to MSDK
   *
   * @param data sent data
   * @param len length of data
   */
  void sendDataToMSDK(uint8_t* data, uint8_t len);
  static void getDataFromMSDKCallback(Vehicle*      vehiclePtr,
                                      RecvContainer recvFrame,
                                      UserData      userData);

public:
  VehicleCallBackHandler fromMSDKHandler;
  void setFromMSDKCallback(VehicleCallBack callback, UserData userData = 0);

private:
  Vehicle* vehicle;
};

} // OSDK
} // DJI

#endif // MOC_H
