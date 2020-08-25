/** @file dji_mobile_device.hpp
 *  @version 4.0.0
 *  @date July 2018
 *
 *  @brief Implementation of DJI Mobile Device Abstraction
 *
 *  @copyright 2017-18 DJI. All rights reserved.
 *
 */

#ifndef MOBILEDEVICE_H
#define MOBILEDEVICE_H

#include "dji_vehicle_callback.hpp"

using namespace DJI::OSDK;

namespace DJI
{
namespace OSDK
{

// Forward Declarations
class Vehicle;

/*! @brief APIs for Mobile-Onboard SDK Communication
 *
 * @details This class implements the Onboard SDK side of
 * Data Transparent Transmission functionality. You must implement APIs
 * available
 * in the Mobile SDK to have full functionality on both directions of the
 * pipeline.
 */
class MobileDevice
{
public:
  MobileDevice(Vehicle* vehicle = 0);
  ~MobileDevice();

  /*
   * Vehicle
   */
public:
  Vehicle* getVehicle() const;
  void setVehicle(Vehicle* value);

private:
  Vehicle* vehicle;

  /*
   * Communication
   */
public:
  /*!
   * @brief sending data from OSDK to MSDK
   *
   * @platforms M210V2, M300
   * @param data sent data
   * @param len length of data
   */
  void sendDataToMSDK(uint8_t* data, uint8_t len);
  static void getDataFromMSDKCallback(Vehicle*      vehiclePtr,
                                      RecvContainer recvFrame,
                                      UserData      userData);

public:
  VehicleCallBackHandler fromMSDKHandler;
  /*!
   * @brief set callback to receive data from MSDK
   *
   * @platforms M210V2, M300
   * @param callback callback to receive data
   * @param userData user data to be passed in callback
   */
  void setFromMSDKCallback(VehicleCallBack callback, UserData userData = 0);
};

} // OSDK
} // DJI

#endif // MOBILEDEVICE_H
