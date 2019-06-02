/** @file dji_payload_device.hpp
 *  @version 3.8.1
 *  @date  May 2019
 *
 *  @brief Implementation of DJI Payload Device Abstraction
 *
 *  @copyright 2017-19 DJI. All rights reserved.
 *
 */
#ifndef PAYLOAD_DEVICE_HPP
#define PAYLOAD_DEVICE_HPP

#include "dji_vehicle_callback.hpp"

using namespace DJI::OSDK;

namespace DJI
{
  namespace OSDK
  {

// Forward Declarations
    class Vehicle;

/*! @brief APIs for Payload-Onboard SDK Communication
 *
 * @details This class implements the Onboard SDK side of
 * Data Transparent Transmission functionality. You must implement APIs
 * available
 * in the Payload SDK to have full functionality on both directions of the
 * pipeline.
 */
    class PayloadDevice
    {
    public:
      PayloadDevice(Vehicle* vehicle = 0);
      ~PayloadDevice();

      /*
       * Vehicle
       */
    public:
      Vehicle* getVehicle() const;
      void setVehicle(Vehicle* value);
    public:
      const static uint16_t   MAX_SIZE_OF_PACKAGE = 255;
    private:
      Vehicle* vehicle;

      /*
       * Communication
       */
    public:
      /*!
       * @brief sending data from OSDK to PSDK
       *
       * @param data sent data
       * @param len length of data
       */
      void sendDataToPSDK(uint8_t* data, uint16_t len);
      static void getDataFromPSDKCallback(Vehicle*      vehiclePtr,
                                          RecvContainer recvFrame,
                                          UserData      userData);

    public:
      VehicleCallBackHandler fromPSDKHandler;
      void setFromPSDKCallback(VehicleCallBack callback, UserData userData = 0);
    };

  } // OSDK
} // DJI

#endif //PAYLOAD_DEVICE_HPP


