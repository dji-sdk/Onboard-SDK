/*! @file mobile_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Mobile SDK Communication API usage in a Linux environment.
 *  Shows example usage of the mobile<-->onboard SDK communication API.
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

#ifndef DJIOSDK_MOBILESAMPLE_HPP
#define DJIOSDK_MOBILESAMPLE_HPP

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

// Other samples for missions implementation
#include <camera_gimbal_sample.hpp>
#include <flight_control_sample.hpp>
#include <mission_sample.hpp>

// Data strucutres to send data back to mobile
#pragma pack(1)
typedef struct AckReturnToMobile
{
  uint16_t cmdID;
  uint16_t ack;
} AckReturnToMobile;

typedef struct VersionMobilePacket
{
  uint16_t cmdID;
  char     version[38];

  VersionMobilePacket(uint16_t cmdID, char* versionPack);
} VersionMobilePacket;
#pragma pack()

// Handlers for processing acknowledgements obtained from calls made by parsing
// mobile commands

void controlAuthorityMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                    DJI::OSDK::RecvContainer recvFrame,
                                    DJI::OSDK::UserData      userData);
void actionMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                          DJI::OSDK::RecvContainer recvFrame,
                          DJI::OSDK::UserData      userData);

void activateMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                            DJI::OSDK::RecvContainer recvFrame,
                            DJI::OSDK::UserData      userData);

/*! @brief A function to get cached drone version when getVersion is called
 * through mobile
   *  @param receivedFrame: RecvContainer populated by the protocolLayer
   *  @return NULL
   */
void sendDroneVersionFromCache(DJI::OSDK::Vehicle* vehiclePtr);

// Mission Calls
bool runPositionControlSample(DJI::OSDK::Vehicle* vehicle);

// Main parser for incoming mobile data. This parser will decide what
// vehicle API calls to make.
void parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                             DJI::OSDK::RecvContainer recvFrame,
                             DJI::OSDK::UserData      userData);

// Register handlers with vehicle
bool setupMSDKParsing(DJI::OSDK::Vehicle* vehicle,
                      LinuxSetup*         linuxEnvironment);

// Utility functions
void sendAckToMobile(DJI::OSDK::Vehicle* vehicle, uint16_t cmdID,
                     uint16_t ack = 0);

pthread_t setupSamplePollingThread(DJI::OSDK::Vehicle* vehicle);
void* mobileSamplePoll(void* vehiclePtr);

void setTestSuite(int testSuiteNumber);

#endif // DJIOSDK_MOBILESAMPLE_HPP
