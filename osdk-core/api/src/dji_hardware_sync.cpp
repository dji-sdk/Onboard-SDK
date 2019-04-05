/** @file dji_hardware_sync.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Hardware Sync API for DJI OSDK
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

#include "dji_hardware_sync.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

HardwareSync::HardwareSync(Vehicle* vehiclePtr)
  : vehicle(vehiclePtr)
{
  ppsNMEAHandler.callback = 0;
  ppsNMEAHandler.userData = 0;
  ppsSourceHandler.callback = 0;
  ppsSourceHandler.userData = 0;
  ppsUTCFCTimeHandler.callback = 0;
  ppsUTCFCTimeHandler.userData = 0;
  ppsUTCTimeHandler.callback = 0;
  ppsUTCTimeHandler.userData = 0;
}

void
HardwareSync::setSyncFreq(uint32_t freqInHz, uint16_t tag)
{
  SyncSettings data;
  data.freq = freqInHz;
  data.tag  = tag;
  startSync(data);
}

void
HardwareSync::startSync(SyncSettings& data)
{
  vehicle->protocolLayer->send(0, vehicle->getEncryption(),
                               OpenProtocolCMD::CMDSet::HardwareSync::broadcast,
                               &data, sizeof(data));
}

void
HardwareSync::subscribeNMEAMsgs(VehicleCallBack cb, void *userData)
{
  if(cb)
  {
    this->ppsNMEAHandler.callback = cb;
    this->ppsNMEAHandler.userData = userData;
  }
}

void
HardwareSync::unsubscribeNMEAMsgs()
{
  this->ppsNMEAHandler.callback = 0;
  this->ppsNMEAHandler.userData = 0;
}

void
HardwareSync::subscribeUTCTime(VehicleCallBack cb, void *userData)
{
  if(cb)
  {
    this->ppsUTCTimeHandler.callback = cb;
    this->ppsUTCTimeHandler.userData = userData;
  }
}

void
HardwareSync::unsubscribeUTCTime()
{
  this->ppsUTCTimeHandler.callback = 0;
  this->ppsUTCTimeHandler.userData = 0;
}

void
HardwareSync::subscribeFCTimeInUTCRef(VehicleCallBack cb, void *userData)
{
  if(cb)
  {
    this->ppsUTCFCTimeHandler.callback = cb;
    this->ppsUTCFCTimeHandler.userData = userData;
  }
}

void
HardwareSync::unsubscribeFCTimeInUTCRef()
{
  this->ppsUTCFCTimeHandler.callback = 0;
  this->ppsUTCFCTimeHandler.userData = 0;
}

void
HardwareSync::subscribePPSSource(VehicleCallBack cb, void *userData)
{
  if(cb)
  {
    this->ppsSourceHandler.callback = cb;
    this->ppsSourceHandler.userData = userData;
  }
}

void
HardwareSync::unsubscribePPSSource()
{
  this->ppsSourceHandler.callback = 0;
  this->ppsSourceHandler.userData = 0;
}

void
HardwareSync::writeNMEA(const std::string &nmea)
{
  std::string head = nmea.substr(0,6);
  if(head == "$GPGSA")
  {
    GPGSAData.sentence = nmea;
    ++GPGSAData.seq;
    timespec_get(&GPGSAData.timestamp, TIME_UTC);
    GPGSAFlag.store(true, std::memory_order_release);
  }
  else if(head == "$GLGSA")
  {
    GLGSAData.sentence = nmea;
    ++GLGSAData.seq;
    timespec_get(&GLGSAData.timestamp, TIME_UTC);
    GLGSAFlag.store(true, std::memory_order_release);
  }
  else if(head == "$GAGSA")
  {
    GAGSAData.sentence = nmea;
    ++GAGSAData.seq;
    timespec_get(&GAGSAData.timestamp, TIME_UTC);
    GAGSAFlag.store(true, std::memory_order_release);
  }
  else if(head == "$BDGSA")
  {
    BDGSAData.sentence = nmea;
    ++BDGSAData.seq;
    timespec_get(&BDGSAData.timestamp, TIME_UTC);
    BDGSAFlag.store(true, std::memory_order_release);
  }
  else if(head == "$GPRMC")
  {
    GPRMCData.sentence = nmea;
    ++GPRMCData.seq;
    timespec_get(&GPRMCData.timestamp, TIME_UTC);
    GPRMCFlag.store(true, std::memory_order_release);
  }
  else if(head.substr(0,3) == "UTC")
  {
    UTCData.sentence = nmea;
    ++UTCData.seq;
    timespec_get(&UTCData.timestamp, TIME_UTC);
    UTCFlag.store(true, std::memory_order_release);
  }
  else
  {
    DERROR("Cannot recognize the NMEA msg received\n");
  }
}

void
HardwareSync::writeData(const uint8_t cmdID, const RecvContainer *recvContainer)
{
  if(OpenProtocolCMD::CMDSet::HardwareSync::ppsNMEAGPSGSA[1] <= cmdID &&
    cmdID <= OpenProtocolCMD::CMDSet::HardwareSync::ppsUTCTime[1] )
  {
    int length = recvContainer->recvInfo.len-OpenProtocol::PackageMin-4;
    uint8_t rawBuf[length];
    memcpy(rawBuf, recvContainer->recvData.raw_ack_array, length);
    writeNMEA(std::string((char*)rawBuf, length));
  }
  else if (cmdID == OpenProtocolCMD::CMDSet::HardwareSync::ppsUTCFCTimeRef[1])
  {
    fcTimeInUTC = recvContainer->recvData.fcTimeInUTC;
    fcTimeFlag.store(true, std::memory_order_release);
  }
  else if (cmdID == OpenProtocolCMD::CMDSet::HardwareSync::ppsSource[1])
  {
    ppsSourceType = (PPSSource)recvContainer->recvData.ppsSourceType;
    ppsSourceFlag.store(true, std::memory_order_release);
  }
  else
  {
    DERROR("Unable to recognize this cmd ID\n");
  }
}

bool
HardwareSync::getUTCTime(NMEAData &utc)
{
  return writeDataHelper<NMEAData>(UTCFlag, UTCData, utc);
}

bool
HardwareSync::getFCTimeInUTCRef(DJI::OSDK::ACK::FCTimeInUTC &_fcTimeInUTC)
{
  return writeDataHelper<ACK::FCTimeInUTC>(fcTimeFlag, fcTimeInUTC, _fcTimeInUTC);
}

bool
HardwareSync::getPPSSource(PPSSource &source)
{
  return writeDataHelper<PPSSource>(ppsSourceFlag, ppsSourceType, source);
}

bool
HardwareSync::getNMEAMsg(NMEAType type, NMEAData &nmea)
{
  bool result = false;
  switch(type)
  {
    case GPGSA:{
      result = writeDataHelper<NMEAData>(GPGSAFlag, GPGSAData, nmea);
    }
      break;
    case GLGSA:{
      result = writeDataHelper<NMEAData>(GLGSAFlag, GLGSAData, nmea);
    }
      break;
    case GAGSA:{
      result = writeDataHelper<NMEAData>(GAGSAFlag, GAGSAData, nmea);
    }
      break;
    case BDGSA:{
      result = writeDataHelper<NMEAData>(BDGSAFlag, BDGSAData, nmea);
    }
      break;
    case GPRMC:{
      result = writeDataHelper<NMEAData>(GPRMCFlag, GPRMCData, nmea);
    }
      break;
    default:
      break;
  }
  return result;
}