// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*========================================================================*
Modifications copyright (C) 2016  DJI.  All rights reserved.
==========================================================================*/

#include "vvPacketSender.h"
#include "vtkPacketFileReader.h"

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>


//-----------------------------------------------------------------------------
class vvPacketSender::vvInternal
{
public:
  vvInternal(std::string destinationIp,
             int lidarPort,
             int positionPort) :
    LIDARSocket(0),
    PositionSocket(0),
    PacketReader(0),
    Done(false),
    PacketCount(0),
    LIDAREndpoint(boost::asio::ip::address_v4::from_string(destinationIp), lidarPort),
    PositionEndpoint(boost::asio::ip::address_v4::from_string(destinationIp), positionPort)
  {
  }

  boost::asio::ip::udp::socket* LIDARSocket;
  boost::asio::ip::udp::socket* PositionSocket;

  vtkPacketFileReader* PacketReader;
  bool Done;
  size_t PacketCount;
  boost::asio::ip::udp::endpoint LIDAREndpoint;
  boost::asio::ip::udp::endpoint PositionEndpoint;
  boost::asio::io_service IOService;
  };

//-----------------------------------------------------------------------------
vvPacketSender::vvPacketSender(std::string pcapfile,
                               std::string destinationIp,
                               int lidarPort,
                               int positionPort,
                               int pktSelection) :
  Internal(new vvPacketSender::vvInternal(destinationIp, lidarPort, positionPort))
{
  this->Internal->PacketReader = new vtkPacketFileReader;
  this->Internal->PacketReader->Open(pcapfile);
  if(!this->Internal->PacketReader->IsOpen())
    {
    throw std::runtime_error("Unable to open packet file");
    }

  this->Internal->LIDARSocket = new boost::asio::ip::udp::socket(this->Internal->IOService);

  boost::system::error_code ec;
  this->Internal->LIDARSocket->set_option(boost::asio::socket_base::reuse_address(true), ec);

  this->Internal->LIDARSocket->open(this->Internal->LIDAREndpoint.protocol());

  this->Internal->PositionSocket = new boost::asio::ip::udp::socket(this->Internal->IOService);
  this->Internal->PositionSocket->open(this->Internal->PositionEndpoint.protocol());

  dataLength=0;
  timeSinceStart=0;
  onePkt=0;
  whichPkt=pktSelection;

}

//-----------------------------------------------------------------------------
vvPacketSender::~vvPacketSender()
{

  delete this->Internal->LIDARSocket;
  delete this->Internal->PositionSocket;
  delete this->Internal;
}

//-----------------------------------------------------------------------------
void vvPacketSender::pumpPacket()
{
  if(this->Internal->Done)
    {
    return;
    }

  if (onePkt<=whichPkt)
  {

      if (!this->Internal->PacketReader->NextPacket(data, dataLength, timeSinceStart))
        {
        this->Internal->Done = true;
        return;
        }

  }
  // Recurse until we get to the right kind of packet
  if (dataLength == 1206)
    {
      if(onePkt >= whichPkt) {
        ++this->Internal->PacketCount;
        size_t bytesSent = this->Internal->LIDARSocket->send_to(boost::asio::buffer(data, dataLength),
                                                                this->Internal->LIDAREndpoint);
      }

      if(onePkt<= whichPkt && whichPkt) onePkt++;

    }

  if( (dataLength == 512) )
    {
    size_t bytesSent = this->Internal->PositionSocket->send_to(boost::asio::buffer(data, dataLength),
                                                               this->Internal->PositionEndpoint);
    }
}

//-----------------------------------------------------------------------------
size_t vvPacketSender::packetCount() const
{
  return this->Internal->PacketCount;
}

//-----------------------------------------------------------------------------
bool vvPacketSender::done() const
{
  return this->Internal->Done;
}
